"""Description grounding and metric RGB-D geometry; model output is data, never robot code."""
import base64
import json
import os
from urllib import request, error

import cv2
import numpy as np

VERSION='semantic-rgbd-v1'


def target_prompt(target):
    return ('Locate the single intended object in this image. Color: {color}. Description: {description}. '
            'Return only JSON with bbox_2d [x1,y1,x2,y2], point_2d [x,y], and label. '
            'Coordinates are normalized to 0..1000. point_2d is a visible point near the object center '
            'Do not choose a gripper orientation or approach direction. '
            'If the requested object is absent return {{"found":false}}. '
            'Do not return robot commands or estimated metric dimensions.').format(**target)


def parse_detection(content, shape):
    text=content.strip()
    if text.startswith('```'):text=text.split('\n',1)[1].rsplit('```',1)[0]
    row=json.loads(text)
    if isinstance(row,list):
        if len(row)!=1:raise ValueError('detector did not select one target')
        row=row[0]
    if not isinstance(row,dict) or row.get('found') is False:raise ValueError('requested object not found')
    box=np.asarray(row['bbox_2d'],dtype=float);point=np.asarray(row['point_2d'],dtype=float)
    if box.shape!=(4,) or point.shape!=(2,) or not np.all(np.isfinite(np.r_[box,point])):
        raise ValueError('invalid detector coordinates')
    h,w=shape
    box=np.clip(box,0,1000)*np.array([w,h,w,h])/1000.
    x1,x2=sorted([int(box[0]),int(box[2])]);y1,y2=sorted([int(box[1]),int(box[3])])
    if x1>=x2 or y1>=y2:raise ValueError('empty detector box')
    point=np.clip(point/1000*np.array([w,h]),[0,0],[w-1,h-1]).astype(int)
    return dict(bbox_px=[x1,y1,x2,y2],keypoint_px=point.tolist(),label=str(row.get('label','')))


def encode_mask(mask):
    ok,png=cv2.imencode('.png',np.asarray(mask,dtype=np.uint8)*255)
    if not ok:raise ValueError('mask PNG encoding failed')
    return base64.b64encode(png).decode('ascii')


def decode_mask(encoded,shape):
    mask=cv2.imdecode(np.frombuffer(base64.b64decode(encoded,validate=True),np.uint8),cv2.IMREAD_GRAYSCALE)
    if mask is None or mask.shape!=tuple(shape):raise ValueError('model mask dimensions mismatch')
    return mask!=0


def locate_and_segment(rgb,config):
    ok,png=cv2.imencode('.png',cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
    if not ok:raise ValueError('image PNG encoding failed')
    body=json.dumps(dict(target=config['target'],image_png=base64.b64encode(png).decode('ascii'))).encode()
    endpoint=os.environ.get('PIPER_VISION_ENDPOINT',config['semantic']['endpoint'])
    req=request.Request(endpoint+'/detect',data=body,headers={'Content-Type':'application/json'})
    try:
        with request.urlopen(req,timeout=config['semantic']['timeout_s']) as response:row=json.load(response)
    except error.HTTPError as exc:
        try:reason=json.loads(exc.read()).get('error','model service failed')
        except Exception:reason='model service failed'
        raise RuntimeError(reason) from None
    row['mask']=decode_mask(row.pop('mask_png'),rgb.shape[:2])
    return row


def target_points(depth,camera,mask):
    """Exact segmented, depth-filtered optical-frame points used by the heuristic."""
    from .vision import intrinsics_valid, rays
    intrinsics_valid(camera,depth.shape)
    mask=np.asarray(mask,bool)
    if mask.shape!=depth.shape:raise ValueError('mask/depth shape mismatch')
    valid=mask & np.isfinite(depth) & (depth>0)
    yy,xx=np.nonzero(valid)
    pixels=np.column_stack([xx,yy]);z=depth[valid].astype(float)*camera['depth_scale']
    if not len(xx):return pixels,np.empty((0,3)),valid
    # SAM edge pixels can contain background depth; measure the target's robust depth population.
    q1,q3=np.percentile(z,[25,75]);spread=max(q3-q1,camera['depth_scale'])
    keep=(z>=q1-1.5*spread)&(z<=q3+1.5*spread)
    pixels,z=pixels[keep],z[keep]
    points=rays(pixels,camera)*z[:,None]
    return pixels,points,valid


def measure(rgb,depth,camera,config,selection):
    from .vision import wait
    mask=np.asarray(selection['mask'],bool)
    if mask.shape!=depth.shape or rgb.shape[:2]!=depth.shape:raise ValueError('mask/RGB/depth shape mismatch')
    pixels,points,valid=target_points(depth,camera,mask)
    if not len(points):return wait('WAIT_DEPTH','target mask has no valid depth')
    transform=np.asarray(config['extrinsics']['T_base_color_optical'])
    world=points@transform[:3,:3].T+transform[:3,3]
    # Language-model point jitter is not object motion; use the segmented geometry's center.
    key=np.rint((pixels.min(axis=0)+pixels.max(axis=0))/2).astype(int)
    nearest=np.argsort(np.sum((pixels-key)**2,axis=1))[:25]
    center=np.median(points[nearest],axis=0)
    x1,y1,x2,y2=selection['bbox_px']
    corners=[]
    for pixel in ([x1,y1],[x2,y1],[x2,y2]):
        near=np.argsort(np.sum((pixels-pixel)**2,axis=1))[:25]
        corners.append(np.median(world[near],axis=0))
    # Image aspect ratio can select a vertical edge; compare measured horizontal spans.
    edges=np.array([corners[0]-corners[1],corners[1]-corners[2]])
    edges[:,2]=0
    axis=edges[np.argmax(np.linalg.norm(edges,axis=1))]
    axis=axis/np.linalg.norm(axis) if np.linalg.norm(axis)>1e-6 else np.array([0.,1.,0.])
    if axis[1]<0:axis=-axis
    low,high=np.min(world,axis=0),np.max(world,axis=0)
    sizes=np.sort(high-low)
    width=float(np.ptp(world@axis))
    image=cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR)
    cv2.rectangle(image,(x1,y1),(x2,y2),(0,255,0),2)
    cv2.circle(image,tuple(map(int,key)),4,(0,0,255),-1)
    return dict(status='DETECTED',reason='description/SAM target with measured RGB-D geometry',
                point_color_m=center.tolist(),width_m=float(sizes[-2]),height_m=float(sizes[-1]),
                dimensions_base_m=(high-low).tolist(),grasp_width_m=width,grasp_axis_base=axis.tolist(),
                visible_hull_base_xy_m=cv2.convexHull(world[:,:2].astype(np.float32)).reshape(-1,2).tolist(),
                normal_color=(transform[:3,:3].T@np.array([0.,0.,1.])).tolist(),
                long_axis_color=(transform[:3,:3].T@axis).tolist(),
                depth_valid_ratio=float(valid.sum()/max(mask.sum(),1)),detector_version=VERSION,
                target=config['target'],bbox_px=selection['bbox_px'],keypoint_px=key.tolist(),
                model_keypoint_px=selection['keypoint_px'],
                mask=mask.astype(np.uint8),annotation_bgr=image,
                model_seconds=selection.get('model_seconds'),size_scope='visible points; hidden thickness unknown')


def detect(rgb,depth,camera,config):
    return measure(rgb,depth,camera,config,locate_and_segment(rgb,config))
