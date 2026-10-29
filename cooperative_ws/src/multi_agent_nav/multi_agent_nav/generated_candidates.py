"""Map-only observation geometry and image/ID selection; no ROS or motion authority."""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
import yaml

from .config import map_file_info, valid_numbers
from .region_guard import edges, inside, segments_intersect, path_intersects
from .models import Pose2D
from .experiment_records import numbered_directory


def rotation_radius(footprint, padding=0.):
    if (not isinstance(footprint, list) or len(footprint)<3
            or any(not valid_numbers(p,2) for p in footprint)
            or not valid_numbers([padding],1) or padding<0):
        raise ValueError('finite footprint and nonnegative padding required')
    radius=max(math.hypot(x,y) for x,y in footprint)+padding
    if radius<=0:raise ValueError('positive footprint radius required')
    return radius


def circle_free(raw, x, y, radius, resolution, blocked):
    """Cell-square intersection with the complete rotation disk, including map edges."""
    height,width=raw.shape
    if x-radius<0 or y-radius<0 or x+radius>=width*resolution or y+radius>=height*resolution:
        return False
    gx0,gy0=int((x-radius)/resolution),int((y-radius)/resolution)
    gx1,gy1=int((x+radius)/resolution)+1,int((y+radius)/resolution)+1
    gy,gx=np.mgrid[gy0:gy1,gx0:gx1]
    dx=np.maximum(np.abs((gx+.5)*resolution-x)-resolution/2,0)
    dy=np.maximum(np.abs((gy+.5)*resolution-y)-resolution/2,0)
    disk=dx*dx+dy*dy<=radius*radius
    return not np.any(disk & blocked(raw[gy0:gy1,gx0:gx1]))


class GeneratedCandidates:
    def __init__(self,config,robot_id='go2'):
        self.robot_id=robot_id
        robot=config[f'robot_{robot_id}'];options=robot['generated_candidates']
        distance=options['observation_distance']
        self.coverage_range=options.get('coverage_range',0.) if robot_id=='go2' else 0.
        if not valid_numbers([self.coverage_range],1) or self.coverage_range<0:
            raise ValueError('finite nonnegative coverage_range required')
        if not valid_numbers([distance],1) or distance<=0:
            raise ValueError('positive finite observation_distance required')
        self.radius=rotation_radius(robot['footprint'],robot.get('footprint_padding',0.))
        self.map_path=Path(robot['map_path']).resolve()
        self.semantic_path=Path(options['semantic_yaml']).resolve() if options.get('semantic_yaml') else None
        info=map_file_info(self.map_path)
        identity=config['alignment']['robots'][robot_id]
        if (info['map_digest']!=robot['map_digest'] or identity['map_version']!='sha256:'+info['map_digest']):
            raise ValueError('generated map identity mismatch')
        source=yaml.safe_load(self.map_path.read_text())
        semantic_bytes=None
        if self.semantic_path is not None:
            try:semantic_bytes=self.semantic_path.read_bytes()
            except FileNotFoundError:pass
        self.furniture=[]
        if semantic_bytes is not None:
            try:semantic=yaml.safe_load(semantic_bytes)
            except yaml.YAMLError as error:raise ValueError('invalid semantic YAML') from error
            if (not isinstance(semantic,dict) or semantic.get('schema_version')!=1 or semantic.get('source_robot')!=robot_id
                    or semantic.get('frame_id')!='map' or semantic.get('units')!='meters'):
                raise ValueError('invalid semantic map frame/schema')
            for key in ('yaml_sha256','image_sha256'):
                if semantic['source_map'][key]!=info[key]:raise ValueError('semantic source digest mismatch')
            for key,other in (('yaml','yaml_path'),('image','image_path')):
                if Path(semantic['source_map'][key]).resolve()!=Path(info[other]):
                    raise ValueError('semantic source path mismatch')
            self.furniture=semantic['furniture']
        if (source.get('mode','trinary')!='trinary' or source.get('negate') not in (0,1)
                or not valid_numbers(source.get('origin'),3)
                or not valid_numbers([source.get('resolution')],1) or source['resolution']<=0
                or not valid_numbers([source.get('free_thresh'),source.get('occupied_thresh')],2)
                or not 0<=source['free_thresh']<source['occupied_thresh']<=1):
            raise ValueError('valid trinary occupancy map required')
        with Image.open(info['image_path']) as image:
            pixels=np.array(image.convert('L'))
        occupancy=pixels/255. if source['negate'] else 1.-pixels/255.
        # Trinary cells at either threshold are unknown, as in Nav2 map_io.
        self.raw=np.flipud(np.where(occupancy<source['free_thresh'],0,
                                   np.where(occupancy>source['occupied_thresh'],254,255))).copy()
        self.source=dict(source,image=info['image_path'],width=pixels.shape[1],height=pixels.shape[0])
        if not isinstance(self.furniture,list):raise ValueError('furniture list required')
        self.points=[];ids=set()
        for item in self.furniture:
            if not isinstance(item,dict):raise ValueError('furniture must be a mapping')
            fid=item.get('id');poly=item.get('polygon')
            if not isinstance(fid,str) or not fid.strip() or fid in ids:
                raise ValueError('invalid or duplicate furniture ID')
            ids.add(fid)
            if (not isinstance(poly,list) or len(poly)<3 or any(not valid_numbers(p,2) for p in poly)
                    or len({tuple(p) for p in poly})!=len(poly)):
                raise ValueError('finite simple furniture polygon required')
            pairs=list(edges(poly))
            if any(segments_intersect(a,b,c,d) for i,(a,b) in enumerate(pairs)
                   for j,(c,d) in enumerate(pairs) if j>i+1 and not (i==0 and j==len(pairs)-1)):
                raise ValueError('self-intersecting furniture polygon')
            area=sum(a[0]*b[1]-b[0]*a[1] for a,b in pairs)
            if abs(area)<1e-9:raise ValueError('degenerate furniture polygon')
            if type(item.get('enabled',True)) is not bool:raise ValueError('invalid furniture enabled flag')
        for item in self.furniture:
            fid=item['id'];poly=item['polygon'];pairs=list(edges(poly))
            area=sum(a[0]*b[1]-b[0]*a[1] for a,b in pairs)
            if not item.get('enabled',True):continue
            for index,(a,b) in enumerate(pairs,1):
                dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
                nx,ny=(dy/length,-dx/length) if area>0 else (-dy/length,dx/length)
                x,y=(a[0]+b[0])/2+distance*nx,(a[1]+b[1])/2+distance*ny
                if any(path_intersects([Pose2D(x,y)],f['polygon'],self.radius) for f in self.furniture):continue
                mx,my=self.map_xy(x,y)
                if not circle_free(self.raw,mx,my,self.radius,source['resolution'],lambda a:a>=253):continue
                if any(math.hypot(x-p['pose'][0],y-p['pose'][1])<source['resolution'] for p in self.points):continue
                self.points.append(dict(id=f'{fid}/edge_{index}',furniture_id=fid,
                                        pose=[x,y,math.atan2(-ny,-nx)],side=index))
        resolution=source['resolution']
        step=math.ceil(max(1.,2*self.radius)/resolution)
        spacing=step*resolution
        ox,oy,yaw=source['origin'];c,s=math.cos(yaw),math.sin(yaw)
        furniture_points=list(self.points)
        for row in range(step//2,self.raw.shape[0],step):
            for col in range(step//2,self.raw.shape[1],step):
                mx,my=(col+.5)*resolution,(row+.5)*resolution
                if not circle_free(self.raw,mx,my,self.radius,resolution,lambda a:a>=253):continue
                x,y=ox+c*mx-s*my,oy+s*mx+c*my
                if any(path_intersects([Pose2D(x,y)],f['polygon'],self.radius) for f in self.furniture):continue
                if any(math.hypot(x-p['pose'][0],y-p['pose'][1])<spacing/2 for p in furniture_points):continue
                self.points.append(dict(id=f'free/{row}/{col}',furniture_id='',pose=[x,y,yaw]))
        self.digests={self.map_path:info['yaml_sha256'],Path(info['image_path']):info['image_sha256']}
        if self.semantic_path is not None:
            self.digests[self.semantic_path]=hashlib.sha256(semantic_bytes).hexdigest() if semantic_bytes is not None else None

    def unchanged(self):
        for path,digest in self.digests.items():
            try:actual=hashlib.sha256(path.read_bytes()).hexdigest()
            except FileNotFoundError:actual=None
            except OSError:return False
            if actual!=digest:return False
        return True

    def map_xy(self,x,y):
        ox,oy,yaw=self.source['origin'];c,s=math.cos(yaw),math.sin(yaw)
        return c*(x-ox)+s*(y-oy),-s*(x-ox)+c*(y-oy)

    def covered_points(self,pose,angles):
        """Conservative static-map visibility after a successful full sweep."""
        # shortcut: static visibility and <=45-degree scan gaps approximate coverage; use measured camera/depth coverage if needed.
        directions=sorted({a%(2*math.pi) for a in angles})
        if (not self.coverage_range or not directions or
                max(b-a for a,b in zip(directions,directions[1:]+[directions[0]+2*math.pi]))>math.pi/4+1e-6):
            return []
        start=self.map_xy(pose.x,pose.y);res=self.source['resolution']
        covered=[]
        for point in self.points:
            x,y=point['pose'][:2]
            if math.hypot(x-pose.x,y-pose.y)>self.coverage_range:continue
            end=self.map_xy(x,y)
            if any(not (0<=px<self.raw.shape[1]*res and 0<=py<self.raw.shape[0]*res) for px,py in (start,end)):continue
            if any(path_intersects([pose,Pose2D(x,y)],f['polygon'],0.) for f in self.furniture):continue
            # Include adjacent cells when the sightline lies exactly on a cell edge.
            left,bottom=[max(0,math.floor(min(a,b)/res)-1) for a,b in zip(start,end)]
            right,top=[math.floor(max(a,b)/res) for a,b in zip(start,end)]
            blocked=False
            for row,col in np.argwhere(self.raw[bottom:top+1,left:right+1]!=0):
                gx,gy=(col+left)*res,(row+bottom)*res
                square=[(gx,gy),(gx+res,gy),(gx+res,gy+res),(gx,gy+res)]
                if inside(start,square) or inside(end,square) or any(segments_intersect(start,end,a,b) for a,b in edges(square)):
                    blocked=True;break
            if not blocked:covered.append(point['id'])
        return covered

    def pixel(self,x,y,scale=3):
        mx,my=self.map_xy(x,y);res=self.source['resolution']
        return (mx/res-.5)*scale,(self.source['height']-.5-my/res)*scale


def render_candidate_camera(snapshot,candidates,directory):
    """Annotate an existing RGB frame; projected map points may be occluded."""
    from vlm_nav.geometry import realsense_intrinsics
    fx,fy,cx,cy=snapshot.intrinsics
    matrix=np.asarray(snapshot.transform_matrix,dtype=float)
    if (not np.isfinite([fx,fy,cx,cy]).all() or min(fx,fy)<=0 or
            matrix.shape!=(4,4) or not np.isfinite(matrix).all()):
        raise ValueError('invalid candidate camera geometry')
    if snapshot.raw_depth_snapshot is not None:
        # Match the existing raw-depth geometry's verified distortion support.
        realsense_intrinsics(snapshot.raw_depth_snapshot[2]['color_info'])
    inverse=np.linalg.inv(matrix)
    image=Image.fromarray(snapshot.rgb).convert('RGBA');width,height=image.size
    overlay=Image.new('RGBA',image.size,(0,0,0,0));draw=ImageDraw.Draw(overlay)
    font_path=Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
    font=ImageFont.truetype(str(font_path),16) if font_path.exists() else ImageFont.load_default()
    projections=[];unprojected=[]
    for candidate in candidates:
        x,y=candidate['pose'][:2];label=candidate['display_id']
        # shortcut: map z=0 is an approximate display plane; use measured floor height for precise overlays.
        point=inverse@np.array([x,y,0.,1.])
        if not np.isfinite(point).all() or point[2]<=0:
            unprojected.append(label);continue
        u,v=fx*point[0]/point[2]+cx,fy*point[1]/point[2]+cy
        if not (0<=u<width and 0<=v<height):
            unprojected.append(label);continue
        u,v=round(u),round(v)
        if not (0<=u<width and 0<=v<height):
            unprojected.append(label);continue
        projections.append(dict(candidate_id=label,pixel=[u,v]))
        draw.ellipse((u-7,v-7,u+7,v+7),outline=(240,140,20,190),width=2)
        box=draw.textbbox((0,0),label,font=font)
        w,h=box[2]+6,box[3]+4
        left=max(0,min(u+10,width-w));top=max(0,min(v-12,height-h))
        draw.rectangle((left,top,left+w,top+h),fill=(255,255,255,210))
        draw.text((left+3,top+1),label,font=font,fill=(150,75,0,255))
    annotated=Image.alpha_composite(image,overlay).convert('RGB')
    output=Image.new('RGB',(width,height+32),'white');output.paste(annotated,(0,0))
    ImageDraw.Draw(output).text((5,height+3),
        'P IDs = map-plane projections\nOcclusion unchecked (map z=0)',fill='black')
    directory=Path(directory)
    Image.fromarray(snapshot.rgb).save(directory/'camera_rgb.png')
    path=directory/'camera.png';output.save(path)
    metadata=dict(image_sequence=snapshot.sequence,frame_id=snapshot.frame_id,
        image_stamp_ns=snapshot.stamp.sec*10**9+snapshot.stamp.nanosec,
        projection_plane_z=0.,occlusion_checked=False,intrinsics=list(snapshot.intrinsics),
        transform_matrix=matrix.tolist(),projections=projections,unprojected_candidate_ids=unprojected)
    if snapshot.raw_depth_snapshot is not None:
        metadata['rgb_stamp_ns']=snapshot.raw_depth_snapshot[2]['rgb_stamp_ns']
    (directory/'camera_projection.json').write_text(json.dumps(metadata,allow_nan=False,indent=2)+'\n')
    return str(path)


def render_selection(catalog,candidates,records,robot_pose,root,*,camera_snapshot=None):
    """One overlay is composited onto both maps so every selectable marker matches."""
    scale=3;width,height=catalog.source['width']*scale,catalog.source['height']*scale
    columns=max(1,width//90)
    footer=math.ceil((len(candidates)+1)/columns)*25+80
    occupancy=Image.fromarray(np.flipud(np.where(catalog.raw==0,254,np.where(catalog.raw==255,205,0))).astype('uint8')).convert('RGBA')
    occupancy=occupancy.resize((width,height),getattr(Image,'Resampling',Image).NEAREST)
    bases=[Image.new('RGBA',(width,height+footer),'white') for _ in range(2)]
    bases[0].paste(occupancy,(0,0));bases[1].paste(occupancy,(0,0))
    font_path=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if not font_path.exists():font_path=Path('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf')
    font=ImageFont.truetype(str(font_path),14) if font_path.exists() else ImageFont.load_default()
    draw=ImageDraw.Draw(bases[1]);reserved=[]
    for item in catalog.furniture:
        polygon=[catalog.pixel(*p) for p in item['polygon']]
        draw.polygon(polygon,fill=(185,215,190,255),outline=(45,85,55,255))
        cx,cy=sum(p[0] for p in polygon)/len(polygon),sum(p[1] for p in polygon)/len(polygon)
        label=item['id']+'\n'+str(item.get('name') or item.get('category') or '')
        box=draw.multiline_textbbox((cx,cy),label,font=font,anchor='mm',align='center')
        if any(box[0]<b[2] and box[2]>b[0] and box[1]<b[3] and box[3]>b[1] for b in reserved):
            raise ValueError('overlapping furniture labels')
        draw.multiline_text((cx,cy),label,fill='black',font=font,anchor='mm',align='center')
        reserved.append(box)
    overlay=Image.new('RGBA',bases[0].size,(0,0,0,0));draw=ImageDraw.Draw(overlay)
    blue,orange,gray=(30,100,230),(240,140,20),(125,125,125)
    rx,ry=catalog.pixel(robot_pose.x,robot_pose.y)
    if not (0<=rx<width and 0<=ry<height):raise ValueError('robot outside map image')
    robot_label='Go2' if catalog.robot_id=='go2' else 'Ranger'
    marks=[(rx,ry,robot_label,blue)]
    marks.extend((*catalog.pixel(*p['pose'][:2]),p['display_id'],orange) for p in candidates)
    marks.extend((*catalog.pixel(*p['pose'][:2]),'observed',gray) for p in catalog.points
                 if records.get(p['id'],{}).get('status') in ('observed','covered'))
    footer_index=0
    for x,y,label,color in marks:
        reserved.append((x-8,y-8,x+8,y+8))
    for x,y,label,color in marks:
        draw.ellipse((x-5,y-5,x+5,y+5),fill=color)
        if label=='observed':continue
        box=None
        # Prefer unused furniture interior, then wide free space so narrow passages stay legible.
        w=draw.textbbox((0,0),label,font=font)[2]+8;h=22
        for interior_only in (True,False):
            for offset in range(15,121,15):
                for dx,dy in ((1,-1),(-1,-1),(1,1),(-1,1)):
                    left,top=x+dx*offset-(w if dx<0 else 0),y+dy*offset
                    trial=(left,top,left+w,top+h)
                    if (left<0 or top<0 or trial[2]>=width or trial[3]>=height
                            or any(left<b[2]+3 and trial[2]+3>b[0] and top<b[3]+3 and trial[3]+3>b[1] for b in reserved)):
                        continue
                    corners=[(left,top),(left+w,top),(left+w,top+h),(left,top+h)]
                    interior=any(all(inside(c,polygon) for c in corners)
                        for polygon in [[catalog.pixel(*p) for p in f['polygon']] for f in catalog.furniture])
                    mx=((left+w/2)/scale+.5)*catalog.source['resolution']
                    my=(catalog.source['height']-.5-(top+h/2)/scale)*catalog.source['resolution']
                    open_space=circle_free(catalog.raw,mx,my,catalog.radius+max(w,h)/scale*catalog.source['resolution']/2,
                                           catalog.source['resolution'],lambda cells:cells>=253)
                    if not (interior if interior_only else open_space):continue
                    box=trial;break
                if box:break
            if box:break
        if box is None:
            row,column=divmod(footer_index,columns);footer_index+=1
            left=10+column*90
            box=(left,height+8+row*25,left+w,height+30+row*25)
        reserved.append(box)
        draw.line((x,y,(box[0]+box[2])/2,(box[1]+box[3])/2),fill=color,width=1)
        draw.rectangle(box,fill=(255,255,255,245))
        draw.text((box[0]+4,box[1]+2),label,fill=color,font=font)
    yaw=robot_pose.yaw-catalog.source['origin'][2]
    ax,ay=rx+22*math.cos(yaw),ry-22*math.sin(yaw)
    draw.line((rx,ry,ax,ay),fill=blue,width=3)
    for sign in (-1,1):
        angle=yaw+sign*.5
        draw.line((ax,ay,ax-8*math.cos(angle),ay+8*math.sin(angle)),fill=blue,width=3)
    for offset,label,color in ((65,robot_label+': blue circle and heading',blue),(45,'P IDs: orange / selectable',orange),
                               (25,'Gray: observed / cannot select',gray)):
        draw.text((10,height+footer-offset),label,fill=color,font=font)
    directory=numbered_directory(root)
    paths=[]
    for base,name in zip(bases,('occupancy.png','semantic.png')):
        path=directory/name;Image.alpha_composite(base,overlay).convert('RGB').save(path);paths.append(str(path))
    if camera_snapshot is not None:
        paths.append(render_candidate_camera(camera_snapshot,candidates,directory))
    return paths


def parse_candidate_selection(raw,allowed):
    def unique(pairs):
        result={}
        for key,value in pairs:
            if key in result:raise ValueError('duplicate candidate field')
            result[key]=value
        return result
    try:reply=json.loads(raw,object_pairs_hook=unique)
    except (TypeError,json.JSONDecodeError) as error:raise ValueError('candidate response must be JSON') from error
    if (not isinstance(reply,dict) or set(reply)!={'candidate_id','reason'}
            or not isinstance(reply['candidate_id'],str) or reply['candidate_id'] not in allowed
            or not isinstance(reply['reason'],str)):
        raise ValueError('invalid or unavailable candidate ID')
    reply['reason']=reply['reason'].strip()[:20]
    return reply


class GeneratedCandidatesVLM:
    def __init__(self,client):self.client=client

    def next_viewpoint(self,snapshot):
        mapping={p['display_id']:p for p in snapshot['candidates']}
        reply=parse_candidate_selection(json.dumps(self.client.choose_candidate(snapshot)),mapping)
        return dict(mapping[reply['candidate_id']],selection_reason=reply['reason'])
