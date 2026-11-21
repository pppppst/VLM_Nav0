"""Localhost Qwen+SAM worker; no robot modules or CAN imports."""
import base64
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import os
from pathlib import Path
import time
from urllib import request, error

import cv2
import numpy as np

from .semantic import target_prompt, parse_detection, encode_mask


class Model:
    def __init__(self,settings):
        import torch
        from segment_anything import sam_model_registry, SamPredictor
        self.settings=settings
        self.key=os.environ.get('PIPER_VLM_API_KEY') or os.environ.get('DASHSCOPE_API_KEY')
        if not self.key:raise ValueError('set PIPER_VLM_API_KEY or DASHSCOPE_API_KEY for model server')
        checkpoint=Path(os.environ.get('PIPER_SAM_CHECKPOINT',settings['sam_checkpoint'])).expanduser()
        if not checkpoint.is_file():raise ValueError('SAM checkpoint not found: '+str(checkpoint))
        torch.set_num_threads(settings.get('cpu_threads',4))
        device=os.environ.get('PIPER_SAM_DEVICE','cuda' if torch.cuda.is_available() else 'cpu')
        model=sam_model_registry['vit_b'](checkpoint=str(checkpoint)).to(device=device)
        self.predictor=SamPredictor(model)
        self.device=device

    def detect(self,rgb,target):
        started=time.monotonic()
        _,png=cv2.imencode('.png',cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
        payload=dict(model=self.settings['model'],temperature=0.,messages=[dict(role='user',content=[
            dict(type='text',text=target_prompt(target)),
            dict(type='image_url',image_url=dict(url='data:image/png;base64,'+base64.b64encode(png).decode()))])])
        req=request.Request(self.settings['base_url'].rstrip('/')+'/chat/completions',
                            data=json.dumps(payload).encode(),headers={'Content-Type':'application/json','Authorization':'Bearer '+self.key})
        try:
            with request.urlopen(req,timeout=self.settings['timeout_s']) as response:result=json.load(response)
        except error.HTTPError as exc:
            raise RuntimeError('vision model HTTP status '+str(exc.code)) from None
        row=parse_detection(result['choices'][0]['message']['content'],rgb.shape[:2])
        self.predictor.set_image(rgb)
        masks,_,_=self.predictor.predict(box=np.asarray(row['bbox_px']),multimask_output=False)
        row['mask_png']=encode_mask(masks[0])
        row['model_seconds']=time.monotonic()-started
        return row


def serve(settings,events):
    events.emit('MODEL_LOADING','loading Qwen client and SAM weights')
    model=Model(settings)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*_):pass
        def do_POST(self):
            try:
                if self.path!='/detect':raise ValueError('unknown endpoint')
                row=json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                raw=cv2.imdecode(np.frombuffer(base64.b64decode(row['image_png'],validate=True),np.uint8),cv2.IMREAD_COLOR)
                if raw is None:raise ValueError('invalid input PNG')
                result=model.detect(cv2.cvtColor(raw,cv2.COLOR_BGR2RGB),row['target']);status=200
                events.emit('MODEL_RESULT','description localized and segmented',measured={k:v for k,v in result.items() if k!='mask_png'})
            except Exception as exc:
                # Keep credentials and raw provider responses out of both remote replies and logs.
                message=str(exc).replace(model.key,'[redacted]')
                result=dict(error=message);status=503
                events.emit('WAIT_VISION',message)
            data=json.dumps(result).encode()
            self.send_response(status);self.send_header('Content-Type','application/json');self.end_headers()
            try:self.wfile.write(data)
            except (BrokenPipeError,ConnectionResetError):pass
    with HTTPServer(('127.0.0.1',settings['port']),Handler) as server:
        events.emit('MODEL_READY','localhost worker ready',device=model.device,port=settings['port'])
        server.serve_forever()
