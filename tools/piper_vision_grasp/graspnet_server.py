"""Local GraspNet inference worker. No camera, cloud credentials or robot commands."""
import hashlib
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from scipy.spatial import cKDTree

from .graspnet import MAX_BODY, decode_cloud, validate_cloud


def target_candidates(points, target, centers, widths, max_width):
    nearest = cKDTree(points).query(centers)[1]
    belongs = target[nearest]
    fits = np.isfinite(widths) & (widths > 0) & (widths <= max_width)
    return belongs & fits, dict(background=int((~belongs).sum()), width=int((belongs & ~fits).sum()))


class Model:
    def __init__(self, settings):
        import torch
        source = Path(os.environ.get('PIPER_GRASPNET_SOURCE', settings['source'])).expanduser().resolve()
        checkpoint = Path(os.environ.get('PIPER_GRASPNET_CHECKPOINT', settings['checkpoint'])).expanduser().resolve()
        if not source.is_dir() or not checkpoint.is_file():
            raise ValueError('GraspNet source/checkpoint missing; see runbook')
        if not torch.cuda.is_available():
            raise ValueError('GraspNet CUDA device unavailable')
        for directory in ('models', 'utils', 'pointnet2', 'knn'):
            sys.path.insert(0, str(source / directory))
        from graspnet import GraspNet, pred_decode
        from graspnetAPI import GraspGroup
        from collision_detector import ModelFreeCollisionDetector
        torch.set_num_threads(4)
        self.torch = torch; self.decode = pred_decode; self.group = GraspGroup
        self.collision = ModelFreeCollisionDetector
        self.net = GraspNet(input_feature_dim=0, num_view=300, num_angle=12, num_depth=4,
                            cylinder_radius=.05, hmin=-.02, hmax_list=[.01, .02, .03, .04], is_training=False).cuda()
        # Only tensor/state data from the explicitly installed official checkpoint is accepted.
        state = torch.load(str(checkpoint), map_location='cpu', weights_only=True)
        self.net.load_state_dict(state['model_state_dict'], strict=True)
        self.net.eval()
        source_sha = subprocess.check_output(['git', '-C', str(source), 'rev-parse', 'HEAD'], text=True).strip()
        patch = subprocess.check_output(['git', '-C', str(source), 'diff', '--binary'])
        self.provenance = dict(checkpoint=str(checkpoint), checkpoint_sha256=hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
                               source_commit=source_sha, compatibility_patch_sha256=hashlib.sha256(patch).hexdigest(),
                               torch=torch.__version__, cuda=torch.version.cuda, device=torch.cuda.get_device_name(0))

    def infer(self, points, target, max_width):
        points, target = validate_cloud(points, target)
        if not np.isfinite(max_width) or not 0 < max_width <= .1:
            raise ValueError('invalid maximum gripper opening')
        started = time.monotonic(); torch = self.torch
        torch.cuda.reset_peak_memory_stats()
        # A fixed seed makes saved-scene replay comparable without changing the official sampler.
        rng = np.random.RandomState(0)
        if len(points) >= 20000:
            indices = rng.choice(len(points), 20000, replace=False)
        else:
            indices = np.concatenate((np.arange(len(points)), rng.choice(len(points), 20000-len(points), replace=True)))
        with torch.inference_mode():
            output = self.decode(self.net(dict(point_clouds=torch.from_numpy(points[indices][None]).cuda())))[0]
            group = self.group(output.cpu().numpy())
        counts = dict(predicted=len(group))
        if len(group):
            keep, rejected = target_candidates(points, target, group.translations, group.widths, max_width)
            counts.update(rejected); group = group[keep]
        if len(group):
            detector = self.collision(points, voxel_size=.01)
            collisions = detector.detect(group, approach_dist=.05, collision_thresh=.01)
            counts['collision'] = int(collisions.sum()); group = group[~collisions]
        counts['before_nms'] = len(group)
        if len(group):
            group = group.nms(); group.sort_by_score(); group = group[:50]
        rows = [dict(score=float(g.score), rotation=g.rotation_matrix.tolist(), translation=g.translation.tolist(),
                     width=float(g.width), height=float(g.height), depth=float(g.depth)) for g in group]
        return dict(frame='color_optical', units='m', candidates=rows, counts=counts, provenance=self.provenance,
                    inference_seconds=time.monotonic()-started, peak_cuda_bytes=torch.cuda.max_memory_allocated(),
                    collision_scope='official generic gripper + 50 mm approach; not Piper full path clearance')


def handler(model, events):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def do_POST(self):
            self.connection.settimeout(30)
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if self.path != '/grasps' or not 0 < size <= MAX_BODY:
                    raise ValueError('invalid endpoint or payload size')
                row = json.loads(self.rfile.read(size))
                points, target = decode_cloud(row)
                result = model.infer(points, target, float(row['max_width_m'])); status = 200
                events.emit('MODEL_RESULT', 'GraspNet scene inference complete', counts=result['counts'],
                            inference_seconds=result['inference_seconds'])
            except Exception as exc:
                result = dict(error=str(exc)); status = 503
                events.emit('WAIT_VISION', str(exc))
            data = json.dumps(result, allow_nan=False).encode()
            self.send_response(status); self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(data))); self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass
    return Handler


def serve(settings, events):
    while True:
        try:
            events.emit('MODEL_LOADING', 'loading local GraspNet RealSense checkpoint')
            model = Model(settings)
            with HTTPServer(('127.0.0.1', settings['port']), handler(model, events)) as server:
                events.emit('MODEL_READY', 'GraspNet localhost worker ready', port=settings['port'], **model.provenance)
                server.serve_forever()
        except Exception as exc:
            events.emit('WAIT_VISION', str(exc))
            time.sleep(2)
