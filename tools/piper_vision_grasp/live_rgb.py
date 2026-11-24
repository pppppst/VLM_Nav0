"""Single-owner RealSense stream with a best-effort thumbnail while grasping."""
from contextlib import contextmanager
from contextvars import ContextVar
from pathlib import Path
import threading

import cv2
import numpy as np

active_stream = ContextVar('piper_rgbd_stream', default=None)


def start(config):
    import pyrealsense2 as rs
    ctx = rs.context('{"inherit":false}')
    pipe, settings = rs.pipeline(ctx), rs.config()
    settings.enable_device(config['camera_serial'])
    camera = config['camera']
    for kind, fmt in ((rs.stream.color, rs.format.rgb8), (rs.stream.depth, rs.format.z16)):
        settings.enable_stream(kind, camera['width'], camera['height'], fmt, camera['fps'])
    try:
        profile = pipe.start(settings)
        for _ in range(camera['warmup_frames']):
            pipe.wait_for_frames(3000)
        return pipe, profile
    except BaseException:
        try: pipe.stop()
        except RuntimeError: pass
        raise


class SharedStream:
    def __init__(self, config, image):
        self.config, self.image = config, Path(image)
        self.lock = threading.Lock()
        self.stopped = threading.Event()
        self.camera = None

    def close_camera(self):
        if self.camera is not None:
            pipe, self.camera = self.camera[0], None
            try: pipe.stop()
            except RuntimeError: pass

    @contextmanager
    def recording(self):
        with self.lock:
            if self.camera is None:
                self.camera = start(self.config)
            try:
                yield self.camera
            except RuntimeError:
                self.close_camera()
                raise

    def publish(self):
        if not self.lock.acquire(blocking=False):
            return
        try:
            if self.camera is None:
                return
            frames = self.camera[0].poll_for_frames()
            color = frames.get_color_frame() if frames else None
            if not color:
                return
            small = cv2.resize(np.asanyarray(color.get_data()), (320,240), interpolation=cv2.INTER_AREA)
        finally:
            self.lock.release()
        temp = self.image.with_suffix('.tmp.jpg')
        if cv2.imwrite(str(temp), cv2.cvtColor(small, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY,75]):
            temp.replace(self.image)

    def preview(self):
        while not self.stopped.wait(2):
            try: self.publish()
            except (OSError, RuntimeError, cv2.error): continue


@contextmanager
def during_grasp(config, directory):
    stream = SharedStream(config, Path(directory)/'live-rgb.jpg')
    token = active_stream.set(stream)
    worker = threading.Thread(target=stream.preview, daemon=True)
    worker.start()
    try:
        yield
    finally:
        active_stream.reset(token)
        stream.stopped.set()
        worker.join()
        stream.close_camera()


@contextmanager
def recording(config):
    shared = active_stream.get()
    if shared is not None:
        with shared.recording() as camera:
            yield camera
    else:
        pipe, profile = start(config)
        try:
            yield pipe, profile
        finally:
            try: pipe.stop()
            except RuntimeError: pass
