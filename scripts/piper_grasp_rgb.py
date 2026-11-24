#!/usr/bin/env python3
"""Desktop RGB thumbnail viewer and SSH launcher for a single Piper grasp."""
import argparse
import base64
import io
import json
import os
from pathlib import Path
import queue
import shlex
import subprocess
import sys
import threading
import time
import uuid


def frames(directory):
    image = directory/'live-rgb.jpg'
    last = None
    while True:
        try:
            # The producer atomically replaces this file. Read time and pixels from one inode.
            with image.open('rb') as stream:
                stamp = os.fstat(stream.fileno()).st_mtime
                if stamp != last:
                    yield ('frame', stamp, base64.b64encode(stream.read()).decode('ascii'))
                    last = stamp
        except FileNotFoundError:
            pass
        if (directory/'result.json').is_file():
            yield ('done', 0, '')
            return
        time.sleep(.5)


def show(directory, host, launch=None):
    import tkinter as tk
    from PIL import Image, ImageTk
    root = tk.Tk()
    root.title('PiPER · 相机 RGB')
    blank = tk.PhotoImage(width=320,height=240)
    label = tk.Label(root, width=320, height=240, image=blank)
    label.pack()
    status = tk.StringVar(value='等待新的 RGB 帧')
    tk.Label(root, textvariable=status).pack()
    messages = queue.Queue(maxsize=2)
    remote = None
    if host:
        command = 'python3 piper-vision-current/scripts/piper_grasp_rgb.py --stream '+shlex.quote(str(directory))
        remote = subprocess.Popen(['ssh','-T','-o','BatchMode=yes',host,command], stdout=subprocess.PIPE,text=True)
    def receive():
        try:
            feed = (json.loads(line) for line in remote.stdout) if remote else frames(directory)
            for msg in feed:
                messages.put(msg)
        finally:
            messages.put(('end',0,''))
    runner = subprocess.Popen(launch) if launch else None
    threading.Thread(target=receive,daemon=True).start()
    stamp = None
    def update():
        nonlocal stamp
        if runner and os.getenv("PIPER_AGENT_GRASP_ID") and runner.poll() is not None:
            root.destroy()
            return
        while not messages.empty():
            kind,t,data = messages.get_nowait()
            if kind in ('end','done'):
                status.set('抓取已结束' if kind=='done' else '预览连接已结束')
                if runner and os.getenv('PIPER_AGENT_GRASP_ID'):
                    root.after(300,update)
                return
            label.photo = ImageTk.PhotoImage(Image.open(io.BytesIO(base64.b64decode(data))))
            label.configure(image=label.photo)
            stamp=t
        if stamp:
            status.set(time.strftime('%H:%M:%S',time.localtime(stamp))+
                       (' · 等待新帧' if time.time()-stamp>5 else ' · RGB'))
        root.after(300,update)
    try:
        update()
        root.mainloop()
    finally:
        if remote:
            remote.terminate()
            remote.wait()
    return runner.wait() if runner else 0


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stream',type=Path)
    parser.add_argument('--view',type=Path)
    parser.add_argument('--host')
    parser.add_argument('--start-grasp',action='store_true')
    parser.add_argument('--run-id',default='grasp-'+uuid.uuid4().hex[:12])
    parser.add_argument('--output-root',default='.local/state/piper-vision-grasp')
    args,extra=parser.parse_known_args()
    if args.stream:
        for row in frames(args.stream):print(json.dumps(row),flush=True)
        return 0
    if args.host and args.host.startswith('-'):parser.error('主机名无效')
    if args.view:
        return show(args.view,args.host)
    if not args.start_grasp:parser.error('请选择 --view 或 desktop-run')
    host=args.host or 'unitree-go'
    if host.startswith('-') or Path(args.run_id).name!=args.run_id or args.run_id in ('','.','..'):
        parser.error('主机名或运行编号无效')
    command=['bash','piper-vision-current/scripts/piper_vision_grasp.sh','run','--output-root',args.output_root,'--run-id',args.run_id]+extra
    launch=['ssh','-xT','-o','BatchMode=yes',host,
            'export PIPER_VISION_GRASP_PYTHON="$HOME/piper-handeye-20261112/.handeye-venv/bin/python"; exec '+shlex.join(command)]
    if os.getenv('PIPER_AGENT_GRASP_ID') and not (os.getenv('DISPLAY') or os.getenv('WAYLAND_DISPLAY')):
        return subprocess.call(launch)
    return show(Path(args.output_root)/args.run_id,host,launch)


if __name__=='__main__':
    sys.exit(main())
