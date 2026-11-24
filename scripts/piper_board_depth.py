#!/usr/bin/env python3
"""Start the Piper ID25 RGB-D calibration from the operator's desktop."""
import argparse
import base64
from datetime import datetime
import hashlib
import io
import json
from pathlib import Path
import queue
import re
import shlex
import subprocess
import sys
import tarfile
import threading
import uuid

ROOT = Path(__file__).resolve().parents[1]
PREFIX = 'PIPER_BOARD '


def deploy(host, log):
    files = ['scripts/go2_piper_boot.py','config/piper_vision_grasp/semantic-camera-avoid.json']
    cfg = json.loads((ROOT/files[1]).read_text())
    files += ['config/piper_vision_grasp/'+cfg[key] for key in ('kinematics','joint_zero_calibration')]
    files += ['tools/piper_vision_grasp/'+name+'.py' for name in
              ('__init__','board_depth','geometry','vision','live_rgb','piper_io','main')]
    digest = hashlib.sha256()
    data = io.BytesIO()
    with tarfile.open(fileobj=data,mode='w:gz') as archive:
        for rel in files:
            digest.update(rel.encode()+b'\0'+(ROOT/rel).read_bytes())
            archive.add(ROOT/rel,arcname=rel)
    release = '.local/share/piper-board-depth/'+digest.hexdigest()[:20]
    # Publish a complete version before use, allowing two independent launches.
    install = ('set -eu; mkdir -p .local/share/piper-board-depth; t=$(mktemp -d .local/share/piper-board-depth/upload-XXXXXXXX); '
               'trap \'rm -rf -- "$t"\' EXIT; tar -xz -C "$t"; '
               'if [ ! -d {0} ]; then mv -T "$t" {0} || test -d {0}; fi').format(release)
    subprocess.run(['ssh','-T','-o','BatchMode=yes','-o','ConnectTimeout=8',host,install],
                   input=data.getvalue(),stdout=log,stderr=log,check=True)
    return release


# Commands are valid only at the displayed prompt and belong to one camera attempt.
PROMPTS = {
    'preview': ('摆好姿态、结束示教并保持静止。回车拍摄，Q 暂停：', {'': 'shoot'}),
    'review': ('请核对上面的三张审核图。OK 收录，NO 弃用，Q 暂停：', {'OK': 'accept', 'NO': 'reject'}),
    'retry': ('修正后回车返回预览，Q 暂停：', {'': 'retry'}),
}


class TerminalReview:
    def __init__(self):
        self.phase = ('', '')
        self.submitted = False

    def show(self, row, review_paths):
        phase = (row['state'], row.get('token', ''))
        if phase == self.phase:
            return  # A new thumbnail does not create another shoot prompt.
        self.phase = phase
        self.submitted = False
        print(row['message'], flush=True)
        if phase[0]=='review':
            print('中间帧拍摄时间（机载 Unix 秒）：{:.6f}'.format(row['captured_at']),flush=True)
        for path in review_paths:
            print('审核图：'+str(path), flush=True)
        if phase[0] in PROMPTS:
            print(PROMPTS[phase[0]][0], flush=True)

    def respond(self, line):
        value = line.strip().upper()
        if not line or value=='Q':
            return 'quit'
        state, token = self.phase
        if state not in PROMPTS or self.submitted:
            return None
        prompt, choices = PROMPTS[state]
        if value not in choices:
            print('输入无效。'+prompt, flush=True)
            return None
        self.submitted = True
        return choices[value]+' '+token


def main():
    parser = argparse.ArgumentParser(description='终端回车拍摄、OK/NO 审核，独立窗口显示实时缩略图。默认新建批次。')
    parser.add_argument('--host',default='unitree-go')
    parser.add_argument('--resume',metavar='SESSION',help='只续采相同安装、相同批次')
    args = parser.parse_args()
    name = args.resume or datetime.now().strftime('board25-%Y%m%d-%H%M%S-')+uuid.uuid4().hex[:6]
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_-]{0,95}',name) or args.host.startswith('-'):
        parser.error('批次名或 SSH 主机名无效')
    from PIL import Image, ImageTk
    import tkinter as tk
    local = Path.home()/'Pictures/piper-handeye-depth'/name
    local.mkdir(parents=True,exist_ok=True)
    dataset = 'piper-board-depth-data/'+name
    logpath = local/'diagnostic.log'
    print('诊断日志：'+str(logpath),flush=True)
    print('暂停后续采：python3 {} --host {} --resume {}'.format(shlex.quote(str(Path(__file__).resolve())),
          shlex.quote(args.host),name),flush=True)
    with logpath.open('a',buffering=1) as log:
        win = tk.Tk()
        win.title('Piper 标定板 · 实时预览与审核图 · '+name)
        try:
            release = deploy(args.host,log)
        except subprocess.CalledProcessError:
            print('SSH 或传输失败，请检查 '+str(logpath),file=sys.stderr)
            return 1
        dst = '"$HOME"/'+shlex.quote(dataset)
        guard = 'test -f '+dst+'/settings.json' if args.resume else 'test ! -e '+dst
        cmd = ('set -e; cd "$HOME"; '+guard+'; export OPENBLAS_NUM_THREADS=1; '
               'exec "$HOME/piper-handeye-20261112/.handeye-venv/bin/python" -I -u '+
               release+'/tools/piper_vision_grasp/board_depth.py collect --dataset '+dst)
        proc = subprocess.Popen(['ssh','-T','-o','BatchMode=yes','-o','ServerAliveInterval=10',
                                 '-o','ServerAliveCountMax=2',args.host,cmd],stdin=subprocess.PIPE,
                                stdout=subprocess.PIPE,stderr=log,text=True)
        messages = queue.Queue()
        def receive():
            try:
                for line in proc.stdout:
                    if line.startswith(PREFIX):
                        row = json.loads(line[len(PREFIX):])
                        if row['state']!='preview':print(row['state'],row['message'],file=log)
                        messages.put(row)
                    else:
                        log.write(line)
            finally:
                code=proc.wait()
                messages.put(dict(state='exit',code=code,message='连接已结束，退出码 {}。诊断见 {}'.format(code,logpath)))
        thread = threading.Thread(target=receive,daemon=True)
        thread.start()
        status = tk.StringVar(value='正在打开机载相机…')
        tk.Label(win,textvariable=status,wraplength=900,font=('sans',13)).pack()
        display = tk.Label(win)
        display.pack()
        terminal = TerminalReview()
        exit_code = 0

        def pause():
            if not proc.stdin.closed:
                proc.stdin.close()
            print('已请求暂停，已收录样本保存在本批次。', flush=True)
            win.quit()

        def terminal_input(stream, _mask):
            value = terminal.respond(stream.readline())
            if value=='quit':
                pause()
            elif value is not None:
                try:
                    proc.stdin.write(value+'\n')
                    proc.stdin.flush()
                except (BrokenPipeError, OSError):
                    print('采样连接已断开，请查看诊断日志。', file=sys.stderr)

        # Tk services terminal input and incoming frames in the same event loop on Linux.
        win.createfilehandler(sys.stdin, tk.READABLE, terminal_input)
        win.protocol('WM_DELETE_WINDOW',win.iconify)
        def update():
            nonlocal exit_code
            while not messages.empty():
                row=messages.get()
                state=row['state']
                if state=='exit':
                    exit_code=row['code']
                    if exit_code:
                        print(row['message'],file=sys.stderr,flush=True)
                    win.quit()
                    return
                status.set(row['message'])
                review_paths=[]
                images=[]
                for n,encoded in enumerate(row.get('images',[])):
                    data=base64.b64decode(encoded)
                    if state in ('border','review'):
                        path=local/('{}-{}-{}-{}.jpg'.format(row['pose'],row['token'],n+1,state))
                        path.write_bytes(data)
                        review_paths.append(path)
                    img=Image.open(io.BytesIO(data)).convert('RGB')
                    if state in ('border','review'):
                        img.thumbnail((max(100,(win.winfo_screenwidth()-48)//len(row['images'])),480))
                    else:
                        img.thumbnail((320,240))
                    images.append(img)
                if images:
                    w,h=images[0].size
                    strip=Image.new('RGB',(w*len(images),h))
                    for n,img in enumerate(images):strip.paste(img,(n*w,0))
                    display.photo=ImageTk.PhotoImage(strip)
                    display.configure(image=display.photo)
                if state=='done':
                    path=local/'result.json'
                    path.write_text(json.dumps(row['result'],indent=2)+'\n')
                    print('结果文件：'+str(path),flush=True)
                terminal.show(row,review_paths)
            win.after(100,update)
        win.after(100,update)
        try:
            win.mainloop()
        except KeyboardInterrupt:
            print('正在暂停采样。',flush=True)
        finally:
            win.deletefilehandler(sys.stdin)
            win.destroy()
            if not proc.stdin.closed:proc.stdin.close()
            try:proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                proc.terminate();proc.wait()
            thread.join(timeout=2)
    return exit_code


if __name__=='__main__':
    sys.exit(main())
