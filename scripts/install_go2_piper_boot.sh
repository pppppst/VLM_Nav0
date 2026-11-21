#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if (( EUID != 0 )); then
  echo '请在 Go2 上位机终端使用 sudo bash 执行本安装脚本。' >&2
  exit 1
fi
log=/var/log/piper-boot-install.log
echo "安装日志：$log"
exec 3>&1
exec >>"$log" 2>&1
created=false
rollback() {
  if "$created"; then
    systemctl disable piper-boot.service || true
    rm -f /etc/systemd/system/piper-boot.service
    rm -f /opt/piper-boot/go2_piper_boot.py /opt/piper-boot/piper.json /opt/piper-boot/piper_boot_status.sh
    rmdir /opt/piper-boot || true
    systemctl daemon-reload || true
  fi
  echo '安装失败，已撤销本次新建部署；请查看 /var/log/piper-boot-install.log。' >&3
}
trap rollback ERR
exec 9>/run/lock/piper-boot-install.lock
flock -n 9
test "$(uname -m)" = aarch64
test -x /home/unitree/piper-venv/bin/python
id unitree
current="$(systemctl show piper-boot.service -p ActiveState --value 2>/dev/null || true)"
if [[ "$current" == active || "$current" == activating || "$current" == deactivating ]]; then
  echo '已有 Piper 服务运行，安装器不停止或覆盖运行服务。' >&3
  exit 1
fi
if [[ -e /etc/systemd/system/piper-boot.service || -L /etc/systemd/system/piper-boot.service || -e /opt/piper-boot || -L /opt/piper-boot ]]; then
  echo '已有 Piper 部署，安装器拒绝覆盖；请先核对现有安装。' >&3
  exit 1
fi
runuser -u unitree -- /home/unitree/piper-venv/bin/python -I -c \
  'from importlib.metadata import version; from piper_sdk import C_PiperInterface_V2; assert version("piper_sdk") == "0.6.2"'
runuser -u unitree -- /home/unitree/piper-venv/bin/python -I -c \
  'import ast,json,pathlib,sys; ast.parse(pathlib.Path(sys.argv[1]).read_text()); json.loads(pathlib.Path(sys.argv[2]).read_text())' \
  "$root/scripts/go2_piper_boot.py" "$root/config/piper_boot/piper.json"
systemd-analyze verify "$root/config/piper_boot/piper-boot.service"
created=true
install -d -m 0755 /opt/piper-boot
install -m 0644 "$root/scripts/go2_piper_boot.py" /opt/piper-boot/
install -m 0644 "$root/config/piper_boot/piper.json" /opt/piper-boot/
install -m 0755 "$root/scripts/piper_boot_status.sh" /opt/piper-boot/
install -m 0644 "$root/config/piper_boot/piper-boot.service" /etc/systemd/system/
systemd-analyze verify /etc/systemd/system/piper-boot.service
systemctl daemon-reload
systemctl enable piper-boot.service
systemctl is-enabled piper-boot.service
echo '安装完成；已设为下次开机启动，没有启动当前服务或发送使能。'
echo '已配置下次开机启动；本次未启动服务。' >&3
