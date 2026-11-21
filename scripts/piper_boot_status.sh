#!/usr/bin/env bash
# Run in a dedicated diagnostic terminal on the Go2 upper computer.
set -euo pipefail
echo '状态和原因日志：journalctl -u piper-boot.service'
systemctl show piper-boot.service -p ActiveState -p SubState -p Result
exec journalctl -u piper-boot.service -b -n 30 -f --no-pager
