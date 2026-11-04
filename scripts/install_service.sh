#!/bin/bash
# Instaluje usluge systemd 'rov' (autostart calego stosu po wlaczeniu Jetsona). Uruchamiac na Jetsonie, pyta o sudo.
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"
sudo cp "$HERE/scripts/rov.service" /etc/systemd/system/rov.service
sudo systemctl daemon-reload
sudo systemctl enable rov.service
sudo systemctl restart rov.service
sleep 3
systemctl --no-pager status rov.service | head -5
echo "logi: journalctl -u rov -f"
