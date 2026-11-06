#!/bin/bash
# GMACH rig - Raspberry Pi installer. Run once after cloning (safe to re-run):
#   bash ~/gmach-rig/pi/install.sh
set -e
PI_DIR="$(cd "$(dirname "$0")" && pwd)"
CONF_DIR="$HOME/.config/gmach"
echo "== GMACH rig: installing from $PI_DIR"

echo "== pyserial"
sudo apt-get update -qq
sudo apt-get install -y python3-serial

echo "== serial port permission"
sudo usermod -aG dialout "$USER"

echo "== local settings (kept out of git)"
mkdir -p "$CONF_DIR" "$HOME/rig_logs"
if [ ! -f "$CONF_DIR/config.json" ]; then
  read -r -p "Choose a control PIN for phones/laptops [1234]: " PIN
  PIN=${PIN:-1234}
  printf '{\n  "pin": "%s",\n  "http_port": 8080,\n  "log_dir": "~/rig_logs",\n  "autolog": true\n}\n' "$PIN" > "$CONF_DIR/config.json"
  chmod 600 "$CONF_DIR/config.json"
  echo "   saved $CONF_DIR/config.json"
else
  echo "   keeping existing $CONF_DIR/config.json"
fi

echo "== server service (starts at boot, restarts if it crashes)"
sudo tee /etc/systemd/system/gmach-rig.service >/dev/null <<UNIT
[Unit]
Description=GMACH rig dashboard server
After=network-online.target
Wants=network-online.target

[Service]
User=$USER
WorkingDirectory=$PI_DIR
ExecStart=/usr/bin/python3 $PI_DIR/rig_server.py
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
UNIT
sudo systemctl daemon-reload
sudo systemctl enable gmach-rig
sudo systemctl restart gmach-rig

echo "== full-screen dashboard on HDMI at login"
chmod +x "$PI_DIR/gmach-kiosk.sh" "$PI_DIR/update.sh"
mkdir -p "$HOME/.config/autostart"
cat > "$HOME/.config/autostart/gmach-kiosk.desktop" <<DESK
[Desktop Entry]
Type=Application
Name=GMACH dashboard
Exec=$PI_DIR/gmach-kiosk.sh
X-GNOME-Autostart-enabled=true
DESK

echo "== auto-login to the desktop, no screen blanking"
sudo raspi-config nonint do_boot_behaviour B4 || true
sudo raspi-config nonint do_blanking 1 || true

echo
echo "Done. Reboot once to start everything:  sudo reboot"
echo "Dashboard on the network:  http://$(hostname).local:8080"
