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

echo "== operator login (kept out of git)"
mkdir -p "$CONF_DIR" "$HOME/rig_logs"
if ! grep -q '"users"' "$CONF_DIR/config.json" 2>/dev/null; then
  echo "   Only this login can control the rig - everyone else (and the HDMI kiosk) can only view."
  python3 "$PI_DIR/rig_server.py" --set-login
else
  echo "   keeping the existing operator login (change it: python3 $PI_DIR/rig_server.py --set-login)"
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

echo "== GMACH boot splash and desktop background"
if [ -d /usr/share/plymouth/themes/pix ]; then
  [ -f /usr/share/plymouth/themes/pix/splash.png.orig ] || sudo cp /usr/share/plymouth/themes/pix/splash.png /usr/share/plymouth/themes/pix/splash.png.orig 2>/dev/null || true
  sudo cp "$PI_DIR/splash.png" /usr/share/plymouth/themes/pix/splash.png
  if [ -n "$(ls /boot/firmware/initramfs* 2>/dev/null)" ] && command -v update-initramfs >/dev/null; then
    sudo update-initramfs -u || true          # boot splash lives in the initramfs on newer images
  fi
fi
# desktop background (works over SSH too: edit the desktop's config files directly)
mkdir -p "$HOME/.config/pcmanfm/LXDE-pi"
for n in 0 1; do
  f="$HOME/.config/pcmanfm/LXDE-pi/desktop-items-$n.conf"
  if [ -f "$f" ] && grep -q '^wallpaper=' "$f"; then
    sed -i "s#^wallpaper=.*#wallpaper=$PI_DIR/splash.png#; s#^wallpaper_mode=.*#wallpaper_mode=crop#" "$f"
  else
    printf '[*]\nwallpaper_mode=crop\nwallpaper=%s\ndesktop_bg=#000000\nshow_trash=0\nshow_mounts=0\n' "$PI_DIR/splash.png" >> "$f"
  fi
done

echo "== auto-login to the desktop, no screen blanking"
sudo raspi-config nonint do_boot_behaviour B4 || true
sudo raspi-config nonint do_blanking 1 || true

echo
echo "Done. Reboot once to start everything:  sudo reboot"
echo "Dashboard on the network:  http://$(hostname).local:8080"
