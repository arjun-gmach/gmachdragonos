#!/bin/bash
# Pull the latest code from GitHub and restart the rig server.
#   ~/gmach-rig/pi/update.sh
set -e
cd "$(dirname "$0")/.."
echo "== pulling latest from GitHub"
git pull --ff-only
echo "== restarting rig server"
sudo systemctl restart gmach-rig
sleep 2
systemctl --no-pager --lines=5 status gmach-rig || true
echo
echo "Updated. Reload the dashboard (F5) to get the new page."
echo "If the Mega sketch changed (mega/test_rig), upload it from the Arduino IDE."
