#!/bin/bash
# Opens the GMACH dashboard full screen on the Pi's HDMI display.
# Started automatically when the desktop starts (see install.sh).

exec 9>/tmp/gmach-kiosk.lock
flock -n 9 || exit 0                     # only one copy

# wait for the rig server (up to 60 s after boot)
for i in $(seq 1 60); do
  python3 -c "import urllib.request; urllib.request.urlopen('http://localhost:8080/', timeout=1)" 2>/dev/null && break
  sleep 1
done

BROWSER=$(command -v chromium-browser || command -v chromium)
exec "$BROWSER" \
  --kiosk --noerrdialogs --disable-infobars --no-first-run \
  --disable-session-crashed-bubble --disable-features=Translate \
  --password-store=basic --check-for-update-interval=31536000 \
  --autoplay-policy=no-user-gesture-required \
  http://localhost:8080/
