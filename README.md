# GMACH test rig

Control and data acquisition for the GMACH micro jet engine test rig.

An **Arduino Mega** runs the rig in real time: RPM control of the air-start valve (PID with autotune), fuel pump, starter-fuel and air solenoids, glow plug, thermocouple, pitot pressures, and the automatic start sequence. A **Raspberry Pi** connected over USB serves the live dashboard full screen on its HDMI display and to every phone/laptop on the network, and records each run as CSV.

```
Mega (runs the rig) ──USB──[USB isolator]── Raspberry Pi ──Wi-Fi/Ethernet──► phones & laptops
                                             dashboard server + CSV logs    http://gmach-rig.local:8080
```

## Repository layout

| Folder | Contents |
|---|---|
| `mega/test_rig/` | Mega sketch. Open `test_rig.ino` in the Arduino IDE. |
| `pi/` | Pi server (`rig_server.py`), installer, updater, HDMI kiosk launcher, served dashboard |
| `dashboard/` | Dashboard source (`index.src.html`) and `build.py`, which generates every copy |
| `usb/` | Stand-alone dashboard: open in Chrome/Edge with the Mega on a USB cable, no Pi needed |
| `esp32/` | Optional ESP32 Wi-Fi bridge (Mega TX1/RX1 → ESP32 hotspot). Not used with the Pi. |

## Mega wiring

| Pin | Connected to |
|---|---|
| D2 | Hall sensor (RPM), interrupt |
| D3 | Air-valve servo |
| D9 | Fuel-pump ESC signal (Hobbywing Skywalker, white wire) |
| A0 | Relay IN7 — solenoid 7 (starter fuel) |
| A1 | Relay IN8 — solenoid 8 |
| A3 | Relay — glow plug (relay COM from a ~1.5–1.8 V supply, **not** 12 V) |
| D50 / D51 / D52 / D53 | MAX31856 thermocouple: SDO / SDI / SCK / CS |
| D20 / D21 | ADS1115 SDA / SCL (ADS A0 = total pressure, A1 = static pressure) |

Relay board is active-low. Flyback diodes (1N4007) across each solenoid coil, striped end to +.

**Libraries (Mega):** Adafruit MAX31856, Adafruit ADS1X15 (Library Manager, "Install all" for Adafruit BusIO). Servo, EEPROM, SPI, Wire are built in.

## Raspberry Pi setup

1. Raspberry Pi OS (64-bit, desktop), hostname `gmach-rig`, Wi-Fi and SSH enabled in Raspberry Pi Imager.
2. Give the Pi read access to this private repository with a deploy key:
   ```bash
   ssh-keygen -t ed25519 -N "" -f ~/.ssh/gmach_deploy
   cat ~/.ssh/gmach_deploy.pub
   ```
   On GitHub: repository → **Settings → Deploy keys → Add deploy key**, paste the key, leave *write access* off. Then:
   ```bash
   printf 'Host github.com\n  IdentityFile ~/.ssh/gmach_deploy\n' >> ~/.ssh/config
   git clone git@github.com:arjun-gmach/gmachdragonos.git ~/gmach-rig
   ```
3. Install and reboot:
   ```bash
   bash ~/gmach-rig/pi/install.sh
   sudo reboot
   ```
   The installer asks for the control PIN and stores it in `~/.config/gmach/config.json` (never committed).

After the reboot the HDMI screen shows the dashboard full screen with full control. Other devices open `http://gmach-rig.local:8080` (or `http://<ip shown by hostname -I>:8080`) and need the PIN to control. **STOP** and **Abort** always work without the PIN.

**Update the Pi** after pushing changes:
```bash
~/gmach-rig/pi/update.sh
```

**Service commands:** `sudo systemctl status gmach-rig` · `journalctl -u gmach-rig -f` · `sudo systemctl restart gmach-rig`

## Serial commands (Mega, 115200 baud)

| Command | Action |
|---|---|
| `3000` / `0` | Hold 3000 RPM / close air valve |
| `tune 3000` | Relay autotune at 3000 RPM, gains stored in EEPROM |
| `table` · `clear` · `sched on` · `sched off` · `kp x` · `ki x` · `kd x` | Gain schedule and manual gains |
| `pump 40` / `pump 0` | Fuel pump throttle % |
| `7on` `7off` `8on` `8off` | Solenoids |
| `glowon` / `glowoff` | Glow plug (auto-off after 30 s) |
| `start` · `go` · `abort` | Start sequence (below) |
| `pzero` · `autozero on/off` · `qdb 8` · `t 30` · `t tc` | Pressure zero, velocity deadband, total temperature source |
| `stop` | Everything off |
| `hb` | Heartbeat (sent automatically by the dashboard / Pi) |

### Start sequence

1. Spin up to 4000 RPM on air, held within ±3 % for 2 s (60 s timeout)
2. Wait for the operator to confirm the glow plug (`go`, or the prompt on every dashboard)
3. Glow plug on, 5 s preheat
4. Starter fuel (solenoid 7) pulses 6 × (0.2 s open / 0.3 s closed)
5. Starter fuel open, glow on; fuel pump throttle stays under operator control

The sequence aborts on `abort`, `stop`, any fault, or if the air valve stops holding RPM. Timings are constants at the top of `test_rig.ino`.

### Safety behaviour

- Overspeed (30 000 RPM) closes the air valve and aborts the start sequence.
- Glow plug switches off after 30 s, on STOP, on any fault, and on link loss.
- **Link loss:** once heartbeats have been received, 3 s without one closes the starter fuel and switches the glow plug off (`LINK_LOSS_FULL_STOP = true` for everything off).
- Opening the Mega's USB port restarts it into the safe STOPPED state. A 10 µF capacitor between RESET and GND prevents that restart (remove it to upload).

## Dashboard

Three pages: **Overview** (rig state, RPM, key readings, controls, start sequence), **Graphs** (all sensors for the whole run), **Logging** (recorder on the Pi with auto-start, saved logs to download, record-on-this-device, event log).

To change the dashboard, edit `dashboard/index.src.html`, run `python3 dashboard/build.py`, and commit the regenerated files.
