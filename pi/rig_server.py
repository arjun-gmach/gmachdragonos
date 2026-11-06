#!/usr/bin/env python3
"""
GMACH RIG SERVER - Raspberry Pi
-------------------------------------------------------------------
Talks to the Mega over USB (through the USB isolator), serves the
dashboard to the HDMI screen and to every phone/laptop on the network,
forwards commands, and records runs as CSV files in ~/rig_logs.

  Dashboard:  http://<pi-name>.local:8080   (on the Pi itself: http://localhost:8080)

Only the operator who logs in can control the rig. Everyone else - including the
Pi's own HDMI screen (kiosk) - can only view.

Operator login: ~/.config/gmach/config.json - created by install.sh, never in git.
Change the login:  python3 rig_server.py --set-login
Runs as the 'gmach-rig' service (see install.sh). Manual run:
    python3 rig_server.py              (auto-detects the Mega)
    python3 rig_server.py --port /dev/ttyACM0
"""

import argparse
import getpass
import hashlib
import hmac
import secrets
import datetime
import json
import os
import queue
import signal
import re
import shutil
import socket
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import serial
import serial.tools.list_ports

# ================= SETTINGS =================
HTTP_PORT = 8080
BAUD = 115200
SESSION_HOURS = 12             # operator stays logged in this long
LOG_DIR = os.path.expanduser("~/rig_logs")
AUTOLOG = True                 # record automatically while the rig is active
AUTOLOG_STOP_AFTER_S = 10      # ... and stop this long after it goes idle
HEARTBEAT_S = 1.0              # "hb" to the Mega; it goes safe if these stop
LINK_TIMEOUT_S = 1.5           # no status line for this long = no rig data
HIST_MAX = 4000                # graph history kept in memory (thinned as runs get long)

HERE = os.path.dirname(os.path.abspath(__file__))

# Local settings that must NOT go to GitHub (the operator login) live in ~/.config/gmach/config.json:
#   {"users": {"arjun": {"salt": ..., "hash": ...}}, "http_port": 8080, "log_dir": "~/rig_logs", "autolog": true}
CONFIG_PATH = os.path.expanduser("~/.config/gmach/config.json")
USERS = {}
try:
    with open(CONFIG_PATH) as _f:
        _cfg = json.load(_f)
    USERS = _cfg.get("users", {})
    HTTP_PORT = int(_cfg.get("http_port", HTTP_PORT))
    LOG_DIR = os.path.expanduser(_cfg.get("log_dir", LOG_DIR))
    AUTOLOG = bool(_cfg.get("autolog", AUTOLOG))
except FileNotFoundError:
    pass
except (ValueError, OSError) as _e:
    print(f"config: could not read {CONFIG_PATH} ({_e}) - using defaults", flush=True)
FIELD_KEYS = ["SP", "RPM", "Open", "T", "Pt", "Ps", "q", "V", "M", "Pump", "R7", "R8",
              "G", "Gs", "Mode", "Cyc", "CycN", "F", "Seq", "SeqS", "Stut"]
CSV_HEADER = ("clock,t_s,setpoint_rpm,rpm,air_valve_deg,temp_c,pt_pa,ps_pa,q_pa,velocity_ms,mach,"
              "pump_pct,sol7,sol8,glow,glow_s,mode,tune_cycle,tune_cycles,fault,"
              "start_step,start_step_s,fuel_pulses,event")


# ================= SHARED STATE =================
class State:
    def __init__(self):
        self.lock = threading.RLock()
        self.ser = None
        self.ser_lock = threading.Lock()
        self.port_name = ""
        self.cur = {}
        self.last_line = 0.0
        self.link_up = False
        self.clients = set()            # one queue per open dashboard
        self.msgs = []                  # recent messages for late joiners
        self.hist = []                  # [t_ms, {fields}]
        self.hist_stride = 1
        self.hist_skip = 0
        self.t0 = time.time()
        # logging
        self.log_file = None
        self.log_name = ""
        self.log_start = 0.0
        self.log_rows = 0
        self.log_by_auto = False
        self.auto_log = AUTOLOG
        self.last_active = 0.0


S = State()


def now_ms():
    return int((time.time() - S.t0) * 1000)


# ================= LIVE STREAM =================
def broadcast(msg):
    with S.lock:
        dead = []
        for q in S.clients:
            try:
                q.put_nowait(msg)
            except queue.Full:
                dead.append(q)          # a stuck viewer: drop it
        for q in dead:
            S.clients.discard(q)


def push_msg(m):
    print(m, flush=True)
    with S.lock:
        S.msgs.append(m)
        del S.msgs[:-40]
    broadcast("M|" + m)
    if S.log_file and not m.startswith("serial:"):
        log_row(m)


def log_state():
    with S.lock:
        logs_bytes = sum(os.path.getsize(os.path.join(LOG_DIR, f)) for f in os.listdir(LOG_DIR) if f.endswith(".csv"))
        free = shutil.disk_usage(LOG_DIR).free
        return {
            "rec": S.log_file is not None,
            "auto": S.auto_log,
            "file": S.log_name if S.log_file else "",
            "size": S.log_file.tell() if S.log_file else 0,
            "secs": int(time.time() - S.log_start) if S.log_file else 0,
            "used": logs_bytes,
            "total": logs_bytes + free,
            "link": S.link_up,
            "clients": len(S.clients),
        }


# ================= LOGGING =================
def rig_active():
    c = S.cur
    return bool(c) and any((c.get(k) or 0) > 0 for k in ("Mode", "Pump", "R7", "R8", "G", "Seq"))


def log_start(by_auto):
    with S.lock:
        if S.log_file:
            return
        base = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        name, n = base + ".csv", 2
        while os.path.exists(os.path.join(LOG_DIR, name)):
            name, n = f"{base}-{n}.csv", n + 1
        S.log_file = open(os.path.join(LOG_DIR, name), "w", buffering=1)
        S.log_file.write(CSV_HEADER + "\n")
        S.log_name, S.log_start, S.log_rows, S.log_by_auto = name, time.time(), 0, by_auto
    push_msg(f"logging to {name}" + (" (auto)" if by_auto else ""))


def log_stop(why):
    with S.lock:
        if not S.log_file:
            return
        S.log_file.close()
        S.log_file = None
        name, rows = S.log_name, S.log_rows
    push_msg(f"log saved: {name} ({rows} rows) - {why}")


def log_row(event=""):
    with S.lock:
        if not S.log_file:
            return
        clock = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-5]
        t = f"{time.time() - S.log_start:.2f}"
        if event:
            vals = [""] * len(FIELD_KEYS)
            event = event.replace(",", ";").replace('"', "'")
        else:
            vals = ["" if S.cur.get(k) is None else f"{S.cur[k]:g}" for k in FIELD_KEYS]
        S.log_file.write(",".join([clock, t] + vals + [event]) + "\n")
        S.log_rows += 1


# ================= FROM THE MEGA =================
def parse_status(line):
    d = {}
    for part in line.split(","):
        k, _, v = part.partition(":")
        if not v:
            continue
        try:
            d[k] = None if v == "nan" else float(v)
        except ValueError:
            pass
    return d if "RPM" in d else None


def hist_add(t, d):
    with S.lock:
        S.hist_skip += 1
        if S.hist_skip < S.hist_stride:
            return
        S.hist_skip = 0
        if len(S.hist) >= HIST_MAX:            # keep the whole run: thin out old points
            S.hist = S.hist[::2]
            S.hist_stride *= 2
        S.hist.append((t, d))


def handle_line(line):
    if not line:
        return
    S.last_line = time.time()
    if line.startswith("SP:"):
        d = parse_status(line)
        if not d:
            return
        S.cur = d
        t = now_ms()
        broadcast(f"D|{t}|{line}")
        hist_add(t, d)
        if rig_active():
            S.last_active = time.time()
        if S.auto_log and not S.log_file and rig_active():
            log_start(True)
        if S.log_file and S.log_by_auto and not rig_active() and time.time() - S.last_active > AUTOLOG_STOP_AFTER_S:
            log_stop("rig idle")
        if S.log_file:
            log_row()
    else:
        m = line[2:].strip() if line.startswith(">>") else line
        if m:
            push_msg(m)


def find_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:                              # Arduino / CH340 / FTDI first
        desc = f"{p.description} {p.manufacturer}".lower()
        if p.vid in (0x2341, 0x2A03, 0x1A86, 0x0403) or "arduino" in desc:
            return p.device
    for p in ports:
        if "ACM" in p.device or "USB" in p.device:
            return p.device
    return None


def serial_loop(fixed_port):
    announced_missing = False
    while True:
        port = fixed_port or find_port()
        if not port:
            if not announced_missing:
                push_msg("serial: waiting for the Mega to be plugged in…")
                announced_missing = True
            time.sleep(1)
            continue
        announced_missing = False
        ser = None
        try:
            ser = serial.Serial()
            ser.port, ser.baudrate, ser.timeout = port, BAUD, 0.2
            ser.dtr = False                      # avoid restarting the Mega where the driver allows
            ser.open()
            with S.ser_lock:
                S.ser, S.port_name = ser, port
            push_msg(f"serial: connected to {port}")
            buf = b""
            while True:
                chunk = ser.read(512)
                if chunk:
                    buf += chunk
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        handle_line(raw.decode(errors="replace").strip())
                    if len(buf) > 2048:
                        buf = b""
        except (serial.SerialException, OSError) as e:
            push_msg(f"FAULT: USB link to the Mega lost ({e.__class__.__name__}) - reconnecting")
        finally:
            with S.ser_lock:
                S.ser = None
            try:
                if ser:
                    ser.close()
            except Exception:
                pass
        time.sleep(1)


def send_to_rig(text):
    with S.ser_lock:
        if not S.ser:
            return False
        try:
            S.ser.write((text + "\n").encode())
            return True
        except (serial.SerialException, OSError):
            return False


def background_loop():
    last_hb = 0.0
    while True:
        t = time.time()
        if t - last_hb >= HEARTBEAT_S:
            send_to_rig("hb")
            last_hb = t
        up = S.last_line > 0 and t - S.last_line < LINK_TIMEOUT_S
        if up != S.link_up:
            S.link_up = up
            push_msg("rig connected" if up else "FAULT: no data from rig")
        broadcast("L|" + json.dumps(log_state()))
        time.sleep(1.0)


# ================= LOGIN =================
PBKDF2_ITER = 200_000
SESSIONS = {}                 # token -> (user, expiry time)
FAILS = {}                    # ip -> [times of failed logins]
AUTH_LOCK = threading.Lock()


def hash_password(password, salt=None):
    salt = salt or secrets.token_bytes(16)
    h = hashlib.pbkdf2_hmac("sha256", password.encode(), salt, PBKDF2_ITER)
    return {"salt": salt.hex(), "hash": h.hex(), "iter": PBKDF2_ITER}


def check_password(user, password):
    rec = USERS.get(user)
    if not rec:
        hashlib.pbkdf2_hmac("sha256", password.encode(), b"x" * 16, PBKDF2_ITER)   # same timing either way
        return False
    h = hashlib.pbkdf2_hmac("sha256", password.encode(), bytes.fromhex(rec["salt"]), int(rec.get("iter", PBKDF2_ITER)))
    return hmac.compare_digest(h.hex(), rec["hash"])


def set_login():
    """python3 rig_server.py --set-login : create / replace the operator account"""
    cfg = {}
    if os.path.exists(CONFIG_PATH):
        with open(CONFIG_PATH) as f:
            cfg = json.load(f)
    user = input("Operator username: ").strip()
    if not user:
        raise SystemExit("no username given")
    while True:
        p1 = getpass.getpass("Password (8+ characters): ")
        if len(p1) < 8:
            print("too short"); continue
        if p1 != getpass.getpass("Repeat password: "):
            print("passwords don't match"); continue
        break
    cfg["users"] = {user: hash_password(p1)}          # one operator: replaces any previous login
    cfg.pop("pin", None)
    os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
    with open(CONFIG_PATH, "w") as f:
        json.dump(cfg, f, indent=2)
    os.chmod(CONFIG_PATH, 0o600)
    print(f"Saved. '{user}' is the only operator. Restart the server: sudo systemctl restart gmach-rig")


# ================= HTTP =================
NAME_RE = re.compile(r"^[A-Za-z0-9_.-]{1,40}\.csv$")


class Handler(BaseHTTPRequestHandler):
    server_version = "GMACH"
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass                                     # keep the console for rig messages

    def is_local(self):
        return self.client_address[0] in ("127.0.0.1", "::1", "::ffff:127.0.0.1")

    def session_user(self):
        cookie = self.headers.get("Cookie", "")
        m = re.search(r"(?:^|;\s*)gmach_session=([A-Za-z0-9_-]+)", cookie)
        if not m:
            return None
        with AUTH_LOCK:
            rec = SESSIONS.get(m.group(1))
            if not rec or rec[1] < time.time():
                SESSIONS.pop(m.group(1), None)
                return None
            return rec[0]

    def reply(self, code, body, ctype="text/plain; charset=utf-8", extra=None):
        data = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-cache")
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        u = urllib.parse.urlparse(self.path)
        q = {k: v[0] for k, v in urllib.parse.parse_qs(u.query).items()}
        route = {
            "/": self.page, "/live": self.live, "/history": self.history, "/logs": self.logs,
            "/dl": self.download, "/delete": self.delete, "/cmd": self.cmd, "/me": self.me,
            "/time": lambda q: self.reply(200, "ok"), "/log": self.logctl,
        }.get(u.path)
        if route:
            route(q)
        else:
            self.reply(302, "", extra={"Location": "/"})

    def do_POST(self):
        u = urllib.parse.urlparse(self.path)
        n = min(int(self.headers.get("Content-Length") or 0), 4096)
        body = self.rfile.read(n).decode(errors="replace") if n else ""
        try:
            data = json.loads(body) if body else {}
        except ValueError:
            data = {}
        if u.path == "/login":
            self.login(data)
        elif u.path == "/logout":
            self.logout()
        else:
            self.reply(404, "not found")

    # ---- login ----
    def me(self, q):
        self.reply(200, json.dumps({"user": self.session_user(), "configured": bool(USERS)}), "application/json")

    def login(self, data):
        ip = self.client_address[0]
        now = time.time()
        with AUTH_LOCK:
            recent = [t for t in FAILS.get(ip, []) if now - t < 300]
            FAILS[ip] = recent
        if len(recent) >= 5:
            return self.reply(429, "too many attempts - wait 5 minutes")
        user, pw = str(data.get("user", "")).strip(), str(data.get("pass", ""))
        if not USERS:
            return self.reply(503, "no operator login set up - run: python3 rig_server.py --set-login")
        if not check_password(user, pw):
            with AUTH_LOCK:
                FAILS.setdefault(ip, []).append(now)
            time.sleep(1)
            return self.reply(403, "wrong username or password")
        token = secrets.token_urlsafe(32)
        with AUTH_LOCK:
            SESSIONS[token] = (user, now + SESSION_HOURS * 3600)
            FAILS.pop(ip, None)
        push_msg(f"operator {user} logged in")
        self.reply(200, json.dumps({"user": user}), "application/json", {
            "Set-Cookie": f"gmach_session={token}; HttpOnly; SameSite=Strict; Path=/; Max-Age={SESSION_HOURS * 3600}"})

    def logout(self):
        cookie = self.headers.get("Cookie", "")
        m = re.search(r"(?:^|;\s*)gmach_session=([A-Za-z0-9_-]+)", cookie)
        user = self.session_user()
        if m:
            with AUTH_LOCK:
                SESSIONS.pop(m.group(1), None)
        if user:
            push_msg(f"operator {user} logged out")
        self.reply(200, "ok", extra={"Set-Cookie": "gmach_session=; HttpOnly; SameSite=Strict; Path=/; Max-Age=0"})

    # ---- pages ----
    def page(self, q):
        with open(os.path.join(HERE, "dashboard.html"), encoding="utf-8") as f:
            html = f.read()
        kiosk = self.is_local() and "operator" not in q        # http://localhost:8080/?operator = normal page
        inject = "<script>window.GMACH_LIVE_URL='/live';window.GMACH_KIOSK=%s;</script>" % (
            "true" if kiosk else "false")
        self.reply(200, html.replace("</head>", inject + "</head>", 1), "text/html; charset=utf-8")

    def live(self, q):
        cq = queue.Queue(maxsize=1000)
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        with S.lock:
            backlog = ["H|%d" % now_ms()] + ["M|" + m for m in S.msgs] + ["L|" + json.dumps(log_state())]
            S.clients.add(cq)
        try:
            self.wfile.write(b"retry: 1500\n\n")
            for m in backlog:
                self.wfile.write(("data: " + m + "\n\n").encode())
            self.wfile.flush()
            while True:
                try:
                    m = cq.get(timeout=5)
                    self.wfile.write(("data: " + m + "\n\n").encode())
                except queue.Empty:
                    self.wfile.write(b": keepalive\n\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            with S.lock:
                S.clients.discard(cq)
        self.close_connection = True

    def history(self, q):
        with S.lock:
            rows = list(S.hist)
        out = ["t," + ",".join(FIELD_KEYS)]
        for t, d in rows:
            out.append(str(t) + "," + ",".join("" if d.get(k) is None else f"{d[k]:g}" for k in FIELD_KEYS))
        self.reply(200, "\n".join(out) + "\n")

    def logs(self, q):
        files = []
        for f in sorted(os.listdir(LOG_DIR)):
            if f.endswith(".csv"):
                files.append({"n": f, "s": os.path.getsize(os.path.join(LOG_DIR, f))})
        self.reply(200, json.dumps({"files": files, "state": log_state()}), "application/json")

    def download(self, q):
        n = q.get("f", "")
        path = os.path.join(LOG_DIR, n)
        if not NAME_RE.match(n) or not os.path.exists(path):
            return self.reply(404, "not found")
        if S.log_file and S.log_name == n:
            S.log_file.flush()
        with open(path, "rb") as f:
            data = f.read()
        self.reply(200, data, "text/csv", {"Content-Disposition": f'attachment; filename="gmach-{n}"'})

    def delete(self, q):
        n = q.get("f", "")
        if not self.session_user():
            return self.reply(403, "view only - log in as the operator to delete logs")
        if not NAME_RE.match(n):
            return self.reply(400, "bad name")
        if S.log_file and S.log_name == n:
            return self.reply(409, "file is being recorded - stop logging first")
        try:
            os.remove(os.path.join(LOG_DIR, n))
        except FileNotFoundError:
            pass
        push_msg(f"deleted log {n}")
        self.reply(200, "ok")

    # ---- control (operator only) ----
    def cmd(self, q):
        user = self.session_user()
        if not user:
            return self.reply(403, "view only - log in as the operator to control the rig")
        c = q.get("c", "").strip()
        if not c or len(c) > 22 or "\n" in c:
            return self.reply(400, "bad command")
        if not send_to_rig(c):
            return self.reply(503, "Mega not connected - check the USB cable / isolator")
        push_msg("> " + c)
        self.reply(200, "ok")

    def logctl(self, q):
        if not self.session_user():
            return self.reply(403, "view only - log in as the operator to change recording")
        a = q.get("a", "")
        if a == "start":
            log_start(False)
        elif a == "stop":
            log_stop("stopped")
        elif a in ("autoon", "autooff"):
            S.auto_log = a == "autoon"
            push_msg("auto-logging " + ("ON" if S.auto_log else "OFF"))
        broadcast("L|" + json.dumps(log_state()))
        self.reply(200, "ok")


def lan_addresses():
    ips = set()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    return sorted(ips)


def main():
    global ARGS
    ap = argparse.ArgumentParser(description="GMACH rig server")
    ap.add_argument("--port", help="serial port of the Mega (default: auto-detect)")
    ap.add_argument("--http", type=int, default=HTTP_PORT, help="dashboard port (default 8080)")
    ap.add_argument("--set-login", action="store_true", help="create / change the operator username and password")
    ARGS = ap.parse_args()
    if ARGS.set_login:
        return set_login()
    if not USERS:
        print("NOTE: no operator login yet - everyone is view-only. Set one: python3 rig_server.py --set-login", flush=True)

    os.makedirs(LOG_DIR, exist_ok=True)
    threading.Thread(target=serial_loop, args=(ARGS.port,), daemon=True).start()
    threading.Thread(target=background_loop, daemon=True).start()

    # systemd stops the service with SIGTERM: treat it like Ctrl+C so the clean-up below runs
    def on_term(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, on_term)

    srv = ThreadingHTTPServer(("0.0.0.0", ARGS.http), Handler)
    srv.daemon_threads = True
    print(f"GMACH rig server on port {ARGS.http}")
    print(f"  this Pi:   http://localhost:{ARGS.http}")
    print(f"  network:   http://{socket.gethostname()}.local:{ARGS.http}")
    for ip in lan_addresses():
        print(f"             http://{ip}:{ARGS.http}")
    print(f"  logs in    {LOG_DIR}", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        send_to_rig("abort")                   # server going away: close starter fuel, glow off
        log_stop("server stopped")


ARGS = None
if __name__ == "__main__":
    main()
