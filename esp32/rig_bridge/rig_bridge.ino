/*
  GMACH RIG - ESP32 Wi-Fi bridge
  ---------------------------------------------------------------
  Receives the Mega's status lines on Serial2, serves the dashboard over Wi-Fi,
  pushes live data to every phone/laptop, forwards commands to the Mega,
  and logs runs to the ESP32's own flash (LittleFS).

  WIRING
    Mega TX1 (D18) --[1k]--+-- ESP32 GPIO16 (RX2)      divider: ESP32 is 3.3 V only
                           |
                         [2k]
                           |
                          GND
    Mega RX1 (D19) ------------ ESP32 GPIO17 (TX2)
    Mega GND   ---------------- ESP32 GND
    5 V (BEC)  ---------------- ESP32 VIN   (or power it from USB)

  BOARD:   "ESP32 Dev Module" (Arduino IDE: Boards Manager -> esp32 by Espressif)
  LIBRARIES: none to install - everything used comes with the ESP32 board package
  PARTITION SCHEME: "Default 4MB with spiffs" (gives ~1.4 MB for logs)

  Live data goes to browsers as Server-Sent Events on port 81;
  commands come in as plain HTTP requests on port 80.

  USE
    Phone/laptop -> Wi-Fi "GMACH-RIG" (password below) -> open http://192.168.4.1
*/

#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <ESPmDNS.h>
#include <sys/time.h>
#include <time.h>
#include "index_html.h"

// ================= SETTINGS =================
const char *AP_SSID     = "GMACH-RIG";
const char *AP_PASS     = "gmach1234";   // at least 8 characters
const char *CONTROL_PIN = "1234";        // needed to control the rig (STOP never needs it)

// set true to join an existing Wi-Fi network instead of making a hotspot
const bool  JOIN_EXISTING_WIFI = false;
const char *STA_SSID = "your-wifi";
const char *STA_PASS = "your-password";

const char *MDNS_NAME = "gmach-rig";      // -> http://gmach-rig.local on most laptops

const int  RIG_RX = 16, RIG_TX = 17;
const long RIG_BAUD = 250000;             // must match BRIDGE_BAUD in test_rig.ino
const unsigned long LINK_TIMEOUT_MS = 1500;
const unsigned long AUTOLOG_STOP_AFTER_MS = 10000;   // rig idle this long -> auto-log stops
const size_t MIN_FREE_BYTES = 40000;      // stop logging when flash is this full

// ================= FIELDS =================
// order of values stored in history and written to CSV
const char *FIELD_KEYS[] = {"SP", "RPM", "Open", "T", "Pt", "Ps", "q", "V", "M",
                            "Pump", "R7", "R8", "Mode", "Cyc", "CycN", "F", "G", "Gs", "Seq", "SeqS", "Stut"};
const byte NF = sizeof(FIELD_KEYS) / sizeof(FIELD_KEYS[0]);
const char *CSV_HEADER = "clock,t_s,setpoint_rpm,rpm,air_valve_deg,temp_c,pt_pa,ps_pa,q_pa,"
                         "velocity_ms,mach,pump_pct,sol7,sol8,mode,tune_cycle,tune_cycles,fault,glow,glow_s,start_step,start_step_s,fuel_pulses,event";

// ================= OBJECTS =================
HardwareSerial &RIG = Serial2;
WebServer http(80);
WiFiServer liveServer(81);                // Server-Sent Events stream

const int MAX_LIVE = 8;
WiFiClient live[MAX_LIVE];

// ================= HISTORY (RAM) =================
// keeps the whole run: when full, every other point is dropped and the
// interval doubles, so long runs stay visible at lower resolution
struct Sample { uint32_t t; float v[NF]; };
const int HIST_N = 1000;
Sample *hist = nullptr;
int histCount = 0;
int histStride = 1, histSkip = 0;

// ================= STATE =================
float cur[NF];
bool  haveData = false;
unsigned long lastRigMs = 0;
bool  linkUp = false;

char  rigLine[256];
size_t rigLen = 0;


// recent messages for phones that connect later
const int MSG_N = 40;
String msgs[MSG_N];
int msgHead = 0, msgCount = 0;

// logging
File  logFile;
bool  logging = false;
bool  autoLog = true;
bool  logStartedByAuto = false;
String logName = "";
unsigned long logStartMs = 0, lastFlushMs = 0, lastActiveMs = 0;
uint32_t logRows = 0;
uint32_t rigLines = 0;                    // status lines received (for the USB debug readout)

// clock (set from the first browser that connects; ESP32 has no RTC)
bool  clockSet = false;
long  tzOffsetMin = 330;                  // replaced by the browser's time zone

// ================= HELPERS =================
int fieldIndex(const char *key) {
  for (byte i = 0; i < NF; i++) if (!strcmp(key, FIELD_KEYS[i])) return i;
  return -1;
}

bool rigActive() {
  return haveData && (cur[12] > 0 || cur[9] > 0 || cur[10] > 0 || cur[11] > 0 || cur[16] > 0 || cur[18] > 0);  // mode, pump, solenoids, glow, start sequence
}

String clockStr(bool forFile) {
  if (!clockSet) return "";
  time_t now = time(nullptr) + tzOffsetMin * 60;
  struct tm t;
  gmtime_r(&now, &t);
  char b[32];
  if (forFile) strftime(b, sizeof b, "%Y%m%d-%H%M%S", &t);
  else {
    struct timeval tv; gettimeofday(&tv, nullptr);
    strftime(b, sizeof b, "%H:%M:%S", &t);
    char ms[8]; snprintf(ms, sizeof ms, ".%01ld", (long)(tv.tv_usec / 100000));
    strcat(b, ms);
  }
  return String(b);
}

// ---------- live stream (Server-Sent Events) ----------
int liveCount() {
  int n = 0;
  for (int i = 0; i < MAX_LIVE; i++) if (live[i] && live[i].connected()) n++;
  return n;
}

void sendEvent(WiFiClient &c, const String &msg) {
  if (!c || !c.connected()) return;
  String frame = "data: " + msg + "\n\n";
  if (c.write((const uint8_t *)frame.c_str(), frame.length()) != frame.length()) c.stop();
}

void broadcast(const String &msg) {
  for (int i = 0; i < MAX_LIVE; i++) if (live[i] && live[i].connected()) sendEvent(live[i], msg);
}

void pushMsg(const String &m) {
  Serial.println(m);                           // also show on the ESP32's USB Serial Monitor
  msgs[msgHead] = m;
  msgHead = (msgHead + 1) % MSG_N;
  if (msgCount < MSG_N) msgCount++;
  broadcast("M|" + m);
}

String jsonEscape(const String &s) {
  String o; o.reserve(s.length() + 4);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if ((uint8_t)c >= 32) o += c;
  }
  return o;
}

// ================= LOGGING =================
void logStop(const char *why) {
  if (!logging) return;
  logFile.flush();
  logFile.close();
  logging = false;
  pushMsg(String("log saved: ") + logName.substring(1) + " (" + logRows + " rows) - " + why);
}

void logStart(bool byAuto) {
  if (logging) return;
  if (LittleFS.totalBytes() - LittleFS.usedBytes() < MIN_FREE_BYTES) {
    pushMsg("FAULT: log storage full - delete old logs on the Logging page");
    return;
  }
  String base = clockSet ? clockStr(true) : String("run-") + String(millis() / 1000);
  logName = "/" + base + ".csv";
  for (int n = 2; LittleFS.exists(logName); n++) logName = "/" + base + "-" + n + ".csv";
  logFile = LittleFS.open(logName, FILE_WRITE);
  if (!logFile) { pushMsg("FAULT: could not create log file"); return; }
  logFile.println(CSV_HEADER);
  logging = true;
  logStartedByAuto = byAuto;
  logStartMs = millis();
  lastFlushMs = millis();
  logRows = 0;
  pushMsg(String("logging to ") + logName.substring(1) + (byAuto ? " (auto)" : ""));
}

void logRow(const String &event) {
  if (!logging) return;
  String row; row.reserve(160);
  row += clockStr(false); row += ',';
  row += String((millis() - logStartMs) / 1000.0, 2);
  for (byte i = 0; i < NF; i++) {
    row += ',';
    if (event.length() == 0 && haveData && !isnan(cur[i])) row += String(cur[i], (i == 8) ? 3 : 1);
  }
  row += ',';
  if (event.length()) { String e = event; e.replace(",", ";"); e.replace("\"", "'"); row += e; }
  logFile.println(row);
  logRows++;
  if (millis() - lastFlushMs > 2000) {
    logFile.flush();
    lastFlushMs = millis();
    if (LittleFS.totalBytes() - LittleFS.usedBytes() < MIN_FREE_BYTES) logStop("storage full");
  }
}

// ================= HISTORY =================
void histAdd(uint32_t t) {
  if (!hist) return;
  if (++histSkip < histStride) return;
  histSkip = 0;
  if (histCount == HIST_N) {                   // compact: keep every other point
    for (int i = 0; i < HIST_N / 2; i++) hist[i] = hist[i * 2];
    histCount = HIST_N / 2;
    histStride *= 2;
  }
  hist[histCount].t = t;
  memcpy(hist[histCount].v, cur, sizeof(cur));
  histCount++;
}

// ================= RIG LINES =================
void handleRigLine(char *line) {
  size_t n = strlen(line);
  while (n && (line[n - 1] == '\r' || line[n - 1] == ' ')) line[--n] = 0;
  if (!n) return;
  lastRigMs = millis();

  if (!strncmp(line, "SP:", 3)) {               // status line
    float v[NF];
    for (byte i = 0; i < NF; i++) v[i] = NAN;
    char tmp[256]; strncpy(tmp, line, sizeof tmp); tmp[sizeof tmp - 1] = 0;
    for (char *tok = strtok(tmp, ","); tok; tok = strtok(nullptr, ",")) {
      char *colon = strchr(tok, ':');
      if (!colon) continue;
      *colon = 0;
      int idx = fieldIndex(tok);
      if (idx >= 0) v[idx] = (!strcmp(colon + 1, "nan")) ? NAN : atof(colon + 1);
    }
    memcpy(cur, v, sizeof cur);
    haveData = true;
    rigLines++;

    uint32_t t = millis();
    broadcast("D|" + String(t) + "|" + line);
    histAdd(t);

    // auto logging
    if (rigActive()) lastActiveMs = millis();
    if (autoLog && !logging && rigActive()) logStart(true);
    if (logging && logStartedByAuto && !rigActive() && millis() - lastActiveMs > AUTOLOG_STOP_AFTER_MS)
      logStop("rig idle");
    if (logging) logRow("");
  } else {                                       // ">> message" lines
    String m = String(line);
    if (m.startsWith(">>")) m = m.substring(2);
    m.trim();
    if (!m.length()) return;
    pushMsg(m);
    if (logging) logRow(m);
  }
}

void readRig() {
  while (RIG.available()) {
    char c = RIG.read();
    if (c == '\n') {
      rigLine[rigLen] = 0;
      handleRigLine(rigLine);
      rigLen = 0;
    } else if (rigLen < sizeof(rigLine) - 1) {
      rigLine[rigLen++] = c;
    } else {
      rigLen = 0;                                // overlong garbage line, drop it
    }
  }
}

// ================= LIVE STREAM + COMMANDS =================
String logStateJson() {
  String j = "{\"rec\":"; j += logging ? "true" : "false";
  j += ",\"auto\":"; j += autoLog ? "true" : "false";
  j += ",\"file\":\""; j += logging ? logName.substring(1) : ""; j += "\"";
  j += ",\"size\":"; j += logging ? String((uint32_t)logFile.size()) : "0";
  j += ",\"secs\":"; j += logging ? String((millis() - logStartMs) / 1000) : "0";
  j += ",\"used\":"; j += String((uint32_t)LittleFS.usedBytes());
  j += ",\"total\":"; j += String((uint32_t)LittleFS.totalBytes());
  j += ",\"link\":"; j += linkUp ? "true" : "false";
  j += ",\"clients\":"; j += String(liveCount());
  j += "}";
  return j;
}

void sendLogState() { broadcast("L|" + logStateJson()); }

// new browser opening the live stream
void acceptLive() {
  WiFiClient c = liveServer.available();
  if (!c) return;
  unsigned long t0 = millis();                 // read the request headers (up to the blank line)
  uint32_t tail = 0;
  while (c.connected() && millis() - t0 < 300) {
    if (!c.available()) { delay(1); continue; }
    tail = (tail << 8) | (uint8_t)c.read();
    if (tail == 0x0D0A0D0A) break;             // "\r\n\r\n"
  }
  int slot = -1;
  for (int i = 0; i < MAX_LIVE; i++) if (!live[i] || !live[i].connected()) { slot = i; break; }
  if (slot < 0) { c.print("HTTP/1.1 503 Busy\r\nAccess-Control-Allow-Origin: *\r\n\r\n"); c.stop(); return; }
  c.setNoDelay(true);
  c.print("HTTP/1.1 200 OK\r\n"
          "Content-Type: text/event-stream\r\n"
          "Cache-Control: no-cache\r\n"
          "Access-Control-Allow-Origin: *\r\n"
          "Connection: keep-alive\r\n\r\n"
          "retry: 1500\n\n");
  live[slot] = c;
  sendEvent(live[slot], "H|" + String(millis()));
  for (int i = 0; i < msgCount; i++) {         // replay recent messages
    int k = (msgHead - msgCount + i + MSG_N) % MSG_N;
    sendEvent(live[slot], "M|" + msgs[k]);
  }
  sendEvent(live[slot], "L|" + logStateJson());
}

// commands that only make things safer work without the PIN
bool isStop(const String &c) {
  String s = c; s.toLowerCase(); s.replace(" ", "");
  return s == "stop" || s == "abort";
}

void cors() { http.sendHeader("Access-Control-Allow-Origin", "*"); http.sendHeader("Cache-Control", "no-cache"); }

// /auth?pin=1234  -> 200 ok / 403
void handleAuth() {
  cors();
  if (http.arg("pin") == CONTROL_PIN) http.send(200, "text/plain", "ok");
  else http.send(403, "text/plain", "wrong PIN");
}

// /cmd?c=3000&pin=1234   (STOP needs no PIN)
void handleCmd() {
  cors();
  String c = http.arg("c");
  c.trim();
  if (!c.length() || c.length() > 22) { http.send(400, "text/plain", "bad command"); return; }
  if (!isStop(c) && http.arg("pin") != CONTROL_PIN) {
    http.send(403, "text/plain", "locked - enter the PIN to control the rig (STOP always works)");
    return;
  }
  RIG.print(c); RIG.print('\n');
  pushMsg("> " + c);
  if (logging) logRow("cmd: " + c);
  http.send(200, "text/plain", "ok");
}

// /time?ms=<epoch ms>&tz=<minutes east of UTC>   (first browser sets the clock)
void handleTime() {
  cors();
  if (!clockSet) {
    double ms = atof(http.arg("ms").c_str());
    if (ms > 1.6e12) {
      tzOffsetMin = http.arg("tz").toInt();
      struct timeval tv;
      tv.tv_sec = (time_t)(ms / 1000.0);
      tv.tv_usec = (suseconds_t)(fmod(ms, 1000.0) * 1000);
      settimeofday(&tv, nullptr);
      clockSet = true;
    }
  }
  http.send(200, "text/plain", "ok");
}

// /log?a=start|stop|autoon|autooff
void handleLogCtl() {
  cors();
  String a = http.arg("a");
  if (a == "start") logStart(false);
  else if (a == "stop") logStop("stopped");
  else if (a == "autoon")  { autoLog = true;  pushMsg("auto-logging ON"); }
  else if (a == "autooff") { autoLog = false; pushMsg("auto-logging OFF"); }
  sendLogState();
  http.send(200, "text/plain", "ok");
}

// ================= HTTP =================
void handleRoot() {
  http.sendHeader("Cache-Control", "no-cache");
  http.send_P(200, "text/html", INDEX_HTML);
}

// whole-run history as CSV: t_ms,SP,RPM,...
void handleHistory() {
  cors();                                      // also reachable from the stand-alone dashboard file
  http.setContentLength(CONTENT_LENGTH_UNKNOWN);
  http.send(200, "text/plain", "");
  String chunk; chunk.reserve(2400);
  chunk = "t";
  for (byte i = 0; i < NF; i++) { chunk += ','; chunk += FIELD_KEYS[i]; }
  chunk += '\n';
  for (int r = 0; r < histCount; r++) {
    chunk += String(hist[r].t);
    for (byte i = 0; i < NF; i++) {
      chunk += ',';
      if (!isnan(hist[r].v[i])) chunk += String(hist[r].v[i], i == 8 ? 3 : 1);
    }
    chunk += '\n';
    if (chunk.length() > 2000) { http.sendContent(chunk); chunk = ""; }
  }
  if (chunk.length()) http.sendContent(chunk);
  http.sendContent("");
}

void handleLogs() {
  cors();                                      // also reachable from the stand-alone dashboard file
  String j = "{\"files\":[";
  File root = LittleFS.open("/");
  bool first = true;
  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    String n = f.name();
    if (n.startsWith("/")) n = n.substring(1);
    if (!n.endsWith(".csv")) continue;
    if (!first) j += ',';
    first = false;
    j += "{\"n\":\"" + jsonEscape(n) + "\",\"s\":" + String((uint32_t)f.size()) + "}";
  }
  j += "],\"state\":" + logStateJson() + "}";
  http.sendHeader("Cache-Control", "no-cache");
  http.send(200, "application/json", j);
}

bool safeName(const String &n) {
  if (!n.length() || n.length() > 30 || !n.endsWith(".csv")) return false;
  for (size_t i = 0; i < n.length(); i++) {
    char c = n[i];
    if (!(isalnum(c) || c == '-' || c == '_' || c == '.')) return false;
  }
  return true;
}

void handleDownload() {
  cors();                                      // also reachable from the stand-alone dashboard file
  String n = http.arg("f");
  if (!safeName(n) || !LittleFS.exists("/" + n)) { http.send(404, "text/plain", "not found"); return; }
  if (logging && logName == "/" + n) logFile.flush();
  File f = LittleFS.open("/" + n, FILE_READ);
  http.sendHeader("Content-Disposition", "attachment; filename=\"gmach-" + n + "\"");
  http.streamFile(f, "text/csv");
  f.close();
}

void handleDelete() {
  cors();                                      // also reachable from the stand-alone dashboard file
  String n = http.arg("f");
  if (http.arg("pin") != CONTROL_PIN) { http.send(403, "text/plain", "wrong PIN"); return; }
  if (!safeName(n)) { http.send(400, "text/plain", "bad name"); return; }
  if (logging && logName == "/" + n) { http.send(409, "text/plain", "file is being recorded - stop logging first"); return; }
  LittleFS.remove("/" + n);
  pushMsg("deleted log " + n);
  http.send(200, "text/plain", "ok");
}

// ================= SETUP / LOOP =================
void setup() {
  Serial.begin(115200);
  RIG.setRxBufferSize(4096);
  RIG.begin(RIG_BAUD, SERIAL_8N1, RIG_RX, RIG_TX);

  for (byte i = 0; i < NF; i++) cur[i] = NAN;
  hist = (Sample *)malloc(sizeof(Sample) * HIST_N);

  if (!LittleFS.begin(true)) Serial.println("LittleFS mount failed");

  if (JOIN_EXISTING_WIFI) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(STA_SSID, STA_PASS);
    Serial.print("Joining Wi-Fi");
    for (int i = 0; i < 40 && WiFi.status() != WL_CONNECTED; i++) { delay(250); Serial.print('.'); }
    Serial.println();
    Serial.print("Dashboard: http://"); Serial.println(WiFi.localIP());
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.print("Hotspot "); Serial.print(AP_SSID);
    Serial.print("  dashboard: http://"); Serial.println(WiFi.softAPIP());
  }
  WiFi.setSleep(false);                        // lower latency
  MDNS.begin(MDNS_NAME);

  http.on("/", handleRoot);
  http.on("/history", handleHistory);
  http.on("/logs", handleLogs);
  http.on("/dl", handleDownload);
  http.on("/delete", handleDelete);
  http.on("/auth", handleAuth);
  http.on("/cmd", handleCmd);
  http.on("/time", handleTime);
  http.on("/log", handleLogCtl);
  http.onNotFound([]() { http.sendHeader("Location", "/"); http.send(302, "text/plain", ""); });
  http.begin();

  liveServer.begin();
  MDNS.addService("http", "tcp", 80);
}

void loop() {
  readRig();
  acceptLive();
  http.handleClient();
  readRig();

  bool up = lastRigMs && millis() - lastRigMs < LINK_TIMEOUT_MS;   // nothing received yet = not connected
  if (up != linkUp) {
    linkUp = up;
    pushMsg(up ? "rig connected" : "FAULT: no data from rig - check Mega to ESP32 wiring");
  }

  // USB Serial Monitor (115200): one line every 2 s showing whether the rig link works
  static unsigned long lastDbg = 0;
  static uint32_t lastLines = 0;
  if (millis() - lastDbg > 2000) {
    lastDbg = millis();
    Serial.print(linkUp ? "[rig OK] " : "[NO RIG DATA] ");
    Serial.print((rigLines - lastLines) / 2.0, 1); Serial.print(" lines/s");
    if (haveData) { Serial.print("  RPM "); Serial.print(cur[1], 0); Serial.print("  T "); Serial.print(cur[3], 1); }
    Serial.print("  viewers "); Serial.print(liveCount());
    Serial.print(logging ? "  logging" : "");
    Serial.println();
    lastLines = rigLines;
  }

  static unsigned long lastState = 0;
  if (millis() - lastState > 1000) { lastState = millis(); sendLogState(); }   // also keeps streams alive
}
