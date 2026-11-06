/*
  TEST RIG - Arduino Mega
  ---------------------------------------------------------------
  RPM control (PID + autotune) via servo air valve, plus:
  thermocouple (MAX31856), pitot pressures (ADS1115),
  fuel pump (BLDC ESC), two solenoid valves (relay IN7 / IN8).

  PINS
    D2   Hall sensor signal (RPM)
    D3   Air-valve servo signal
    D9   Pump ESC signal (white wire)
    A0   Relay IN7  (solenoid 7)
    A1   Relay IN8  (solenoid 8)
    A3   Relay      (glow plug)  - relay COM must come from a ~1.5-1.8 V supply, NOT 12 V
    D50  MAX31856 SDO   D51 SDI   D52 SCK   D53 CS
    D20  ADS1115 SDA    D21 SCL   (ADS1115 A0 = total, A1 = static)

  LIBRARIES (Library Manager)
    Adafruit MAX31856, Adafruit ADS1X15  (Servo, EEPROM, SPI, Wire built in)

  COMMANDS (Serial Monitor, 115200)
    3000        hold 3000 RPM          0       close air valve
    tune 3000   autotune at 3000 RPM    table   show stored gains
    clear       delete stored gains     sched on|off
    kp x | ki x | kd x                  manual gains
    pump 40     pump throttle 40 %      pump 0  pump off
    7on 7off 8on 8off                   solenoid valves
    glowon glowoff                      glow plug relay (switches itself off after GLOW_MAX_S)
    start       START SEQUENCE: spin up to 4000 RPM, then waits for 'go'
    go          (when asked) glow plug on -> 5 s preheat -> pulse starter fuel (solenoid 7) -> open fully
    abort       cancel the start sequence: starter fuel closed, glow off (air and pump unchanged)
    hb          heartbeat, sent every second by the dashboard / Pi. Once seen, if it stops for
                LINK_TIMEOUT_MS the rig goes safe (see LINK LOSS below)
    pzero       re-zero pressure sensors (valve must be closed, no flow)
    qdb 8       velocity deadband in Pa (dynamic pressure below this reads 0 m/s)
    autozero on|off   auto re-zero pressures while air valve closed and RPM 0
    t 30        fixed total temp for velocity calc   t tc  use thermocouple
    stop        EVERYTHING OFF (air valve closed, pump 0, solenoids off)
*/

#include <Servo.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MAX31856.h>
#include <Adafruit_ADS1X15.h>

// ================= PINS =================
const byte SERVO_PIN  = 3;
const byte HALL_PIN   = 2;            // INT4 on Mega (port E4)
const byte ESC_PIN    = 9;
const byte RELAY7_PIN = A0;
const byte RELAY8_PIN = A1;
const byte GLOW_PIN   = A3;
const byte TC_CS_PIN  = 53;
const byte MAGNETS_PER_REV = 1;

// ================= AIR VALVE =================
const float VALVE_CLOSED_DEG = 90.0;  // servo angle = valve closed
const float VALVE_OPEN_DEG   = 0.0;   // servo angle = valve fully open
const float MAX_SLEW_DEG_S   = 30.0;  // max valve speed while holding RPM
const float VALVE_CRACK_DEG  = 40.0;  // just below where the ball valve starts passing air

// ================= MANUAL PID (used when no tuned gains stored) =================
float Kp = 0.005;
float Ki = 0.001;
float Kd = 0.0;
const unsigned long CONTROL_MS = 50;  // 20 Hz control loop
const unsigned long PRINT_MS   = 200;
const float SP_RAMP_RPM_S = 1000;

// ================= AUTOTUNE =================
const float TUNE_RELAY_DEG   = 10.0;
const float TUNE_HYST_FRAC   = 0.02;
const float TUNE_HYST_MIN    = 30.0;
const float TUNE_SEEK_DEG_S  = 1.0;
const unsigned long TUNE_STUCK_MS = 6000;
const float TUNE_STUCK_SHIFT_DEG = 4.0;
const byte  TUNE_SKIP_CYCLES = 2;
const byte  TUNE_MEAS_CYCLES = 4;
const unsigned long TUNE_TIMEOUT_MS = 150000;
const unsigned long TUNE_MIN_DWELL_MS = 300;
const float TUNE_MIN_TU_S = 0.4;
const float TUNE_MAX_TU_S = 30.0;
const float TUNE_MAX_AMP_FRAC = 0.5;

// ================= SAFETY =================
const float OVERSPEED_RPM = 30000;

// ================= RPM MEASUREMENT =================
const unsigned long MAX_RPM = 40000;
const unsigned long MIN_PERIOD_US = 60000000UL / (MAX_RPM * MAGNETS_PER_REV);
const unsigned long STOP_TIMEOUT_US = 2000000UL;
const float CHATTER_FRACTION = 0.5;

// ================= PUMP (ESC) =================
const int   ESC_MIN_US = 1000;
const int   ESC_MAX_US = 2000;
const float PUMP_RAMP_PCT_S = 50;
const float PUMP_MAX_PCT    = 100;

// ================= RELAYS =================
const bool RELAY_ON  = LOW;           // board is active-low
const unsigned long GLOW_MAX_S = 30;  // glow plug switches itself off after this many seconds

// ================= START SEQUENCE =================
const float SEQ_RPM = 4000;                       // spin-up target on air
const float SEQ_BAND_FRAC = 0.03;                 // "reached" = within 3 % (min 100 RPM) ...
const unsigned long SEQ_STABLE_MS = 2000;         // ... for this long
const unsigned long SEQ_SPINUP_TIMEOUT_S = 60;    // give up if not reached in time
const unsigned long SEQ_PREHEAT_S = 5;            // glow plug on this long before fuel
const byte  STUTTER_PULSES = 6;                   // starter-fuel pulses before opening fully
const unsigned long STUTTER_ON_MS  = 200;
const unsigned long STUTTER_OFF_MS = 300;
const byte STARTER_SOL = 7;                       // starter fuel = solenoid 7 (A0)

// ================= LINK LOSS =================
// armed by the first "hb" from the dashboard / Pi; plain Serial Monitor use is unaffected
const unsigned long LINK_TIMEOUT_MS = 3000;
const bool LINK_LOSS_FULL_STOP = false;           // false: starter fuel closed + glow off
                                                  // true:  everything off (air valve, pump, solenoids, glow)
const bool RELAY_OFF = HIGH;

// ================= PRESSURE (ADS1115) =================
// q (dynamic pressure) is read DIFFERENTIALLY: ADS1115 measures A0 - A1 directly at
// +/-0.256 V (16x gain). ~2 Pa per count instead of ~47 Pa, and supply sag / ground
// bounce hit both sensors equally so they cancel. Both transducers must be the same range.
const float PV_MIN = 0.5;             // transducer output at zero pressure (V)
const float PV_MAX = 4.5;             // output at full scale (V)
const float PT_MAX_BAR = 10.0;        // ADS A0: total-pressure sensor full scale
const float PS_MAX_BAR = 10.0;        // ADS A1: static-pressure sensor full scale (same as PT)
const float P_ATM_PA   = 101325.0;    // local atmospheric pressure
const float GAMMA = 1.4, R_AIR = 287.05;
const float P_FILTER = 0.05;          // smoothing for Pt / Ps readings (lower = smoother)
const float Q_FILTER = 0.03;          // smoothing for differential q (~190 samples/s)
float       qDeadbandPa = 8;          // |q| below this reads 0 m/s (live: qdb 8)
bool        autoZero = true;          // re-zero automatically while air valve closed & RPM 0
const unsigned long AUTOZERO_AFTER_MS = 3000;
const byte  SPIKE_N = 7;              // median window: rejects glitches up to 3 samples long

// ================= SERIAL OUTPUT =================
// wired only: everything goes to the USB port (open gmach_dashboard.html in Chrome/Edge)
#define OUT Serial

// ================= OBJECTS =================
Servo valve;
Servo esc;
Adafruit_MAX31856 tc(TC_CS_PIN);
Adafruit_ADS1115 ads;

// ================= RPM ISR =================
const byte NP = 5;
volatile unsigned long periods[NP];
volatile byte pIdx = 0, pCount = 0;
volatile unsigned long lastPulseUs = 0;
volatile unsigned long minValidUs = 0;

void hallISR() {
  unsigned long now = micros();
  if (PINE & _BV(PE4)) return;
  unsigned long dt = now - lastPulseUs;
  if (dt < MIN_PERIOD_US) return;
  if (dt < minValidUs) return;
  lastPulseUs = now;
  if (dt < STOP_TIMEOUT_US) {
    periods[pIdx] = dt;
    pIdx = (pIdx + 1) % NP;
    if (pCount < NP) pCount++;
  }
}

// ================= GAIN SCHEDULE =================
struct GainPoint { float rpm, kp, ki, kd; };
const byte MAX_SCHED = 6;
GainPoint sched[MAX_SCHED];
byte nSched = 0;
bool useSched = true;
const byte EE_MAGIC = 0x5B;

void saveSched() {
  EEPROM.update(0, EE_MAGIC);
  EEPROM.update(1, nSched);
  EEPROM.put(2, sched);
}

void loadSched() {
  if (EEPROM.read(0) == EE_MAGIC) {
    nSched = EEPROM.read(1);
    if (nSched > MAX_SCHED) nSched = 0;
    EEPROM.get(2, sched);
  }
}

void printTable() {
  if (nSched == 0) { OUT.println(F(">> no tuned gains stored")); return; }
  OUT.print(F(">> gain schedule ("));
  OUT.print(useSched ? F("ON") : F("OFF"));
  OUT.println(F("):"));
  for (byte i = 0; i < nSched; i++) {
    OUT.print(F(">>   ")); OUT.print(sched[i].rpm, 0);
    OUT.print(F(" RPM  kp=")); OUT.print(sched[i].kp, 5);
    OUT.print(F("  ki="));     OUT.print(sched[i].ki, 5);
    OUT.print(F("  kd="));     OUT.println(sched[i].kd, 5);
  }
}

// plain floats (not GainPoint) so the Arduino IDE's auto-prototypes compile
void addSchedPoint(float rpm, float kp, float ki, float kd) {
  GainPoint g;
  g.rpm = rpm; g.kp = kp; g.ki = ki; g.kd = kd;
  for (byte i = 0; i < nSched; i++) {
    if (fabs(sched[i].rpm - g.rpm) < 0.1 * g.rpm) { sched[i] = g; saveSched(); return; }
  }
  if (nSched < MAX_SCHED) {
    sched[nSched++] = g;
  } else {
    byte best = 0;
    for (byte i = 1; i < nSched; i++)
      if (fabs(sched[i].rpm - g.rpm) < fabs(sched[best].rpm - g.rpm)) best = i;
    sched[best] = g;
  }
  for (byte i = 1; i < nSched; i++) {
    GainPoint v = sched[i]; byte j = i;
    while (j > 0 && sched[j - 1].rpm > v.rpm) { sched[j] = sched[j - 1]; j--; }
    sched[j] = v;
  }
  saveSched();
}

void getGains(float sp, float &kp, float &ki, float &kd) {
  if (!useSched || nSched == 0) { kp = Kp; ki = Ki; kd = Kd; return; }
  if (sp <= sched[0].rpm) { kp = sched[0].kp; ki = sched[0].ki; kd = sched[0].kd; return; }
  byte L = nSched - 1;
  if (sp >= sched[L].rpm) { kp = sched[L].kp; ki = sched[L].ki; kd = sched[L].kd; return; }
  for (byte i = 0; i < L; i++) {
    if (sp <= sched[i + 1].rpm) {
      float t = (sp - sched[i].rpm) / (sched[i + 1].rpm - sched[i].rpm);
      kp = sched[i].kp + t * (sched[i + 1].kp - sched[i].kp);
      ki = sched[i].ki + t * (sched[i + 1].ki - sched[i].ki);
      kd = sched[i].kd + t * (sched[i + 1].kd - sched[i].kd);
      return;
    }
  }
}

// ================= STATE =================
const byte M_IDLE = 0, M_HOLD = 1, M_SEEK = 2, M_RELAY = 3;
byte mode = M_IDLE;

float setpoint = 0, spRamped = 0;
float rpmF = 0, lastRpm = 0;
float integral = 0;
float opening = 0;

float tuneSp, tuneBias, tuneHyst, pkMax, pkMin;
float ampBuf[TUNE_MEAS_CYCLES], perBuf[TUNE_MEAS_CYCLES];
bool relayHigh;
byte cycleCount;
unsigned long tuneStartMs, lastSwitchMs, cycleStartMs, tHighMs, fullOpenSinceMs;

float pumpTarget = 0, pumpPct = 0;
bool  faultActive = false;            // latched until the next start / stop
bool relay7 = false, relay8 = false;
bool glowOn = false;
unsigned long glowStartMs = 0;

// start sequence: 0 off, 1 spinning up, 2 waiting for 'go', 3 glow preheat, 4 pulsing fuel, 5 starter fuel on
byte seqStep = 0;
unsigned long seqStepMs = 0, seqStableSince = 0, stutterMs = 0;
byte stutterCount = 0;
bool stutterOn = false;

bool hbSeen = false, linkLost = false;
unsigned long lastHbMs = 0;

bool  tcOk = false;
float tcTemp = NAN;
uint8_t tcFaultLast = 0;

bool  adsOk = false;
float vPt = NAN, vPs = NAN;           // filtered single-ended voltages (A0, A1)
float vDiff = NAN;                    // filtered differential voltage A0 - A1
float ptZeroPa = 0, psZeroPa = 0, qZeroPa = 0;
float ptPa = NAN, psPa = NAN, qPa = NAN, velocity = NAN, mach = NAN;   // qPa signed, zero-corrected
float tTotalFixedC = 30.0;
bool  useTcForVelocity = false;
byte  adsSlot = 0;                    // position in the read sequence
byte  adsKind = 0;                    // what the running conversion is: 0 diff, 1 A0, 2 A1
bool  adsBusy = false;
unsigned long idleSinceMs = 0;

// ================= RPM =================
float measureRPM() {
  unsigned long p[NP];
  noInterrupts();
  byte c = pCount;
  for (byte i = 0; i < c; i++) p[i] = periods[i];
  unsigned long last = lastPulseUs;
  interrupts();

  unsigned long since = micros() - last;
  if (since > STOP_TIMEOUT_US) {
    noInterrupts(); pCount = 0; pIdx = 0; minValidUs = 0; interrupts();
    return 0;
  }
  if (c < 2) { noInterrupts(); minValidUs = 0; interrupts(); return 0; }

  for (byte i = 1; i < c; i++) {
    unsigned long v = p[i]; byte j = i;
    while (j > 0 && p[j - 1] > v) { p[j] = p[j - 1]; j--; }
    p[j] = v;
  }
  unsigned long period = p[c / 2];

  unsigned long lock = (since > 3 * period) ? 0 : (unsigned long)(period * CHATTER_FRACTION);
  noInterrupts(); minValidUs = lock; interrupts();

  if (since > period) period = since;
  return 60000000.0 / ((float)period * MAGNETS_PER_REV);
}

// ================= AIR VALVE =================
void writeValve(float openDeg) {
  openDeg = constrain(openDeg, 0, 90);
  float angle = VALVE_CLOSED_DEG + (VALVE_OPEN_DEG - VALVE_CLOSED_DEG) * openDeg / 90.0;
  int us = 544 + (int)(angle * 1856.0 / 180.0);
  valve.writeMicroseconds(us);
}

void closeValve() {
  mode = M_IDLE;
  integral = 0;
  opening = 0;
  writeValve(0);
}

void setGlow(bool on);

void fault(const __FlashStringHelper *msg) {
  closeValve();
  if (glowOn) setGlow(false);
  if (seqStep) { digitalWrite(RELAY7_PIN, RELAY_OFF); relay7 = false; seqStep = 0; }
  faultActive = true;
  OUT.print(F(">> FAULT: "));
  OUT.println(msg);
  OUT.println(F(">> Air valve closed."));
}

// ================= PUMP =================
void writePump(float pct) {
  esc.writeMicroseconds(ESC_MIN_US + (int)(pct * (ESC_MAX_US - ESC_MIN_US) / 100.0));
}

void pumpStep(float dt) {
  float step = PUMP_RAMP_PCT_S * dt;
  pumpPct += constrain(pumpTarget - pumpPct, -step, step);
  writePump(pumpPct);
}

// ================= RELAYS =================
void setRelay(byte which, bool on) {
  if (which == 7) { relay7 = on; digitalWrite(RELAY7_PIN, on ? RELAY_ON : RELAY_OFF); }
  if (which == 8) { relay8 = on; digitalWrite(RELAY8_PIN, on ? RELAY_ON : RELAY_OFF); }
  OUT.print(F(">> solenoid ")); OUT.print(which); OUT.println(on ? F(" ON") : F(" OFF"));
}

void setGlow(bool on) {
  glowOn = on;
  if (on) glowStartMs = millis();
  digitalWrite(GLOW_PIN, on ? RELAY_ON : RELAY_OFF);
  OUT.print(F(">> glow plug "));
  if (on) { OUT.print(F("ON (auto-off in ")); OUT.print((int)GLOW_MAX_S); OUT.println(F(" s)")); }
  else OUT.println(F("OFF"));
}

void glowStep() {
  if (glowOn && millis() - glowStartMs > GLOW_MAX_S * 1000UL) {
    setGlow(false);
    OUT.println(F(">> glow plug timed out"));
  }
}

void allOff() {
  faultActive = false;
  seqStep = 0;
  closeValve();
  pumpTarget = pumpPct = 0;
  writePump(0);
  setRelay(7, false);
  setRelay(8, false);
  setGlow(false);
  OUT.println(F(">> STOP: air valve closed, pump off, solenoids off, glow off"));
}

// ================= START SEQUENCE =================
void seqSet(byte st) { seqStep = st; seqStepMs = millis(); }

void startHold(float sp);

void seqStart() {
  if (seqStep) { OUT.println(F(">> START already running - 'abort' to cancel")); return; }
  startHold(SEQ_RPM);
  seqStableSince = 0;
  seqSet(1);
  OUT.print(F(">> START: spinning up to ")); OUT.print(SEQ_RPM, 0); OUT.println(F(" RPM on air"));
}

void seqAbort(const __FlashStringHelper *why) {
  if (!seqStep) return;
  digitalWrite(RELAY7_PIN, RELAY_OFF); relay7 = false;   // starter fuel closed first
  if (glowOn) setGlow(false);
  seqStep = 0;
  OUT.print(F(">> START aborted: ")); OUT.println(why);
}

void seqGo() {
  if (seqStep != 2) { OUT.println(F(">> nothing to confirm - 'go' only works when the start sequence asks for it")); return; }
  setGlow(true);
  seqSet(3);
  OUT.print(F(">> START: glow plug on, preheating ")); OUT.print((int)SEQ_PREHEAT_S); OUT.println(F(" s"));
}

void seqStepFn() {
  if (!seqStep) return;
  if (mode != M_HOLD) { seqAbort(F("air valve stopped holding RPM")); return; }
  unsigned long inStep = millis() - seqStepMs;

  switch (seqStep) {
    case 1: {                                   // spinning up
      float band = max(100.0f, SEQ_BAND_FRAC * SEQ_RPM);
      if (fabs(rpmF - SEQ_RPM) <= band) {
        if (!seqStableSince) seqStableSince = millis();
        if (millis() - seqStableSince >= SEQ_STABLE_MS) {
          seqSet(2);
          OUT.print(F(">> START: holding ")); OUT.print(SEQ_RPM, 0);
          OUT.println(F(" RPM - READY FOR GLOW PLUG (press Start glow / type go)"));
        }
      } else seqStableSince = 0;
      if (seqStep == 1 && inStep > SEQ_SPINUP_TIMEOUT_S * 1000UL) seqAbort(F("did not reach target RPM in time"));
      break;
    }
    case 2:                                     // waiting for the operator
      break;
    case 3:                                     // glow preheat
      if (!glowOn) { seqAbort(F("glow plug switched off during preheat")); break; }
      if (inStep >= SEQ_PREHEAT_S * 1000UL) {
        seqSet(4);
        stutterCount = 0; stutterOn = true; stutterMs = millis();
        digitalWrite(RELAY7_PIN, RELAY_ON); relay7 = true;
        OUT.println(F(">> START: pulsing starter fuel (solenoid 7)"));
      }
      break;
    case 4:                                     // stutter solenoid 7
      if (stutterOn && millis() - stutterMs >= STUTTER_ON_MS) {
        digitalWrite(RELAY7_PIN, RELAY_OFF); relay7 = false;
        stutterOn = false; stutterMs = millis();
        stutterCount++;
        if (stutterCount >= STUTTER_PULSES) {
          seqSet(5);
          digitalWrite(RELAY7_PIN, RELAY_ON); relay7 = true;
          OUT.println(F(">> START: starter fuel OPEN - glow plug on - set pump throttle as needed"));
        }
      } else if (!stutterOn && millis() - stutterMs >= STUTTER_OFF_MS) {
        digitalWrite(RELAY7_PIN, RELAY_ON); relay7 = true;
        stutterOn = true; stutterMs = millis();
      }
      break;
    case 5:                                     // starter fuel on: stays until abort / stop / fault
      break;
  }
}

// ================= LINK LOSS =================
void linkStep() {
  if (!hbSeen || linkLost || millis() - lastHbMs < LINK_TIMEOUT_MS) return;
  linkLost = true;
  OUT.println(F(">> FAULT: link to dashboard lost - going safe"));
  if (LINK_LOSS_FULL_STOP) { allOff(); return; }
  if (seqStep) seqAbort(F("link to dashboard lost"));
  if (relay7) setRelay(7, false);                  // starter fuel closed
  if (glowOn) setGlow(false);
}

// ================= HOLD (PID) =================
void startHold(float sp) {
  faultActive = false;
  if (mode == M_IDLE) { integral = VALVE_CRACK_DEG; opening = VALVE_CRACK_DEG; }
  else if (mode != M_HOLD) { integral = opening; }
  if (mode != M_HOLD) { spRamped = rpmF; lastRpm = rpmF; }
  setpoint = sp;
  mode = M_HOLD;
}

void holdStep(float dt) {
  float rampStep = SP_RAMP_RPM_S * dt;
  spRamped += constrain(setpoint - spRamped, -rampStep, rampStep);

  float kp, ki, kd;
  getGains(spRamped, kp, ki, kd);

  float err  = spRamped - rpmF;
  float P    = kp * err;
  float D    = -kd * (rpmF - lastRpm) / dt;
  float newI = integral + ki * err * dt;
  float out  = P + newI + D;

  if (!((out > 90 && err > 0) || (out < 0 && err < 0))) integral = newI;
  integral = constrain(integral, 0, 90);

  float target = constrain(P + integral + D, 0, 90);
  float maxStep = MAX_SLEW_DEG_S * dt;
  opening += constrain(target - opening, -maxStep, maxStep);
  writeValve(opening);
}

// ================= AUTOTUNE =================
void beginRelay() {
  tuneBias = opening;
  relayHigh = false;
  lastSwitchMs = millis();
  cycleStartMs = 0;
  pkMax = pkMin = rpmF;
  mode = M_RELAY;
  OUT.print(F(">> TUNE: reached target, relay test started (bias "));
  OUT.print(tuneBias, 1);
  OUT.println(F(" deg)"));
}

void startTune(float sp) {
  faultActive = false;
  if (mode == M_IDLE) { integral = 0; opening = VALVE_CRACK_DEG; }
  tuneSp = sp;
  tuneHyst = max(TUNE_HYST_MIN, sp * TUNE_HYST_FRAC);
  tuneStartMs = millis();
  fullOpenSinceMs = 0;
  cycleCount = 0;
  OUT.print(F(">> TUNE at ")); OUT.print(sp, 0); OUT.println(F(" RPM - keep air supply steady"));
  if (rpmF >= sp) beginRelay();
  else { mode = M_SEEK; OUT.println(F(">> TUNE: opening valve to find target...")); }
}

void seekStep(float dt) {
  opening = min(opening + TUNE_SEEK_DEG_S * dt, 90.0f);
  writeValve(opening);
  if (rpmF >= tuneSp) { beginRelay(); return; }
  if (opening >= 90) {
    if (!fullOpenSinceMs) fullOpenSinceMs = millis();
    if (millis() - fullOpenSinceMs > 5000) fault(F("can't reach tune RPM with valve fully open"));
  }
}

float medianOf(float *v) {
  float s[TUNE_MEAS_CYCLES];
  for (byte i = 0; i < TUNE_MEAS_CYCLES; i++) s[i] = v[i];
  for (byte i = 1; i < TUNE_MEAS_CYCLES; i++) {
    float x = s[i]; byte j = i;
    while (j > 0 && s[j - 1] > x) { s[j] = s[j - 1]; j--; }
    s[j] = x;
  }
  return (s[(TUNE_MEAS_CYCLES - 1) / 2] + s[TUNE_MEAS_CYCLES / 2]) / 2.0;
}

void finishTune(float hi, float lo) {
  float a  = medianOf(ampBuf);
  float Tu = medianOf(perBuf);
  float d  = (hi - lo) / 2.0;

  OUT.print(F(">> TUNE measured: amplitude=")); OUT.print(a, 0);
  OUT.print(F(" RPM  period=")); OUT.print(Tu, 2); OUT.println(F(" s"));

  if (a < 1 || d <= 0) { fault(F("autotune failed: no usable oscillation - nothing saved")); return; }
  if (Tu < TUNE_MIN_TU_S) { fault(F("autotune rejected: swings too fast, RPM signal noisy - nothing saved")); return; }
  if (Tu > TUNE_MAX_TU_S) { fault(F("autotune rejected: swings too slow - nothing saved")); return; }
  if (a > TUNE_MAX_AMP_FRAC * tuneSp) { fault(F("autotune rejected: swing too large, lower TUNE_RELAY_DEG - nothing saved")); return; }

  float denom = (a > tuneHyst * 1.1) ? sqrt(a * a - tuneHyst * tuneHyst) : a;
  float Ku = 4.0 * d / (PI * denom);

  float tkp = Ku / 2.2;               // Tyreus-Luyben (conservative)
  float Ti  = 2.2 * Tu;
  float Td  = Tu / 6.3;
  float tki = tkp / Ti;
  float tkd = tkp * Td;

  OUT.println(F(">> TUNE complete"));
  OUT.print(F(">>   Ku=")); OUT.print(Ku, 5);
  OUT.print(F("  Tu=")); OUT.print(Tu, 2); OUT.println(F(" s"));
  OUT.print(F(">>   kp=")); OUT.print(tkp, 5);
  OUT.print(F("  ki=")); OUT.print(tki, 5);
  OUT.print(F("  kd=")); OUT.println(tkd, 5);

  addSchedPoint(tuneSp, tkp, tki, tkd);
  useSched = true;
  printTable();

  opening = constrain(tuneBias, 0, 90);
  startHold(tuneSp);
  OUT.print(F(">> now holding ")); OUT.print(tuneSp, 0); OUT.println(F(" RPM"));
}

void relayStep() {
  unsigned long now = millis();
  float hi = constrain(tuneBias + TUNE_RELAY_DEG, 0, 90);
  float lo = constrain(tuneBias - TUNE_RELAY_DEG, 0, 90);

  if (rpmF > pkMax) pkMax = rpmF;
  if (rpmF < pkMin) pkMin = rpmF;

  bool dwellOk = (now - lastSwitchMs) >= TUNE_MIN_DWELL_MS;

  if (now - lastSwitchMs > TUNE_STUCK_MS) {
    if (!relayHigh && rpmF > tuneSp) tuneBias -= TUNE_STUCK_SHIFT_DEG;
    else if (relayHigh && rpmF < tuneSp) tuneBias += TUNE_STUCK_SHIFT_DEG;
    tuneBias = constrain(tuneBias, 0, 90);
    lastSwitchMs = now;
    cycleStartMs = 0;
    pkMax = pkMin = rpmF;
    OUT.print(F(">> TUNE: RPM not crossing back, balance point moved to "));
    OUT.print(tuneBias, 1); OUT.println(F(" deg"));
  }

  if (dwellOk && relayHigh && rpmF > tuneSp + tuneHyst) {
    relayHigh = false;
    tHighMs = now - lastSwitchMs;
    lastSwitchMs = now;
  } else if (dwellOk && !relayHigh && rpmF < tuneSp - tuneHyst) {
    unsigned long tLowMs = now - lastSwitchMs;
    relayHigh = true;
    lastSwitchMs = now;
    if (cycleStartMs) {
      cycleCount++;
      if (cycleCount > TUNE_SKIP_CYCLES) {
        byte k = cycleCount - TUNE_SKIP_CYCLES - 1;
        if (k < TUNE_MEAS_CYCLES) {
          ampBuf[k] = (pkMax - pkMin) / 2.0;
          perBuf[k] = (now - cycleStartMs) / 1000.0;
        }
      } else if (tHighMs + tLowMs > 0) {
        tuneBias -= 0.5 * TUNE_RELAY_DEG * ((float)tLowMs - (float)tHighMs) / (float)(tHighMs + tLowMs);
        tuneBias = constrain(tuneBias, 0, 90);
      }
      OUT.print(F(">> TUNE: cycle ")); OUT.print(cycleCount);
      OUT.print(F("/")); OUT.println(TUNE_SKIP_CYCLES + TUNE_MEAS_CYCLES);
    }
    cycleStartMs = now;
    pkMax = pkMin = rpmF;
    if (cycleCount >= TUNE_SKIP_CYCLES + TUNE_MEAS_CYCLES) { finishTune(hi, lo); return; }
  }

  opening = relayHigh ? hi : lo;
  writeValve(opening);
}

// ================= THERMOCOUPLE =================
void tcStep() {
  if (!tcOk) return;
  uint8_t f = tc.readFault();
  if (f) {
    tcTemp = NAN;
    if (f != tcFaultLast) {
      OUT.print(F(">> TC FAULT:"));
      if (f & MAX31856_FAULT_OPEN)    OUT.print(F(" thermocouple open"));
      if (f & MAX31856_FAULT_OVUV)    OUT.print(F(" over/under voltage"));
      if (f & MAX31856_FAULT_TCRANGE) OUT.print(F(" TC out of range"));
      if (f & MAX31856_FAULT_CJRANGE) OUT.print(F(" cold junction out of range"));
      OUT.println();
    }
  } else {
    if (tcFaultLast) OUT.println(F(">> TC fault cleared"));
    tcTemp = tc.readThermocoupleTemperature();
  }
  tcFaultLast = f;
}

// ================= PRESSURE / VELOCITY =================
const float PA_PER_VOLT = (PT_MAX_BAR * 1e5) / (PV_MAX - PV_MIN);

float voltsToPa(float v, float maxBar) {
  return (v - PV_MIN) * (maxBar * 1e5) / (PV_MAX - PV_MIN);
}
bool voltsOk(float v) { return v > 0.3 && v < 4.8; }

// median-of-N spike rejection, one buffer per measurement kind
float medianFilter(byte kind, float v) {
  static float raw[3][SPIKE_N];
  static byte rIdx[3] = {0, 0, 0}, rCnt[3] = {0, 0, 0};
  raw[kind][rIdx[kind]] = v;
  rIdx[kind] = (rIdx[kind] + 1) % SPIKE_N;
  if (rCnt[kind] < SPIKE_N) rCnt[kind]++;
  float s[SPIKE_N];
  byte n = rCnt[kind];
  for (byte i = 0; i < n; i++) s[i] = raw[kind][i];
  for (byte i = 1; i < n; i++) {
    float x = s[i]; byte j = i;
    while (j > 0 && s[j - 1] > x) { s[j] = s[j - 1]; j--; }
    s[j] = x;
  }
  return s[n / 2];
}

// non-blocking read sequence: mostly differential q, with Pt and Ps in between
//   slots: D D D A0 D D D A1  -> ~190 q samples/s, ~30 Pt and Ps samples/s at 250 SPS
const byte SEQ[8] = {0, 0, 0, 1, 0, 0, 0, 2};

void adsStep() {
  if (!adsOk) return;
  if (!adsBusy) {
    adsKind = SEQ[adsSlot];
    if (adsKind == 0) {
      ads.setGain(GAIN_SIXTEEN);                       // +/-0.256 V, 7.8 uV/count
      ads.startADCReading(ADS1X15_REG_CONFIG_MUX_DIFF_0_1, false);
    } else {
      ads.setGain(GAIN_TWOTHIRDS);                     // +/-6.144 V
      ads.startADCReading(adsKind == 1 ? ADS1X15_REG_CONFIG_MUX_SINGLE_0
                                       : ADS1X15_REG_CONFIG_MUX_SINGLE_1, false);
    }
    adsBusy = true;
    return;
  }
  if (!ads.conversionComplete()) return;
  int16_t counts = ads.getLastConversionResults();
  adsBusy = false;
  adsSlot = (adsSlot + 1) % 8;

  if (adsKind == 0) {
    float v = medianFilter(0, counts * (0.256f / 32768.0f));
    if (counts > 32000 || counts < -32000) v = NAN;    // over range (q > ~60 kPa)
    if (!isnan(v)) vDiff = isnan(vDiff) ? v : vDiff + Q_FILTER * (v - vDiff);
  } else {
    float v = medianFilter(adsKind, counts * (6.144f / 32768.0f));
    if (adsKind == 1) vPt = isnan(vPt) ? v : vPt + P_FILTER * (v - vPt);
    else              vPs = isnan(vPs) ? v : vPs + P_FILTER * (v - vPs);
  }
}

void computeFlow() {
  if (!adsOk || isnan(vPt) || isnan(vPs) || isnan(vDiff) || !voltsOk(vPt) || !voltsOk(vPs)) {
    ptPa = psPa = qPa = velocity = mach = NAN;
    return;
  }
  float qUncorr = vDiff * PA_PER_VOLT;

  // auto-zero: valve closed and rotor stopped for a while = no flow, so slowly pull
  // q, Pt and Ps back to zero. Cancels warm-up drift without you pressing pzero.
  bool noFlow = (mode == M_IDLE && rpmF < 1);
  if (!noFlow) idleSinceMs = millis();
  if (autoZero && noFlow && millis() - idleSinceMs > AUTOZERO_AFTER_MS) {
    qZeroPa  += 0.05 * (qUncorr - qZeroPa);
    ptZeroPa += 0.05 * (voltsToPa(vPt, PT_MAX_BAR) - ptZeroPa);
    psZeroPa += 0.05 * (voltsToPa(vPs, PS_MAX_BAR) - psZeroPa);
  }

  ptPa = voltsToPa(vPt, PT_MAX_BAR) - ptZeroPa;
  psPa = voltsToPa(vPs, PS_MAX_BAR) - psZeroPa;
  qPa  = qUncorr - qZeroPa;                         // signed, so you can see the noise floor
  float qUse = (qPa < qDeadbandPa) ? 0 : qPa;

  float psA = psPa + P_ATM_PA;
  float ratio = (psA + qUse) / psA;
  mach = sqrt((2.0 / (GAMMA - 1.0)) * (pow(ratio, (GAMMA - 1.0) / GAMMA) - 1.0));

  float t0C = (useTcForVelocity && !isnan(tcTemp)) ? tcTemp : tTotalFixedC;
  float Ts = (t0C + 273.15) / (1.0 + 0.5 * (GAMMA - 1.0) * mach * mach);
  velocity = mach * sqrt(GAMMA * R_AIR * Ts);
}

void zeroPressure() {
  if (!adsOk) { OUT.println(F(">> ADS1115 not found, can't zero")); return; }
  if (mode != M_IDLE) { OUT.println(F(">> close the air valve first (type 0), then pzero")); return; }
  OUT.println(F(">> zeroing pressure sensors - no flow please..."));
  adsBusy = false;
  long sd = 0, s0 = 0, s1 = 0;
  ads.setGain(GAIN_SIXTEEN);
  for (byte i = 0; i < 40; i++) sd += ads.readADC_Differential_0_1();
  ads.setGain(GAIN_TWOTHIRDS);
  for (byte i = 0; i < 20; i++) { s0 += ads.readADC_SingleEnded(0); s1 += ads.readADC_SingleEnded(1); }
  float vd = (sd / 40.0) * (0.256 / 32768.0);
  float v0 = (s0 / 20.0) * (6.144 / 32768.0);
  float v1 = (s1 / 20.0) * (6.144 / 32768.0);
  vDiff = vd; vPt = v0; vPs = v1;
  qZeroPa  = vd * PA_PER_VOLT;
  ptZeroPa = voltsToPa(v0, PT_MAX_BAR);
  psZeroPa = voltsToPa(v1, PS_MAX_BAR);
  idleSinceMs = millis();
  OUT.print(F(">> zero done: Pt ")); OUT.print(v0, 3);
  OUT.print(F(" V, Ps ")); OUT.print(v1, 3);
  OUT.print(F(" V, sensor offset ")); OUT.print(qZeroPa, 0); OUT.println(F(" Pa"));
}

// ================= CONTROL LOOP =================
void controlStep() {
  const float dt = CONTROL_MS / 1000.0;

  float rpm = measureRPM();
  rpmF += 0.4 * (rpm - rpmF);

  if (rpmF > OVERSPEED_RPM && mode != M_IDLE) { fault(F("overspeed")); }
  if ((mode == M_SEEK || mode == M_RELAY) && millis() - tuneStartMs > TUNE_TIMEOUT_MS) {
    fault(F("autotune timed out"));
  }

  switch (mode) {
    case M_HOLD:  holdStep(dt); break;
    case M_SEEK:  seekStep(dt); break;
    case M_RELAY: relayStep();  break;
    default: break;
  }
  lastRpm = rpmF;

  pumpStep(dt);
  glowStep();
  seqStepFn();
  linkStep();
}

// ================= SERIAL =================
// separate line buffers for USB (0) and the ESP32 (1), so their commands never mix
char inBuf[2][24];
byte inLen[2] = {0, 0};
unsigned long inLastMs[2] = {0, 0};

bool validSp(float sp) {
  if (sp <= 0) return false;
  if (sp >= OVERSPEED_RPM * 0.9) {
    OUT.println(F(">> too close to overspeed limit, ignored"));
    return false;
  }
  return true;
}

void printHelp() {
  OUT.println(F(">> commands:"));
  OUT.println(F(">>   3000 | 0 | tune 3000 | table | clear | sched on|off | kp/ki/kd x"));
  OUT.println(F(">>   pump 40 | pump 0 | 7on | 7off | 8on | 8off | glowon | glowoff"));
  OUT.println(F(">>   start (4000 RPM start sequence) | go (confirm glow) | abort"));
  OUT.println(F(">>   pzero | autozero on|off | qdb 8 | t 30 | t tc | stop (everything off)"));
}

void handleCommand(char *raw) {
  if (raw[0] == 'h' && raw[1] == 'b' && raw[2] == 0) {   // heartbeat: silent and fast
    lastHbMs = millis();
    hbSeen = true;
    if (linkLost) { linkLost = false; OUT.println(F(">> link to dashboard restored")); }
    return;
  }
  // lowercase; also build a no-space copy for 7on / 8 off style commands
  char cmd[24], ns[24]; byte n = 0, m = 0;
  for (char *p = raw; *p && n < sizeof(cmd) - 1; p++) {
    char c = tolower(*p);
    cmd[n++] = c;
    if (c != ' ' && m < sizeof(ns) - 1) ns[m++] = c;
  }
  cmd[n] = 0; ns[m] = 0;

  if      (!strcmp(ns, "stop"))  allOff();
  else if (!strcmp(ns, "7on"))   setRelay(7, true);
  else if (!strcmp(ns, "7off"))  setRelay(7, false);
  else if (!strcmp(ns, "8on"))   setRelay(8, true);
  else if (!strcmp(ns, "8off"))  setRelay(8, false);
  else if (!strcmp(ns, "glowon"))  setGlow(true);
  else if (!strcmp(ns, "glowoff")) setGlow(false);
  else if (!strcmp(ns, "start"))   seqStart();
  else if (!strcmp(ns, "go"))      seqGo();
  else if (!strcmp(ns, "abort"))   seqAbort(F("cancelled by operator"));
  else if (!strncmp(ns, "pump", 4)) {
    if (!strcmp(ns + 4, "stop") || !strcmp(ns + 4, "off")) pumpTarget = 0;
    else pumpTarget = constrain(atof(ns + 4), 0, PUMP_MAX_PCT);
    if (pumpTarget == 0) { pumpPct = 0; writePump(0); }   // stop instantly
    OUT.print(F(">> pump target ")); OUT.print(pumpTarget, 0); OUT.println(F("%"));
  }
  else if (!strcmp(ns, "pzero")) zeroPressure();
  else if (!strcmp(ns, "autozeroon"))  { autoZero = true;  OUT.println(F(">> auto-zero ON (while air valve closed and RPM 0)")); }
  else if (!strcmp(ns, "autozerooff")) { autoZero = false; OUT.println(F(">> auto-zero OFF")); }
  else if (!strncmp(ns, "qdb", 3) && (isdigit(ns[3]) || ns[3] == '.')) {
    qDeadbandPa = max(0.0f, (float)atof(ns + 3));
    OUT.print(F(">> velocity deadband = ")); OUT.print(qDeadbandPa, 0); OUT.println(F(" Pa"));
  }
  else if (!strcmp(ns, "ttc"))   { useTcForVelocity = true; OUT.println(F(">> velocity uses thermocouple temperature")); }
  else if (cmd[0] == 't' && cmd[1] == ' ' && isdigit(cmd[2])) {
    tTotalFixedC = atof(cmd + 2); useTcForVelocity = false;
    OUT.print(F(">> velocity uses fixed total temp ")); OUT.print(tTotalFixedC, 1); OUT.println(F(" C"));
  }
  else if (!strncmp(cmd, "tune ", 5)) { float sp = atof(cmd + 5); if (validSp(sp)) startTune(sp); }
  else if (!strcmp(cmd, "table"))     printTable();
  else if (!strcmp(cmd, "clear"))     { nSched = 0; saveSched(); OUT.println(F(">> stored gains cleared")); }
  else if (!strcmp(cmd, "sched on"))  { useSched = true;  OUT.println(F(">> using stored gains")); }
  else if (!strcmp(cmd, "sched off")) { useSched = false; OUT.println(F(">> using manual gains")); }
  else if (!strncmp(cmd, "kp ", 3))   { Kp = atof(cmd + 3); OUT.print(F(">> manual Kp = ")); OUT.println(Kp, 5); }
  else if (!strncmp(cmd, "ki ", 3))   { Ki = atof(cmd + 3); OUT.print(F(">> manual Ki = ")); OUT.println(Ki, 5); }
  else if (!strncmp(cmd, "kd ", 3))   { Kd = atof(cmd + 3); OUT.print(F(">> manual Kd = ")); OUT.println(Kd, 5); }
  else if (isdigit(cmd[0])) {
    float sp = atof(cmd);
    if (sp <= 0) { closeValve(); OUT.println(F(">> air valve closed")); }
    else if (validSp(sp)) {
      startHold(sp);
      OUT.print(F(">> holding ")); OUT.print(sp, 0); OUT.println(F(" RPM"));
    }
  }
  else printHelp();
}

// plain byte argument (no custom types) so the Arduino IDE's auto-prototypes compile
void readPort(byte p) {
  Stream &port = Serial;
  while (port.available()) {
    char c = port.read();
    inLastMs[p] = millis();
    if (c == '\n' || c == '\r') {
      if (inLen[p]) { inBuf[p][inLen[p]] = 0; handleCommand(inBuf[p]); inLen[p] = 0; }
    } else if (inLen[p] < sizeof(inBuf[p]) - 1) {
      inBuf[p][inLen[p]++] = c;
    }
  }
  if (inLen[p] && millis() - inLastMs[p] > 100) {   // works with "No line ending"
    inBuf[p][inLen[p]] = 0; handleCommand(inBuf[p]); inLen[p] = 0;
  }
}

void readSerial() {
  readPort(0);
}

// one line, label:value pairs - works in Serial Plotter and the dashboard
void printVal(const __FlashStringHelper *label, float v, byte dp) {
  OUT.print(label);
  if (isnan(v)) OUT.print(F("nan")); else OUT.print(v, dp);
}

void printStatus() {
  float sp = 0;
  if (mode == M_HOLD) sp = spRamped;
  else if (mode == M_SEEK || mode == M_RELAY) sp = tuneSp;
  printVal(F("SP:"),     sp, 0);
  printVal(F(",RPM:"),   rpmF, 0);
  printVal(F(",Open:"),  opening, 1);
  printVal(F(",T:"),     tcTemp, 1);
  printVal(F(",Pt:"),    ptPa, 0);
  printVal(F(",Ps:"),    psPa, 0);
  printVal(F(",q:"),     qPa, 0);
  printVal(F(",V:"),     velocity, 1);
  printVal(F(",M:"),     mach, 3);
  printVal(F(",Pump:"),  pumpPct, 1);
  printVal(F(",R7:"),    relay7 ? 1 : 0, 0);
  printVal(F(",R8:"),    relay8 ? 1 : 0, 0);
  printVal(F(",G:"),     glowOn ? 1 : 0, 0);
  printVal(F(",Gs:"),    glowOn ? (millis() - glowStartMs) / 1000 : 0, 0);
  printVal(F(",Seq:"),   seqStep, 0);
  printVal(F(",SeqS:"),  seqStep ? (millis() - seqStepMs) / 1000 : 0, 0);
  printVal(F(",Stut:"),  seqStep == 4 ? stutterCount : 0, 0);
  printVal(F(",Mode:"),  mode, 0);                       // 0 idle, 1 hold, 2 tune-seek, 3 tune-relay
  printVal(F(",Cyc:"),   mode == M_RELAY ? cycleCount : 0, 0);
  printVal(F(",CycN:"),  TUNE_SKIP_CYCLES + TUNE_MEAS_CYCLES, 0);
  printVal(F(",F:"),     faultActive ? 1 : 0, 0);
  OUT.println();
}

// ================= MAIN =================
void setup() {
  // relays OFF before they become outputs, so nothing clicks at boot
  digitalWrite(RELAY7_PIN, RELAY_OFF);
  digitalWrite(RELAY8_PIN, RELAY_OFF);
  digitalWrite(GLOW_PIN, RELAY_OFF);
  pinMode(RELAY7_PIN, OUTPUT);
  pinMode(RELAY8_PIN, OUTPUT);
  pinMode(GLOW_PIN, OUTPUT);

  Serial.begin(115200);

  esc.attach(ESC_PIN, ESC_MIN_US, ESC_MAX_US);
  writePump(0);                        // zero throttle so the ESC arms

  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);
  valve.attach(SERVO_PIN);
  closeValve();
  loadSched();

  tcOk = tc.begin();
  if (tcOk) {
    tc.setThermocoupleType(MAX31856_TCTYPE_K);
    tc.setNoiseFilter(MAX31856_NOISE_FILTER_50HZ);
    tc.setConversionMode(MAX31856_CONTINUOUS);
  }

  adsOk = ads.begin(0x48);
  if (adsOk) {
    ads.setGain(GAIN_TWOTHIRDS);       // +/-6.144 V
    ads.setDataRate(RATE_ADS1115_250SPS);   // quieter than 860, still ~190 q samples/s
  }

  OUT.println(F(">> Test rig ready"));
  OUT.println(tcOk  ? F(">> MAX31856 OK") : F(">> MAX31856 NOT FOUND - check SPI wiring"));
  OUT.println(adsOk ? F(">> ADS1115 OK")  : F(">> ADS1115 NOT FOUND - check SDA/SCL"));
  printHelp();
  printTable();

  delay(500);                          // let sensors settle
  if (adsOk) zeroPressure();           // no flow at power-up
}

void loop() {
  readSerial();
  adsStep();

  static unsigned long lastControl = 0;
  if (millis() - lastControl >= CONTROL_MS) {
    lastControl += CONTROL_MS;
    controlStep();
  }

  static unsigned long lastTc = 0;
  if (millis() - lastTc >= 100) { lastTc = millis(); tcStep(); }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= PRINT_MS) {
    lastPrint = millis();
    computeFlow();
    printStatus();
  }
}
