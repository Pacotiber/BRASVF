// ESP32 <-> PCA9685 bridge for the arm_app.py application (folder 07_Application).
// The application computes everything (kinematics, calibration, map) and sends pulse widths in
// microseconds. This program makes the motions smooth and checks every I2C write.
//
// Commands (115200 baud, one per line):
//   ?                          -> status, e.g. "BRIDGE 1 pca=ok err=0 v=900 a=4000 pos 1500 1500 0 0 0"
//   T u0 u1 u2 u3 u4           -> new targets, followed with limited speed and acceleration (0 = unchanged).
//                                 All channels move together and arrive at the same time: the arm moves in
//                                 a straight line (in joint space) from its current position to the target,
//                                 which lets the application check the path before sending it.
//   M duration u0 u1 u2 u3 u4  -> synchronised move: all channels arrive together after duration (ms)
//   P c u                      -> channel c directly to u microseconds (no ramp)
//   V speed acceleration       -> limits of the follower, in us/s and us/s2
//   O                          -> release all servos (no more pulses: they become free)
//   O c                        -> release channel c
// Messages sent: "pos u0 .. u4" (10 times per second during a motion and after each
//                T, M, P, O command; 0 = released),
//                "done" (end of an M move), "ok", "err ...".
// At start-up: after a power-up, all servos are released (the application activates them). After a
// simple reboot of the ESP32 (EN button, upload), the PCA9685 still outputs the last pulses: they are
// read back and kept, so the arm stays held instead of falling.

#include <Wire.h>

const uint8_t PCA_ADDRESS = 0x40;
const int     PIN_SDA = 21, PIN_SCL = 22;
const float   PCA_OSC = 25000000.0f;   // internal oscillator of the PCA9685
const uint8_t PRESCALE = 121;          // 25 MHz / (4096 x 122) = 50.03 Hz
const int     N = 5;                   // channels 0 to 4: base, shoulder, elbow, wrist, gripper
const float   US_MIN = 400, US_MAX = 2600;   // absolute safety clamp
const unsigned long PERIOD_MS = 20;    // one update per servo pulse

float pcaFreq = PCA_OSC / (4096.0f * (PRESCALE + 1));
bool  pcaOk = false;
unsigned long errorCount = 0;

float current[N], target[N], startPos[N];
bool  active[N];
uint16_t written[N];                    // last value written to the PCA9685 (0xFFFF = unknown)
bool  syncMove = false;                 // M move in progress
unsigned long tSync = 0, syncDuration = 1;
float vMax = 900.0f;                    // us/s for the channel with the longest way to go (about 80 deg/s)
float aMax = 4000.0f;                   // us/s2
float vFollow = 0;                      // current speed of the follower (us/s)
bool  moving = false;
bool  changed = false;                  // command received: the position is sent at the next report

// ------------------------------------------------------------------ PCA9685
bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(PCA_ADDRESS);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

int readReg(uint8_t reg) {
  Wire.beginTransmission(PCA_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return -1;
  if (Wire.requestFrom(PCA_ADDRESS, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

bool pcaInit() {
  Wire.beginTransmission(PCA_ADDRESS);
  if (Wire.endTransmission() != 0) return false;
  bool ok = writeReg(0x00, 0x10);       // sleep (required to change the frequency)
  ok &= writeReg(0xFE, PRESCALE);
  ok &= writeReg(0x00, 0x00);           // wake up
  delay(1);
  ok &= writeReg(0x00, 0xA0);           // restart + auto-increment
  ok &= writeReg(0x01, 0x04);           // push-pull outputs
  for (int c = 0; c < N; c++) written[c] = 0xFFFF;
  return ok && readReg(0xFE) == PRESCALE;
}

// writes a channel then reads it back to check it (up to 3 tries). released = no more pulses.
// After 3 failures nothing is written until the next check (checkPca, every second): the PCA9685
// keeps generating the last pulses it received, so the arm stays still.
bool writeChannel(int c, float us, bool released) {
  if (!pcaOk) return false;
  uint16_t off = released ? 0x1000 : (uint16_t)lroundf(us * 4096.0f * pcaFreq / 1000000.0f);
  if (off == written[c]) return true;
  for (int attempt = 0; attempt < 3; attempt++) {
    Wire.beginTransmission(PCA_ADDRESS);
    Wire.write(0x06 + 4 * c);           // LEDc_ON_L, ON_H, OFF_L, OFF_H (auto-increment)
    Wire.write(0);
    Wire.write(0);
    Wire.write(off & 0xFF);
    Wire.write(off >> 8);
    if (Wire.endTransmission() == 0) {     // read back the 4 registers of the channel
      Wire.beginTransmission(PCA_ADDRESS);
      Wire.write(0x06 + 4 * c);
      if (Wire.endTransmission() == 0 && Wire.requestFrom(PCA_ADDRESS, (uint8_t)4) == 4) {
        uint8_t onL = Wire.read(), onH = Wire.read(), offL = Wire.read(), offH = Wire.read();
        if (onL == 0 && onH == 0 && offL == (off & 0xFF) && offH == (off >> 8)) { written[c] = off; return true; }
      }
    }
    errorCount++;
  }
  written[c] = 0xFFFF;                  // unknown value: try again after the check
  pcaOk = false;
  return false;
}

// at start-up: read back the pulses the PCA9685 is already sending (full-off = released servo)
void restoreOutputs() {
  for (int c = 0; c < N; c++) {
    int offL = readReg(0x08 + 4 * c), offH = readReg(0x09 + 4 * c);
    active[c] = false;
    if (offL < 0 || offH < 0 || (offH & 0x10)) continue;
    uint16_t off = offL | ((offH & 0x0F) << 8);
    float us = off * 1000000.0f / (4096.0f * pcaFreq);
    if (us < US_MIN || us > US_MAX) continue;
    active[c] = true;
    current[c] = target[c] = us;
    written[c] = off;
  }
}

// every second: if the PCA9685 was disturbed (I2C noise) or stopped answering, set it up again
void checkPca() {
  int mode1 = readReg(0x00), pre = readReg(0xFE);
  if (!pcaOk || mode1 < 0 || pre != PRESCALE || (mode1 & 0x10) || !(mode1 & 0x20)) {
    if (pcaOk) errorCount++;
    pcaOk = pcaInit();
  }
  for (int c = 0; c < N; c++) written[c] = 0xFFFF;    // refresh every channel at the next update
}

// ------------------------------------------------------------------ motion
float clampUs(float us) { return us < US_MIN ? US_MIN : (us > US_MAX ? US_MAX : us); }

void updateMotion() {
  float dt = PERIOD_MS / 1000.0f;
  moving = false;
  if (!pcaOk) {          // PCA9685 not reachable: motion paused (it will resume smoothly, without a jump)
    tSync += PERIOD_MS;
    vFollow = 0;
    return;
  }
  if (syncMove) {
    float t = (float)(millis() - tSync) / syncDuration;
    if (t > 1) t = 1;
    float s = t * t * (3 - 2 * t);      // smooth start and stop
    for (int c = 0; c < N; c++) {
      if (!active[c]) continue;
      float v = startPos[c] + s * (target[c] - startPos[c]);
      moving |= v != current[c];
      current[c] = v;
    }
    if (t >= 1) { syncMove = false; Serial.println("done"); }
  } else {
    // synchronised follower: E = remaining way of the channel that is most behind; every channel covers
    // the same fraction of its way, so they all arrive together (straight-line path)
    float E = 0;
    for (int c = 0; c < N; c++)
      if (active[c]) E = fmaxf(E, fabsf(target[c] - current[c]));
    if (E < 0.5f) {
      for (int c = 0; c < N; c++)
        if (active[c]) current[c] = target[c];
      vFollow = 0;
    } else {
      // wanted speed: vMax, but slow down early enough to stop exactly on the target
      float vWanted = fminf(vMax, sqrtf(2 * aMax * E));
      float dv = vWanted - vFollow, dvMax = aMax * dt;
      vFollow += dv > dvMax ? dvMax : (dv < -dvMax ? -dvMax : dv);
      float f = fminf(vFollow * dt, E) / E;     // fraction of the way covered during this update
      for (int c = 0; c < N; c++)
        if (active[c]) current[c] += f * (target[c] - current[c]);
      moving = true;
    }
  }
  for (int c = 0; c < N; c++) writeChannel(c, current[c], !active[c]);
}

void sendPos() {
  Serial.print("pos");
  for (int c = 0; c < N; c++) {
    Serial.print(' ');
    Serial.print(active[c] ? (int)lroundf(current[c]) : 0);
  }
  Serial.println();
}

// ------------------------------------------------------------------ commands
int readNumbers(const char* p, float* v, int nMax) {
  int n = 0;
  while (n < nMax) {
    char* end;
    float x = strtof(p, &end);
    if (end == p) break;
    v[n++] = x;
    p = end;
  }
  return n;
}

void activateChannel(int c, float us) {   // a released servo jumps straight to its target
  if (!active[c]) { active[c] = true; current[c] = us; }
}

void handleLine(char* line) {
  while (*line == ' ') line++;
  char cmd = toupper(*line);
  if (cmd == 0) return;
  float v[N + 1];
  int n = readNumbers(line + 1, v, N + 1);
  changed |= cmd == 'T' || cmd == 'M' || cmd == 'P' || cmd == 'O';
  if (cmd == '?') {
    Serial.printf("BRIDGE 1 pca=%s err=%lu v=%.0f a=%.0f ", pcaOk ? "ok" : "missing", errorCount, vMax, aMax);
    sendPos();
  } else if (cmd == 'T') {
    if (n != N) { Serial.println("err T expects 5 values"); return; }
    syncMove = false;
    for (int c = 0; c < N; c++)
      if (v[c] > 0) { target[c] = clampUs(v[c]); activateChannel(c, target[c]); }
  } else if (cmd == 'M') {
    if (n != N + 1 || v[0] < 0) { Serial.println("err M expects a duration and 5 values"); return; }
    for (int c = 0; c < N; c++) {
      if (v[c + 1] > 0) { target[c] = clampUs(v[c + 1]); activateChannel(c, target[c]); }
      startPos[c] = current[c];
    }
    vFollow = 0;
    syncDuration = v[0] < 1 ? 1 : (unsigned long)v[0];
    tSync = millis();
    syncMove = true;
    Serial.println("ok");
  } else if (cmd == 'P') {
    int c = (int)v[0];
    if (n != 2 || c < 0 || c >= N) { Serial.println("err P expects a channel (0-4) and a value"); return; }
    syncMove = false;
    active[c] = true;
    current[c] = target[c] = clampUs(v[1]);
    Serial.println("ok");
  } else if (cmd == 'V') {
    if (n >= 1 && v[0] > 0) vMax = v[0];
    if (n >= 2 && v[1] > 0) aMax = v[1];
    Serial.printf("ok v=%.0f a=%.0f\n", vMax, aMax);
  } else if (cmd == 'O') {
    syncMove = false;
    for (int c = 0; c < N; c++)
      if (n == 0 || (int)v[0] == c) active[c] = false;
    vFollow = 0;
    Serial.println("ok");
  } else {
    Serial.println("err unknown command");
  }
}

// ------------------------------------------------------------------ program
void setup() {
  Serial.begin(115200);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);                // slow bus: more tolerant of jumper wires
  Wire.setTimeOut(100);
  for (int c = 0; c < N; c++) { active[c] = false; current[c] = target[c] = 1500; written[c] = 0xFFFF; }
  int mode1 = readReg(0x00), pre = readReg(0xFE);
  bool alreadySet = mode1 >= 0 && pre == PRESCALE && !(mode1 & 0x10) && (mode1 & 0x20);
  pcaOk = alreadySet ? true : pcaInit();   // already set = simple reboot of the ESP32
  if (pcaOk) restoreOutputs();
  Serial.println(pcaOk ? "BRIDGE ready" : "BRIDGE error: PCA9685 not found (SDA=21, SCL=22, VCC, GND)");
  sendPos();
}

void loop() {
  static char line[96];
  static uint8_t n = 0;
  static unsigned long tUpdate = 0, tPos = 0, tCheck = 0;
  static bool hasMoved = false;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (n > 0) { line[n] = 0; handleLine(line); n = 0; }
    } else if (n < sizeof(line) - 1) {
      line[n++] = c;
    }
  }
  unsigned long t = millis();
  if (t - tUpdate >= PERIOD_MS) {
    tUpdate = t;
    updateMotion();
    hasMoved |= moving;
  }
  if (t - tPos >= 100) {                // position 10 times per second during a motion
    tPos = t;
    if (hasMoved || changed) sendPos();
    hasMoved = moving;
    changed = false;
  }
  if (t - tCheck >= 1000) {
    tCheck = t;
    checkPca();
  }
}
