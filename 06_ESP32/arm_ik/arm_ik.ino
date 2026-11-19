// Robot arm: inverse kinematics on the ESP32 + PCA9685 (standalone, without the PC application).
// Type a position in mm in the serial monitor (115200 baud, line ending "New Line"),
// the arm computes the angles with the MATLAB model and moves there smoothly.
//
// Commands:
//   200 150 80        -> go to X=200, Y=150 (height), Z=80 (mm), gripper angle chosen automatically
//                        (the solution furthest from the limits)
//   200 150 80 -90    -> same, with the gripper tilted at -90 deg (pointing down)
//   a 0 90 -90 0      -> model angles in degrees: base, shoulder, elbow, wrist
//   s 2 95            -> sends 95 deg raw to servo 2 (calibration; servo 4 = gripper)
//   o / c             -> open / close the gripper
//   rest (or h, home, default) -> back to the default position (REST)
//   v 40              -> maximum speed in deg/s
//   ?                 -> help
//
// Note: this standalone program has its own calibration (SERVOS table below). The PC application
// (07_Application) uses the arm_bridge firmware instead and its own calibration tab.

#include <Wire.h>
#include <strings.h>   // strncasecmp
#include "kinematics.h"

// ======================= 1. Arm dimensions (mm, measured in the CAD model) =======================
const Arm ARM = {
  153.86f,   // L1: shoulder axis -> elbow axis
  120.39f,   // L2: elbow axis -> wrist axis
  124.86f,   // L3: wrist axis -> fingertips, along the gripper ("gripper v1")
  1.50f,     // HP: fingertips offset perpendicular to the gripper axis
  10.25f,    // R0: shoulder axis in front of the base axis
  97.4f,     // H0: height of the shoulder axis above the floor (base V2: CAD value, measure it with a ruler)
  21.75f     // C : total sideways offset (3 x 7.25)
};
const float FLOOR_HEIGHT = 0.0f;   // no point of the arm may go below this height (mm)

// ======================= 2. Servo calibration (TO BE SET on the real arm) =======================
// servo angle = zero + dir x model angle
//   zero : angle sent to the servo when the joint is at 0 in the model
//          (base pointing forward, arm 1 horizontal, arm 2 in line with arm 1,
//           gripper in line with arm 2)
//   dir  : +1 or -1 depending on whether the servo turns in the positive direction of the model
//   us0 / us180 : pulse width (microseconds) that gives 0 and 180 deg on THIS servo
//   smin / smax : allowed servo angle range (mechanical limits)
struct ServoCfg {
  uint8_t channel;  // PCA9685 output
  float   us0, us180;
  float   zero;
  int8_t  dir;
  float   smin, smax;
};

ServoCfg SERVOS[4] = {
  // channel us0   us180  zero  dir  smin  smax
  {  0,      500,  2500,  90,   +1,  0,    180 },  // base
  {  1,      500,  2500,  90,   +1,  0,    180 },  // shoulder
  {  2,      500,  2500,  90,   +1,  0,    180 },  // elbow
  {  3,      500,  2500,  90,   +1,  0,    180 },  // wrist
};
ServoCfg GRIPPER = { 4, 500, 2500, 0, +1, 0, 180 };   // SG90 of the gripper (direct servo angle)
const float GRIPPER_OPEN   = 30.0f;                   // servo angle with the gripper open (to be set)
const float GRIPPER_CLOSED = 180.0f;                  // servo angle with the gripper closed (to be set)

// model limits on top of the servo limits (deg): base, shoulder, elbow, wrist
const float Q_MIN[4] = { -90, -10, -150, -120 };
const float Q_MAX[4] = {  90, 170,  150,  120 };

const JointAngles REST = { 0, 180, -90, -90 };   // rest position (arm folded, gripper flat), far from the limits

// ======================= 3. PCA9685 (driven over I2C, no library) =======================
const uint8_t PCA_ADDRESS = 0x40;
const int     PIN_SDA = 21, PIN_SCL = 22;
const float   PCA_OSC = 25000000.0f;   // internal oscillator; adjust (23-27 MHz) if the angles are offset
const float   PCA_FREQ_WANTED = 50.0f; // 50 Hz for servos
float pcaFreq = 50.0f;                 // frequency actually obtained

void pcaWrite(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(PCA_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

bool pcaInit() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);   // slow bus: more tolerant of jumper wires
  Wire.setTimeOut(100);
  Wire.beginTransmission(PCA_ADDRESS);
  if (Wire.endTransmission() != 0) return false;            // PCA9685 missing or badly wired
  uint8_t prescale = (uint8_t)lroundf(PCA_OSC / (4096.0f * PCA_FREQ_WANTED)) - 1;
  pcaFreq = PCA_OSC / (4096.0f * (prescale + 1));
  pcaWrite(0x00, 0x10);        // MODE1: sleep (required to change the frequency)
  pcaWrite(0xFE, prescale);    // PRESCALE
  pcaWrite(0x00, 0x00);        // wake up
  delay(1);
  pcaWrite(0x00, 0xA0);        // restart + register auto-increment
  pcaWrite(0x01, 0x04);        // MODE2: push-pull outputs
  return true;
}

void pcaPulse(uint8_t channel, float us) {
  uint16_t off = (uint16_t)lroundf(us * 4096.0f * pcaFreq / 1000000.0f);
  if (off > 4095) off = 4095;
  Wire.beginTransmission(PCA_ADDRESS);
  Wire.write(0x06 + 4 * channel);  // LEDn_ON_L, then ON_H, OFF_L, OFF_H (auto-increment)
  Wire.write(0);
  Wire.write(0);
  Wire.write(off & 0xFF);
  Wire.write(off >> 8);
  Wire.endTransmission();
}

void servoAngle(const ServoCfg& s, float servoDeg) {
  servoDeg = constrain(servoDeg, s.smin, s.smax);
  pcaPulse(s.channel, s.us0 + servoDeg * (s.us180 - s.us0) / 180.0f);
}

// ======================= 4. Model <-> servos =======================
float toServo(int i, float qDeg) { return SERVOS[i].zero + SERVOS[i].dir * qDeg; }

float getQ(const JointAngles& q, int i) {
  return i == 0 ? q.base : i == 1 ? q.shoulder : i == 2 ? q.elbow : q.wrist;
}

void setQ(JointAngles& q, int i, float v) {
  if (i == 0) q.base = v; else if (i == 1) q.shoulder = v; else if (i == 2) q.elbow = v; else q.wrist = v;
}

// margin (deg) before the nearest limit, negative if an angle is outside the limits.
// The base is not part of the margin: its angle is imposed by the target, only its validity matters.
float limitMargin(const JointAngles& q) {
  float margin = 1000.0f;
  for (int i = 0; i < 4; i++) {
    float qi = getQ(q, i), s = toServo(i, qi);
    float m = fminf(fminf(qi - Q_MIN[i], Q_MAX[i] - qi), fminf(s - SERVOS[i].smin, SERVOS[i].smax - s));
    if (m < 0) return m;
    if (i > 0) margin = fminf(margin, m);
  }
  return margin;
}

void apply(const JointAngles& q) {
  for (int i = 0; i < 4; i++) servoAngle(SERVOS[i], toServo(i, getQ(q, i)));
}

// ======================= 5. Smooth motion =======================
// interpolation in joint space, with a smooth start and stop
JointAngles qCurrent = REST, qStart = REST, qTarget = REST;
unsigned long tStart = 0, durationMs = 0;
float maxSpeed = 45.0f;   // deg/s for the joint that moves the most
bool inMotion = false;

void moveTo(const JointAngles& target) {
  qStart = qCurrent;
  qTarget = target;
  float maxGap = 0;
  for (int i = 0; i < 4; i++) maxGap = fmaxf(maxGap, fabsf(getQ(target, i) - getQ(qStart, i)));
  durationMs = (unsigned long)(1000.0f * maxGap / maxSpeed) + 1;
  tStart = millis();
  inMotion = true;
}

void updateMotion() {
  static unsigned long last = 0;
  if (!inMotion || millis() - last < 20) return;   // one update every 20 ms
  last = millis();
  float t = fminf(1.0f, (float)(millis() - tStart) / durationMs);
  float s = t * t * (3.0f - 2.0f * t);             // smooth start and stop
  for (int i = 0; i < 4; i++)
    setQ(qCurrent, i, getQ(qStart, i) + s * (getQ(qTarget, i) - getQ(qStart, i)));
  apply(qCurrent);
  if (t >= 1.0f) {
    inMotion = false;
    Serial.println("Arrived.");
  }
}

// ======================= 6. Serial monitor commands =======================
void printAngles(const char* title, const JointAngles& q) {
  Serial.printf("%s base %.1f | shoulder %.1f | elbow %.1f | wrist %.1f (deg)\n",
                title, q.base, q.shoulder, q.elbow, q.wrist);
  Serial.printf("  servos: %.1f  %.1f  %.1f  %.1f\n",
                toServo(0, q.base), toServo(1, q.shoulder), toServo(2, q.elbow), toServo(3, q.wrist));
}

void help() {
  Serial.println("Commands:");
  Serial.println("  X Y Z         go to (X, Y, Z) mm, automatic gripper angle");
  Serial.println("  X Y Z phi     same with a fixed gripper angle (deg, -90 = pointing down)");
  Serial.println("  a b s e w     model angles (deg): base shoulder elbow wrist");
  Serial.println("  s n angle     raw angle on servo n (0 to 3, 4 = gripper) for calibration");
  Serial.println("  o / c         open / close the gripper");
  Serial.println("  rest          back to the default position (also: h, home, default)");
  Serial.println("  v deg_per_s   maximum speed");
}

int readNumbers(char* text, float* values, int nMax) {   // reads up to nMax numbers separated by spaces
  int n = 0;
  char* p = text;
  while (n < nMax) {
    char* end;
    float v = strtof(p, &end);
    if (end == p) break;
    values[n++] = v;
    p = end;
  }
  return n;
}

void positionCommand(float* v, int n) {
  if (n < 3) { Serial.println("At least X Y Z are needed."); return; }
  JointAngles q;
  float phi;
  unsigned long t0 = micros();
  bool ok = ik(ARM, v[0], v[1], v[2], n >= 4, n >= 4 ? v[3] : 0, FLOOR_HEIGHT, limitMargin, q, phi);
  unsigned long dt = micros() - t0;
  if (!ok) {
    Serial.printf("Cannot reach (%.1f ; %.1f ; %.1f): out of reach, outside the limits or below the floor.\n",
                  v[0], v[1], v[2]);
    return;
  }
  float X, Y, Z, hMin;
  fk(ARM, q, X, Y, Z, hMin);   // check: compute the position again with forward kinematics
  Serial.printf("Target (%.1f ; %.1f ; %.1f) mm, gripper at %.0f deg, computed in %lu us\n", v[0], v[1], v[2], phi, dt);
  printAngles("  angles:", q);
  Serial.printf("  check: (%.2f ; %.2f ; %.2f) mm, error %.3f mm\n",
                X, Y, Z, sqrtf(sq(X - v[0]) + sq(Y - v[1]) + sq(Z - v[2])));
  moveTo(q);
}

void anglesCommand(float* v, int n) {
  if (n < 4) { Serial.println("4 angles are needed: base shoulder elbow wrist."); return; }
  JointAngles q = { v[0], v[1], v[2], v[3] };
  if (limitMargin(q) < 0) { Serial.println("Angles outside the limits."); return; }
  float X, Y, Z, hMin;
  fk(ARM, q, X, Y, Z, hMin);
  if (hMin < FLOOR_HEIGHT) { Serial.println("The arm would go below the floor."); return; }
  printAngles("Angles:", q);
  Serial.printf("  fingertips at (%.1f ; %.1f ; %.1f) mm\n", X, Y, Z);
  moveTo(q);
}

void rawServoCommand(float* v, int n) {
  if (n < 2 || v[0] < 0 || v[0] > 4) { Serial.println("Usage: s number(0-4) angle"); return; }
  int i = (int)v[0];
  if (i == 4) { servoAngle(GRIPPER, v[1]); Serial.printf("Gripper: %.1f deg\n", v[1]); return; }
  inMotion = false;
  servoAngle(SERVOS[i], v[1]);
  setQ(qCurrent, i, (v[1] - SERVOS[i].zero) / SERVOS[i].dir);   // keeps the model up to date
  Serial.printf("Servo %d: %.1f deg raw (%.1f deg in the model)\n", i, v[1], getQ(qCurrent, i));
}

// true if the line is exactly this word (any case), possibly followed by spaces
bool isWord(const char* line, const char* word) {
  size_t n = strlen(word);
  if (strncasecmp(line, word, n) != 0) return false;
  for (const char* p = line + n; *p; p++) if (*p != ' ') return false;
  return true;
}

void goRest() {
  Serial.println("Back to the default position.");
  printAngles("Rest:", REST);
  moveTo(REST);
}

void handleLine(char* line) {
  while (*line == ' ') line++;
  if (*line == 0) return;
  if (isWord(line, "rest") || isWord(line, "h") || isWord(line, "home") || isWord(line, "default")) {
    goRest();
    return;
  }
  float v[5];
  char c = line[0];
  if (c == '?')      help();
  else if (c == 'o') { servoAngle(GRIPPER, GRIPPER_OPEN);   Serial.println("Gripper open."); }
  else if (c == 'c') { servoAngle(GRIPPER, GRIPPER_CLOSED); Serial.println("Gripper closed."); }
  else if (c == 'a') anglesCommand(v, readNumbers(line + 1, v, 4));
  else if (c == 's') rawServoCommand(v, readNumbers(line + 1, v, 2));
  else if (c == 'v') {
    if (readNumbers(line + 1, v, 1) == 1 && v[0] > 0) maxSpeed = v[0];
    Serial.printf("Max speed: %.0f deg/s\n", maxSpeed);
  }
  else positionCommand(v, readNumbers(line, v, 4));
}

// ======================= 7. Program =======================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nRobot arm - inverse kinematics (ESP32 + PCA9685)");
  if (!pcaInit()) {
    Serial.println("ERROR: PCA9685 not found at address 0x40. Check SDA=21, SCL=22, VCC and GND.");
    while (true) delay(1000);
  }
  Serial.printf("PCA9685 ready, frequency %.2f Hz\n", pcaFreq);
  // the MG995 have no position sensor: at start-up they jump straight to the rest position.
  // They are activated one by one to limit the current surge.
  for (int i = 0; i < 4; i++) {
    servoAngle(SERVOS[i], toServo(i, getQ(REST, i)));
    delay(300);
  }
  servoAngle(GRIPPER, GRIPPER_OPEN);
  help();
}

void loop() {
  static char line[64];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (n > 0) { line[n] = 0; handleLine(line); n = 0; }
    } else if (n < sizeof(line) - 1) {
      line[n++] = c;
    }
  }
  updateMotion();
}
