// Test: puts one servo (one PCA9685 channel) at a given angle and holds it.
// Same wiring as the other programs: ESP32 SDA=21, SCL=22, PCA9685 at 0x40,
// MG995 with 500 us = 0 deg and 2500 us = 180 deg. Serial monitor at 115200 baud.
// Warning: the PCA9685 keeps sending the pulse after this program, as long as it is powered.

#include <Wire.h>

const uint8_t CHANNEL = 0;       // channel to test: 0 base, 1 shoulder, 2 elbow, 3 wrist, 4 gripper
const float   ANGLE   = 90.0;    // wanted servo angle (deg)
const float   US0     = 500.0f;  // pulse width for 0 deg
const float   US180   = 2500.0f; // pulse width for 180 deg

// ======================= PCA9685 (driven over I2C, no library) =======================
const uint8_t PCA_ADDRESS = 0x40;
const int     PIN_SDA = 21, PIN_SCL = 22;
const float   PCA_OSC = 25000000.0f;   // internal oscillator; adjust (23-27 MHz) if the angle is offset
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
  Wire.setClock(100000);
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

void setup() {
  Serial.begin(115200);
  delay(500);
  if (!pcaInit()) {
    Serial.println("ERROR: PCA9685 not found at address 0x40. Check SDA=21, SCL=22, VCC and GND.");
    while (true) delay(1000);
  }
  float us = US0 + constrain(ANGLE, 0.0f, 180.0f) * (US180 - US0) / 180.0f;
  pcaPulse(CHANNEL, us);   // the PCA9685 keeps sending the pulse: the servo stays held
  Serial.printf("Channel %d at %.1f deg (pulse %.0f us, %.2f Hz)\n", CHANNEL, ANGLE, us, pcaFreq);
}

void loop() {}
