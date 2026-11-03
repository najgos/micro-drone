#pragma once
// DRONE – Seeed XIAO ESP32-C6 on the DR3 board.
// USB VBUS and the motor supply share the 5 V rail on this board.
#include "Imu.h"
#include "Attitude.h"

ImuSample sample{};
bool imuOK = false, haveSample = false, gyroCalibrated = false;
uint32_t lastPoll = 0, lastFresh = 0;

// Gyro bias: average of the first 200 samples after power-up (the drone must be still).
constexpr uint16_t GYRO_CAL_SAMPLES = 200;
float gyroBias[3]{};
double gyroSum[3]{};
uint16_t gyroCount = 0;

// Remote: the drone remembers the first sender of valid frames and accepts control only from it.
uint8_t peerMac[6]{};
bool peerKnown = false;
uint32_t peerSequence = 0, peerSeen = 0;

#include "Control.h"

void imuFault() {
  imuOK = false; haveSample = false;
  disarm(true);
  attitude.reset();
}

// Reads a new sample (the sensor provides one every 10 ms), calibrates and updates the attitude.
bool pollImu() {
  if (!imuOK || uint32_t(millis() - lastPoll) < 2) return false;
  lastPoll = millis();
  uint8_t status = 0;
  if (!readRegisters(0x3A, &status, 1)) { imuFault(); return false; }
  if (!(status & 1)) {
    if (uint32_t(millis() - lastFresh) > 50) imuFault();   // the sensor stopped producing samples
    return false;
  }
  if (!readSample(sample)) { imuFault(); return false; }
  lastFresh = millis();

  float a[3], g[3];
  for (int i = 0; i < 3; ++i) { a[i] = sample.a[i] / ACCEL_LSB_PER_G; g[i] = sample.g[i] / GYRO_LSB_PER_DPS; }
  if (!gyroCalibrated) {
    for (int i = 0; i < 3; ++i) gyroSum[i] += g[i];
    if (++gyroCount == GYRO_CAL_SAMPLES) {
      for (int i = 0; i < 3; ++i) gyroBias[i] = gyroSum[i] / GYRO_CAL_SAMPLES;
      gyroCalibrated = true;
      attitude.reset();
    }
  }
  for (int i = 0; i < 3; ++i) g[i] -= gyroBias[i];
  attitude.update(a, g, lastFresh);
  haveSample = true;
  return true;
}

// Receives control frames: correct format and project, known remote, newer sequence number.
void droneRadio() {
  RadioInbox item{};
  while (radioInbox && xQueueReceive(radioInbox, &item, 0) == pdTRUE) {
    const Frame &f = item.frame;
    if (f.magic != FRAME_MAGIC || memcmp(f.project, PROJECT_ID, 8) != 0) continue;
    if (peerKnown && !sameMac(item.source, peerMac)) continue;
    // After a link gap (e.g. remote restart) the numbering may start again.
    if (linkFresh() && !newerSequence(f.sequence, peerSequence)) continue;
    const ControlData &c = f.control;
    bool valid = c.calibrated <= 1 && c.arm <= 1 && c.kill <= 1;
    for (int16_t v : c.axes) if (v < -1000 || v > 1000) valid = false;
    if (!valid) continue;
    if (!peerKnown) { memcpy(peerMac, item.source, 6); peerKnown = true; }
    peerSequence = f.sequence;
    peerSeen = item.arrival;
    controlInput = c;
  }
}

// LED: fault – fast blink, armed – solid, ready to arm – slow blink, not ready – short flash every 2 s.
void updateLed(bool ready) {
  uint32_t now = millis();
  bool on;
  if (controlState == 3) on = (now / 100) % 2;
  else if (controlState == 2) on = true;
  else if (ready) on = (now / 500) % 2;
  else on = now % 2000 < 100;
  digitalWrite(DRONE_LED_PIN, on != DRONE_LED_ACTIVE_LOW ? HIGH : LOW);
}

void droneSetup() {
  for (uint8_t pin : MOTOR_PINS) { digitalWrite(pin, LOW); pinMode(pin, OUTPUT); }   // motors off from the first moment
  pinMode(DRONE_LED_PIN, OUTPUT);
  Serial.begin(115200);                          // needed to detect a USB connection
  digitalWrite(3, LOW); pinMode(3, OUTPUT);      // enable the XIAO RF switch
  digitalWrite(14, LOW); pinMode(14, OUTPUT);    // built-in antenna
  radioOK = startRadio();
  beginOutputs();
  if (Wire.begin(IMU_SDA, IMU_SCL, 100000)) { Wire.setTimeOut(10); imuOK = beginImu(); }
  lastFresh = millis();
}

void droneLoop() {
  droneRadio();
  bool newSample = pollImu();
  controlStep(imuOK && haveSample, newSample);
  updateLed(controlState == 1 && canOperate(imuOK && haveSample));
  delay(1);
}
