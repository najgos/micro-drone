#pragma once
// REMOTE – ESP32 Dev Module with two analog joysticks.
// The joysticks are powered from 3.3 V (the ESP32 ADC is not 5 V tolerant).
// Stick centres are measured during the first second after power-up.

uint16_t stickCenter[4]{};
uint32_t centerSum[4]{};
uint16_t centerCount = 0;
bool sticksCalibrated = false;
bool buttonDown[2] = {false, false};
uint32_t lastSample = 0, txSequence = 0;

uint16_t readStick(int i) {
  uint32_t total = 0;
  for (int k = 0; k < 8; ++k) total += analogRead(STICK_PINS[i]);   // average of 8 readings
  return total / 8;
}
// Position relative to the measured centre and the ends from config.h: -1000..1000, then the deadband.
int16_t normalizeAxis(int i, uint16_t raw) {
  int32_t delta = int32_t(raw) - stickCenter[i];
  int32_t span = delta >= 0 ? STICK_HIGH[i] - stickCenter[i] : stickCenter[i] - STICK_LOW[i];
  if (span <= 0) return 0;
  int32_t n = constrain(delta * 1000 / span, -1000L, 1000L);
  if (STICK_INVERT[i]) n = -n;
  float v = n * .001f, d = STICK_DEADBAND[i];
  v = fabsf(v) <= d ? 0 : copysignf((fabsf(v) - d) / (1 - d), v);
  return static_cast<int16_t>(v * 1000);
}
// Buttons: contact bounce filtered for 30 ms.
void updateButtons() {
  static bool candidate[2] = {false, false};
  static uint32_t changed[2]{};
  const uint8_t pins[2] = {ARM_PIN, KILL_PIN};
  for (int i = 0; i < 2; ++i) {
    bool raw = digitalRead(pins[i]) != 0;
    bool down = BUTTONS_ACTIVE_LOW ? !raw : raw;
    if (down != candidate[i]) { candidate[i] = down; changed[i] = millis(); }
    if (uint32_t(millis() - changed[i]) >= 30) buttonDown[i] = candidate[i];
  }
}
bool sendControl(const ControlData &control) {
  Frame frame{};
  frame.magic = FRAME_MAGIC;
  memcpy(frame.project, PROJECT_ID, 8);
  frame.sequence = ++txSequence;
  frame.control = control;
  return esp_now_send(DRONE_MAC, reinterpret_cast<const uint8_t *>(&frame), sizeof(frame)) == ESP_OK;
}

void remoteSetup() {
  analogReadResolution(12);
  for (uint8_t pin : STICK_PINS) { pinMode(pin, INPUT); analogSetPinAttenuation(pin, ADC_11db); }
  pinMode(ARM_PIN, BUTTONS_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
  pinMode(KILL_PIN, BUTTONS_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
  radioOK = startRadio() && addPeer(DRONE_MAC);
}

void remoteLoop() {
  updateButtons();
  if (uint32_t(millis() - lastSample) < 20) return;   // 50 control frames per second
  lastSample = millis();
  ControlData control{};
  for (int i = 0; i < 4; ++i) {
    uint16_t raw = readStick(i);
    if (sticksCalibrated) control.axes[i] = normalizeAxis(i, raw);
    else centerSum[i] += raw;
  }
  if (!sticksCalibrated && ++centerCount == 50) {      // 50 samples = 1 s
    for (int i = 0; i < 4; ++i) stickCenter[i] = centerSum[i] / centerCount;
    sticksCalibrated = true;
  }
  control.calibrated = sticksCalibrated;
  control.arm = buttonDown[0];
  control.kill = buttonDown[1];
  if (radioOK) sendControl(control);
}
