#pragma once
// Flight controller and arming.
// States: 0 locked, 1 disarmed, 2 armed, 3 fault (cleared with KILL).
uint8_t controlState = 0;
bool pwmOK = false, killReleased = false, armReleased = false;
uint32_t armHold = 0, lastControl = 0;
float integrals[3]{}, derivative[3]{}, previousRates[3]{};
ControlData controlInput{};

// Rate PID gains per axis (roll, pitch, yaw) from config.h.
constexpr float KP[3] = {ROLL_P, PITCH_P, YAW_P};
constexpr float KI[3] = {ROLL_I, PITCH_I, YAW_I};
constexpr float KD[3] = {ROLL_D, PITCH_D, YAW_D};

void zeroController() {
  for (int i = 0; i < 3; ++i) { integrals[i] = 0; derivative[i] = 0; previousRates[i] = attitude.bodyG[i]; }
}
void stopOutputs() {
  for (uint8_t pin : MOTOR_PINS) if (pwmOK) ledcWrite(pin, 0); else digitalWrite(pin, LOW);
}
void disarm(bool fault = false) {
  controlState = fault ? 3 : 1;
  armReleased = false; armHold = 0;
  stopOutputs(); zeroController();
}
// PWM 20 kHz, 10 bit, starting at zero duty cycle.
void beginOutputs() {
  pwmOK = true;
  for (uint8_t pin : MOTOR_PINS) if (!ledcAttach(pin, 20000, 10)) pwmOK = false;
  stopOutputs();
}

bool linkFresh() { return peerKnown && uint32_t(millis() - peerSeen) <= LINK_TIMEOUT_MS; }
// Conditions for running the motors, checked continuously (USB blocks the motors: shared 5 V rail).
bool canOperate(bool imuValid) {
  return !bool(Serial) && imuValid && gyroCalibrated && attitude.valid && pwmOK && linkFresh() && controlInput.calibrated;
}

void controlStep(bool imuValid, bool freshSample) {
  uint32_t now = millis();
  bool fresh = linkFresh();
  if (controlInput.kill && fresh) { killReleased = false; disarm(); return; }
  if (!controlInput.kill && fresh) killReleased = true;
  if (!canOperate(imuValid)) {
    if (controlState == 2) disarm(true);
    else { stopOutputs(); if (controlState != 3) controlState = 0; }
    return;
  }
  if (controlState == 0) controlState = 1;
  if (controlState == 3) { stopOutputs(); return; }   // a fault is cleared only by KILL

  // The throttle stick springs back to the centre: centre and lower half = 0, upper half = throttle 0..1.
  float throttle = constrain(controlInput.axes[0] * .001f, 0.f, 1.f);
  if (!controlInput.arm) { armReleased = true; armHold = 0; }

  if (controlState == 1) {   // disarmed: arm after holding ARM for 1.5 s
    stopOutputs();
    bool sticksCentered = abs(controlInput.axes[1]) < 100 && abs(controlInput.axes[2]) < 100 && abs(controlInput.axes[3]) < 100;
    if (controlInput.arm && armReleased && killReleased && throttle < .03f && sticksCentered) {
      if (!armHold) armHold = now;
      if (uint32_t(now - armHold) >= 1500) { controlState = 2; zeroController(); lastControl = now; armReleased = false; }
    } else armHold = 0;
    return;
  }

  // Armed: the controller runs only on a new IMU sample.
  if (!freshSample) return;
  float dt = uint32_t(now - lastControl) * .001f;
  lastControl = now;
  if (dt <= 0 || dt > .03f) { disarm(true); return; }
  if (throttle < .03f) { stopOutputs(); zeroController(); return; }
  float roll, pitch, yaw;
  attitude.angles(roll, pitch, yaw);
  if (fabsf(roll) > 45 || fabsf(pitch) > 45) { disarm(true); return; }

  // Cascade: (stick angle − measured angle) × ANGLE_P = target angular rate.
  // RX = roll, RY pushed forward = nose down, LX = yaw (rate only).
  float rateTarget[3] = {
    constrain((controlInput.axes[3] * .001f * MAX_ANGLE - roll) * ANGLE_P, -MAX_RATE, MAX_RATE),
    constrain((-controlInput.axes[2] * .001f * MAX_ANGLE - pitch) * ANGLE_P, -MAX_RATE, MAX_RATE),
    controlInput.axes[1] * .001f * MAX_RATE};

  // Rate PID for each axis: D computed from the measurement (no kick on stick moves) and low-pass filtered.
  float u[3], oldI[3];
  float alpha = dt / (dt + 1 / (2 * PI * D_CUTOFF_HZ));
  for (int i = 0; i < 3; ++i) {
    float error = rateTarget[i] - attitude.bodyG[i];
    oldI[i] = integrals[i];
    integrals[i] = constrain(integrals[i] + KI[i] * error * dt, -I_LIMIT, I_LIMIT);
    derivative[i] += alpha * ((attitude.bodyG[i] - previousRates[i]) / dt - derivative[i]);
    previousRates[i] = attitude.bodyG[i];
    u[i] = KP[i] * error + integrals[i] - KD[i] * derivative[i];
  }

  // X mixer: FR CCW, RR CW, RL CCW, FL CW.
  float base = throttle * MAX_OUTPUT;
  float mixed[4] = {base - u[0] + u[1] + u[2], base - u[0] - u[1] - u[2],
                    base + u[0] - u[1] + u[2], base + u[0] + u[1] - u[2]};
  bool saturated = false;
  for (float v : mixed) if (v < 0 || v > MAX_OUTPUT) saturated = true;
  if (saturated) for (int i = 0; i < 3; ++i) integrals[i] = oldI[i];   // anti-windup
  for (int i = 0; i < 4; ++i) ledcWrite(MOTOR_PINS[i], uint32_t(constrain(mixed[i], 0.f, MAX_OUTPUT) * 1023));
}
