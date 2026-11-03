#pragma once
// Attitude estimator: quaternion describing the rotation of the drone body (FRD) relative to the ground.
// The gyroscope tracks fast changes but drifts once integrated.
// The accelerometer measures gravity ((0,0,-1) g when level) and slowly corrects that drift.
// The MPU6050 has no compass, so yaw is only relative.
struct Attitude {
  float q[4] = {1, 0, 0, 0}, bodyA[3] = {}, bodyG[3] = {};
  uint32_t last = 0;
  bool valid = false;

  void reset() { q[0] = 1; q[1] = q[2] = q[3] = 0; valid = false; last = 0; }

  // a [g], g [°/s] in sensor axes; at = sample time [ms]
  void update(const float *a, const float *g, uint32_t at) {
    // 1. Rotate the measurement from sensor axes to body axes (MOUNT_MATRIX in config.h).
    for (int i = 0; i < 3; ++i) {
      bodyA[i] = bodyG[i] = 0;
      for (int j = 0; j < 3; ++j) {
        bodyA[i] += MOUNT_MATRIX[3 * i + j] * a[j];
        bodyG[i] += MOUNT_MATRIX[3 * i + j] * g[j];
      }
    }
    float n = sqrtf(bodyA[0] * bodyA[0] + bodyA[1] * bodyA[1] + bodyA[2] * bodyA[2]);

    // 2. Start (or gap >150 ms): attitude from gravity only, yaw = 0.
    if (!valid || uint32_t(at - last) > 150) {
      if (n < .85f || n > 1.15f) { reset(); return; }
      float r = atan2f(-bodyA[1], -bodyA[2]), p = atan2f(bodyA[0], hypotf(bodyA[1], bodyA[2]));
      q[0] = cosf(r / 2) * cosf(p / 2); q[1] = sinf(r / 2) * cosf(p / 2);
      q[2] = cosf(r / 2) * sinf(p / 2); q[3] = -sinf(r / 2) * sinf(p / 2);
      last = at; valid = true;
      return;
    }
    float dt = uint32_t(at - last) * .001f;
    if (dt <= 0) return;
    last = at;

    float omega[3];
    for (int i = 0; i < 3; ++i) omega[i] = bodyG[i] * DEG_TO_RAD;
    float w = q[0], x = q[1], y = q[2], z = q[3];

    // 3. Accelerometer correction (only when the drone is not accelerating hard, |a| ≈ 1 g):
    //    the cross product of measured and predicted gravity rotates the estimate towards the measurement.
    if (n > .85f && n < 1.15f) {
      float predicted[3] = {-2 * (x * z - y * w), -2 * (y * z + x * w), -1 + 2 * (x * x + y * y)};
      for (int i = 0; i < 3; ++i)
        omega[i] += 2 * (bodyA[(i + 1) % 3] * predicted[(i + 2) % 3] - bodyA[(i + 2) % 3] * predicted[(i + 1) % 3]) / n;
    }

    // 4. Gyro integration: q += ½·dt·q⊗ω, then normalisation to unit length.
    q[0] += .5f * dt * (-x * omega[0] - y * omega[1] - z * omega[2]);
    q[1] += .5f * dt * (w * omega[0] + y * omega[2] - z * omega[1]);
    q[2] += .5f * dt * (w * omega[1] - x * omega[2] + z * omega[0]);
    q[3] += .5f * dt * (w * omega[2] + x * omega[1] - y * omega[0]);
    n = 0;
    for (float v : q) n += v * v;
    n = sqrtf(n);
    if (!isfinite(n) || n < .1f) { reset(); return; }
    for (float &v : q) v /= n;
  }

  // Angles in degrees: roll (right side down +), pitch (nose up +), yaw (clockwise from above +).
  void angles(float &roll, float &pitch, float &yaw) const {
    if (!valid) { roll = pitch = yaw = 0; return; }
    roll = atan2f(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2])) * RAD_TO_DEG;
    pitch = asinf(constrain(2 * (q[0] * q[2] - q[3] * q[1]), -1.f, 1.f)) * RAD_TO_DEG;
    yaw = atan2f(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3])) * RAD_TO_DEG;
  }
} attitude;
