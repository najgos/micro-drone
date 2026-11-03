#pragma once
// MPU6050 over I²C: configuration and sample readout.
#include <Wire.h>

// Ranges ±8 g and ±1000 °/s (GYRO_CONFIG and ACCEL_CONFIG = 0x10).
constexpr float ACCEL_LSB_PER_G = 4096.0f, GYRO_LSB_PER_DPS = 32.8f;

struct ImuSample { int16_t a[3], g[3]; };
uint8_t imuAddress = 0x68;

bool readRegisters(uint8_t reg, uint8_t *data, size_t count) {
  Wire.beginTransmission(imuAddress);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(imuAddress, count, true) != count) return false;
  for (size_t i = 0; i < count; ++i) data[i] = static_cast<uint8_t>(Wire.read());
  return true;
}
bool writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(imuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

// Sensor detection (address 0x68 or 0x69) and configuration: 100 samples/s, DLPF=3.
bool beginImu() {
  bool found = false;
  for (uint8_t address = 0x68; address <= 0x69 && !found; ++address) {
    uint8_t id = 0;
    imuAddress = address;
    found = readRegisters(0x75, &id, 1) && id == 0x68;   // WHO_AM_I
  }
  if (!found || !writeRegister(0x6B, 0x80)) return false;   // reset
  delay(100);
  bool ok = writeRegister(0x6B, 0x01) && writeRegister(0x6C, 0) && writeRegister(0x19, 9) &&
            writeRegister(0x1A, 3) && writeRegister(0x1B, 0x10) && writeRegister(0x1C, 0x10) &&
            writeRegister(0x38, 1);
  delay(100);
  return ok;
}

// 14 bytes from register 0x3B: accelerometer XYZ, temperature (skipped), gyroscope XYZ.
bool readSample(ImuSample &out) {
  uint8_t b[14]{};
  if (!readRegisters(0x3B, b, sizeof(b))) return false;
  for (int i = 0; i < 3; ++i) {
    out.a[i] = static_cast<int16_t>((b[2 * i] << 8) | b[2 * i + 1]);
    out.g[i] = static_cast<int16_t>((b[8 + 2 * i] << 8) | b[8 + 2 * i + 1]);
  }
  return true;
}
