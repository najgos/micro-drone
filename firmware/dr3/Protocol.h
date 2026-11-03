#pragma once
// Control data sent from the remote to the drone.

constexpr uint32_t FRAME_MAGIC = 0x44523303;
constexpr uint32_t LINK_TIMEOUT_MS = 200;   // the drone stops the motors after this long without control

struct ControlData {
  int16_t axes[4];                // LY, LX, RY, RX in the range -1000..1000
  uint8_t calibrated, arm, kill, reserved;
};
