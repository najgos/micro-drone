#pragma once
// =====================================================================
//  DR3 – hardware settings for the drone and the remote.
// =====================================================================

// ------------------------- RADIO (shared) -------------------------
constexpr uint8_t RADIO_CHANNEL = 6;   // Wi-Fi channel used by ESP-NOW, same on both boards
// Set identifier – tells this drone and remote apart from other DR3 sets nearby.
constexpr uint8_t PROJECT_ID[8] = {'D', 'R', '3', '-', '0', '0', '0', '1'};
// Wi-Fi MAC address of the drone (XIAO ESP32-C6).
constexpr uint8_t DRONE_MAC[6] = {0x10, 0xBD, 0xA3, 0xA1, 0xC2, 0xDC};

// ------------------------- DRONE (XIAO ESP32-C6) -------------------------

// Motors (top view, drone nose pointing away from the operator):
// M1 front right (CCW), M2 rear right (CW), M3 rear left (CCW), M4 front left (CW).
constexpr uint8_t MOTOR_PINS[4] = {0, 1, 2, 21};   // XIAO: D0, D1, D2, D3
constexpr uint8_t IMU_SDA = 22, IMU_SCL = 23;        // XIAO: D4, D5
constexpr uint8_t DRONE_LED_PIN = 15;                // XIAO yellow user LED
constexpr bool DRONE_LED_ACTIVE_LOW = true;

// MPU6050 orientation relative to the frame: sensor -> body rotation matrix, row by row.
// FRD body frame: X forward, Y right, Z down.
// Default: GY-521 module lying flat, components facing up, X arrow pointing to the nose.
// Other common mountings (module flat, components up):
//   X arrow backwards:  {-1, 0, 0,   0, 1, 0,   0, 0,-1}
//   X arrow to the right: { 0, 1, 0,   1, 0, 0,   0, 0,-1}
//   X arrow to the left:  { 0,-1, 0,  -1, 0, 0,   0, 0,-1}
constexpr float MOUNT_MATRIX[9] = { 1, 0, 0,
                                    0,-1, 0,
                                    0, 0,-1 };

// Flight controller gains.
constexpr float ROLL_P  = 0.0007f, ROLL_I  = 0.0025f, ROLL_D  = 0.00001f;   // roll rate PID  [PWM per °/s]
constexpr float PITCH_P = 0.001f,  PITCH_I = 0.0025f, PITCH_D = 0.00001f;   // pitch rate PID [PWM per °/s]
constexpr float YAW_P   = 0.0005f, YAW_I   = 0.001f,  YAW_D   = 0.0f;       // yaw rate PID   [PWM per °/s]
constexpr float ANGLE_P = 8.0f;          // angle error -> target rate [1/s]
constexpr float MAX_ANGLE = 30.0f;       // full stick deflection [°]
constexpr float MAX_RATE = 45.0f;        // target rate limit [°/s]
constexpr float MAX_OUTPUT = 0.8f;       // motor output limit [0–1]
constexpr float I_LIMIT = 0.1f;          // integral term limit [PWM]
constexpr float D_CUTOFF_HZ = 15.0f;     // derivative low-pass filter

// ------------------------- REMOTE (ESP32 Dev Module) -------------------------

// Sticks: LY = throttle, LX = yaw, RY = pitch, RX = roll.
constexpr uint8_t STICK_PINS[4] = {34, 35, 32, 33};
// ADC reading (0–4095) at both ends of each axis. Centres are measured at power-up.
constexpr uint16_t STICK_LOW[4]  = {   0,    0,    0,    0};
constexpr uint16_t STICK_HIGH[4] = {4095, 4095, 4095, 4095};
// Axis inversion: X must increase to the right, Y must increase when pushed forward.
constexpr bool STICK_INVERT[4] = {false, true, true, false};
constexpr float STICK_DEADBAND[4] = {0.03f, 0.03f, 0.03f, 0.03f};

constexpr uint8_t ARM_PIN = 26, KILL_PIN = 25;       // buttons in the left and right stick
constexpr bool BUTTONS_ACTIVE_LOW = true;            // pressing shorts the pin to GND
