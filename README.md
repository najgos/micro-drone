# DR3 – micro quadcopter with a custom flight controller

![DR3 drone with the custom controller board](docs/images/drone_top.jpg)

DR3 is a micro drone built from scratch: a custom PCB flight controller designed in KiCad and
C++ firmware for the drone (Seeed XIAO ESP32-C6) and the hand-held remote (ESP32).
Both boards calibrate themselves at power-up; the flight controller gains are defined in `config.h`.

## System overview

| Part | Hardware | Role |
|---|---|---|
| Drone | Custom PCB: Seeed XIAO ESP32-C6, MPU6050 IMU, 4 MOSFET motor channels | Attitude estimation, cascaded PID control, motor PWM, arming logic |
| Remote | ESP32 Dev Module, 2 analog joysticks with ARM / KILL buttons | Reads the sticks and sends control frames over ESP-NOW |

## Hardware – controller PCB

![PCB layout in KiCad](docs/images/pcb_layout_kicad.png)

- 2-layer PCB, approx. 45.5 × 51 mm, designed in KiCad (schematic, layout, Gerber files)
- 4 motor channels: N-MOSFET (SOT-23) low-side switch with a Schottky flyback diode each
- Seeed XIAO ESP32-C6 module and MPU6050 (GY-521) IMU over I²C
- USB VBUS and the motor supply share the 5 V rail; the firmware keeps the motors off while USB is connected
- 0805 / SOT-23 / SMA footprints; design and production files in [`dr3/`](dr3)

![Controller board on the frame](docs/images/drone_side.jpg)

## Firmware

One Arduino sketch for both boards in [`firmware/dr3/`](firmware/dr3). The role is chosen automatically
from the selected board: **XIAO_ESP32C6 → drone**, **ESP32 Dev Module → remote**.

- **Attitude estimation:** gyro and accelerometer fusion in the FRD body frame
- **Control:** angle P loop → angular-rate PID (filtered derivative on measurement, anti-windup), X-frame mixer
- **Arming and stop conditions:** state machine (locked / disarmed / armed / fault); arming only at zero
  throttle with centred sticks; motors stop on KILL, link loss, IMU fault, excessive tilt or USB connection
- **Radio:** ESP-NOW control frames with a project ID and sequence numbers; the drone accepts frames only
  from the first remote it hears and only newer than the previous one
- **Calibration at power-up:** the remote averages the stick centres, the drone averages 200 gyro samples
  to obtain the gyro bias
- **Status LED:** fault, armed, ready to arm, waiting

| File | Purpose |
|---|---|
| `dr3.ino` | selects the role from the board |
| `config.h` | hardware settings: radio (channel, project ID, drone MAC), pins, IMU orientation, stick ranges, controller gains |
| `Drone.h` / `Remote.h` | main program of the drone / the remote |
| `Control.h` | flight controller, mixer and arming |
| `Attitude.h`, `Imu.h` | attitude estimator, MPU6050 driver |
| `Radio.h`, `Protocol.h` | ESP-NOW link and control frame format |

## Controller gains

Defined in `config.h`.

| Axis | P | I | D |
|---|---|---|---|
| Roll rate | 0.0007 | 0.0025 | 0.00001 |
| Pitch rate | 0.001 | 0.0025 | 0.00001 |
| Yaw rate | 0.0005 | 0.001 | 0 |

| Parameter | Value |
|---|---|
| Angle loop gain (`ANGLE_P`) | 8 1/s |
| Max angle at full stick | 30° |
| Target rate limit | 45 °/s |
| Motor output limit | 0.8 |
| Integral term limit | 0.1 |
| Derivative low-pass filter | 15 Hz |

## Radio frame

| Field | Size |
|---|---|
| magic (frame format) | 4 B |
| project ID | 8 B |
| sequence number | 4 B |
| 4 stick axes (−1000…1000) | 8 B |
| calibrated / ARM / KILL flags | 4 B |
| **total** | **28 B** |

## Build

Arduino-ESP32 3.3.x (tested with 3.3.10). Board **XIAO_ESP32C6** (USB CDC On Boot enabled) builds the drone,
**ESP32 Dev Module** builds the remote. Hardware-specific settings (radio, pins, IMU mounting matrix,
stick ranges) are in `config.h`.

## Repository layout

```
firmware/dr3/   drone and remote firmware (C++)
dr3/            KiCad project and Gerber files
docs/images/    photos and the KiCad screenshot used here
```
