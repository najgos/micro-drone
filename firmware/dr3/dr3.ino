// DR3 – drone and remote firmware in a single sketch (Arduino-ESP32 3.x).
//
// The role is selected automatically from the board chosen in Arduino IDE:
//   XIAO_ESP32C6      -> drone  (Drone.h)
//   ESP32 Dev Module  -> remote (Remote.h)
#include <Arduino.h>
#include "config.h"
#include "Protocol.h"
#include "Radio.h"

#if defined(CONFIG_IDF_TARGET_ESP32C6)
#include "Drone.h"
void setup() { droneSetup(); }
void loop() { droneLoop(); }
#elif defined(CONFIG_IDF_TARGET_ESP32)
#include "Remote.h"
void setup() { remoteSetup(); }
void loop() { remoteLoop(); }
#else
#error "Select the XIAO_ESP32C6 board (drone) or ESP32 Dev Module (remote)."
#endif
