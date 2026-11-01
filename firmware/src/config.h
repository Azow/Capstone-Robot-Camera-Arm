#pragma once

#include <Arduino.h>

// Which joint is on which driver slot, and how each driver is set up.
// Decisions D-10, D-12, D-16 in docs/control_system_decisions.md.
//
// Currents and microsteps are low bring-up placeholders until the motors are chosen (D-14).

enum class DriverType : uint8_t { TMC5160, TMC2209 };

enum Joint : uint8_t { THETA, Z, R, YAW, PITCH, JOINT_COUNT };

struct JointConfig {
  const char* name;
  uint8_t endstop;  // index into board::kEndstopPins (STOP_n)
};

struct DriverConfig {
  const char* name;
  Joint joint;
  uint8_t slot;  // index into board::kSlotPins (Driver n)
  DriverType type;
  bool invert_dir;
  uint16_t run_current_ma;  // RMS
  uint16_t microsteps;
};

constexpr JointConfig kJoints[JOINT_COUNT] = {
    {"THETA", 0},
    {"Z", 1},
    {"R", 3},
    {"YAW", 4},
    {"PITCH", 5},
};

// Z1 and Z2 always get the same steps (D-12). Their settings must match; if the two motors
// face opposite ways, flip invert_dir on one of them.
constexpr DriverConfig kDrivers[] = {
    {"THETA", THETA, 0, DriverType::TMC5160, false, 400, 16},
    {"Z1", Z, 1, DriverType::TMC5160, false, 400, 16},
    {"Z2", Z, 2, DriverType::TMC5160, false, 400, 16},
    {"R", R, 3, DriverType::TMC5160, false, 400, 16},
    {"YAW", YAW, 4, DriverType::TMC2209, false, 300, 16},
    {"PITCH", PITCH, 5, DriverType::TMC2209, false, 300, 16},
};
constexpr size_t kDriverCount = sizeof(kDrivers) / sizeof(kDrivers[0]);

// Sense resistors set the current scale. VERIFY against the driver boards: if the value here
// is higher than the real resistor, the motors get MORE current than requested.
constexpr float kRSense5160 = 0.022f;  // BTT TMC5160T Pro
constexpr float kRSense2209 = 0.110f;  // BTT TMC2209

constexpr uint32_t kTmc2209Baud = 19200;  // software UART; the TMC2209 detects the baud rate
