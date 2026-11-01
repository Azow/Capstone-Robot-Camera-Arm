#include "shell.h"

#include <Arduino.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "board_pins.h"
#include "config.h"
#include "drivers.h"

namespace shell {
namespace {

constexpr const char* kFirmwareVersion = "bringup-0.1";
constexpr size_t kLineMax = 96;
constexpr long kMaxSpinRate = 20000;  // steps/s; no acceleration, so keep test speeds low
constexpr long kDefaultSpinRate = 400;

char line[kLineMax];
size_t lineLength = 0;
bool lineTooLong = false;

const char* const kHelp =
    "Commands (one per line, not case-sensitive):\n"
    "  PING                         reply OK\n"
    "  INFO                         firmware version and CPU clock\n"
    "  DRIVERS                      configure all drivers and check they answer\n"
    "                               (needs motor power; run again after it's cycled)\n"
    "  ENABLE <joint|ALL>           energize a joint's driver(s)\n"
    "  DISABLE <joint|ALL>          de-energize (Z will fall unless supported!)\n"
    "  SPIN <joint> <steps> [rate]  move a joint; negative steps reverse;\n"
    "                               rate in steps/s (default 400). Any input aborts.\n"
    "  ENDSTOPS                     raw endstop input levels\n"
    "Joints: THETA Z R YAW PITCH";

// Returns the joint index, JOINT_COUNT for "ALL", or -1 if unknown.
int parseJoint(const char* name, bool allowAll) {
  if (allowAll && strcmp(name, "ALL") == 0) {
    return JOINT_COUNT;
  }
  for (int j = 0; j < JOINT_COUNT; ++j) {
    if (strcmp(name, kJoints[j].name) == 0) {
      return j;
    }
  }
  return -1;
}

bool driverInJoint(size_t driver, int joint) {
  return joint == JOINT_COUNT || kDrivers[driver].joint == joint;
}

void cmdInfo() {
  Serial.printf("OK firmware=%s sysclk=%lu Hz\n", kFirmwareVersion,
                (unsigned long)HAL_RCC_GetSysClockFreq());
}

void cmdDrivers() {
  if (drivers::configureAll(Serial)) {
    Serial.println("OK all drivers answered");
  } else {
    Serial.println("ERR some drivers did not answer");
  }
}

void cmdEnable(const char* arg, bool on) {
  int joint = arg ? parseJoint(arg, true) : -1;
  if (joint < 0) {
    Serial.println("ERR usage: ENABLE|DISABLE <joint|ALL>");
    return;
  }
  for (size_t d = 0; d < kDriverCount; ++d) {
    if (driverInJoint(d, joint)) {
      drivers::setEnabled(d, on);
    }
  }
  Serial.printf("OK %s %s\n", arg, on ? "enabled" : "disabled");
}

// Bring-up only: blocking, constant speed, no acceleration. Replaced by the timer-driven
// step generator and segment queue (D-05).
void cmdSpin(const char* jointArg, const char* stepsArg, const char* rateArg) {
  int joint = jointArg ? parseJoint(jointArg, false) : -1;
  if (joint < 0 || !stepsArg) {
    Serial.println("ERR usage: SPIN <joint> <steps> [rate]");
    return;
  }
  long steps = strtol(stepsArg, nullptr, 10);
  long rate = rateArg ? strtol(rateArg, nullptr, 10) : kDefaultSpinRate;
  if (rate <= 0 || rate > kMaxSpinRate) {
    Serial.printf("ERR rate must be 1..%ld steps/s\n", kMaxSpinRate);
    return;
  }

  size_t joined[kDriverCount];
  size_t count = 0;
  for (size_t d = 0; d < kDriverCount; ++d) {
    if (driverInJoint(d, joint)) {
      if (!drivers::isEnabled(d)) {
        Serial.printf("ERR %s is disabled; ENABLE it first\n", kJoints[joint].name);
        return;
      }
      joined[count++] = d;
    }
  }

  bool forward = steps >= 0;
  long total = labs(steps);
  for (size_t k = 0; k < count; ++k) {
    const DriverConfig& c = kDrivers[joined[k]];
    digitalWrite(board::kSlotPins[c.slot].dir, (forward != c.invert_dir) ? HIGH : LOW);
  }
  delayMicroseconds(5);  // direction setup time before the first step

  uint32_t interval = 1000000UL / rate;
  uint32_t next = micros();
  long done = 0;
  for (; done < total; ++done) {
    if (Serial.available()) {
      break;
    }
    while ((int32_t)(micros() - next) < 0) {
    }
    next += interval;
    for (size_t k = 0; k < count; ++k) {
      digitalWrite(board::kSlotPins[kDrivers[joined[k]].slot].step, HIGH);
    }
    delayMicroseconds(2);
    for (size_t k = 0; k < count; ++k) {
      digitalWrite(board::kSlotPins[kDrivers[joined[k]].slot].step, LOW);
    }
  }

  if (done < total) {
    Serial.printf("ERR aborted after %ld of %ld steps\n", done, total);
  } else {
    Serial.printf("OK %s moved %ld steps\n", kJoints[joint].name, forward ? total : -total);
  }
}

void cmdEndstops() {
  Serial.print("OK");
  for (const JointConfig& j : kJoints) {
    Serial.printf(" %s=%d", j.name, digitalRead(board::kEndstopPins[j.endstop]));
  }
  Serial.println();
}

void runLine(char* text) {
  for (char* p = text; *p; ++p) {
    *p = toupper(*p);
  }
  const char* cmd = strtok(text, " \t");
  if (!cmd) {
    return;
  }
  const char* a1 = strtok(nullptr, " \t");
  const char* a2 = strtok(nullptr, " \t");
  const char* a3 = strtok(nullptr, " \t");

  if (strcmp(cmd, "HELP") == 0) {
    Serial.println(kHelp);
    Serial.println("OK");
  } else if (strcmp(cmd, "PING") == 0) {
    Serial.println("OK");
  } else if (strcmp(cmd, "INFO") == 0) {
    cmdInfo();
  } else if (strcmp(cmd, "DRIVERS") == 0) {
    cmdDrivers();
  } else if (strcmp(cmd, "ENABLE") == 0) {
    cmdEnable(a1, true);
  } else if (strcmp(cmd, "DISABLE") == 0) {
    cmdEnable(a1, false);
  } else if (strcmp(cmd, "SPIN") == 0) {
    cmdSpin(a1, a2, a3);
  } else if (strcmp(cmd, "ENDSTOPS") == 0) {
    cmdEndstops();
  } else {
    Serial.printf("ERR unknown command %s (try HELP)\n", cmd);
  }
}

}  // namespace

void poll() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineTooLong) {
        Serial.println("ERR line too long");
      } else if (lineLength > 0) {
        line[lineLength] = '\0';
        runLine(line);
      }
      lineLength = 0;
      lineTooLong = false;
    } else if (lineLength < kLineMax - 1) {
      line[lineLength++] = c;
    } else {
      lineTooLong = true;
    }
  }
}

}  // namespace shell
