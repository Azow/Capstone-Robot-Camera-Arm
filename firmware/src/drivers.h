#pragma once

#include <Arduino.h>

// The TMC drivers: configuration over SPI (TMC5160) / UART (TMC2209), and the enable pins.
// Indexes are into kDrivers (config.h).

namespace drivers {

// Sets up the step/dir/CS pins and the SPI and UART links. Does not talk to the drivers:
// they only answer when motor power is on.
void begin();

// Writes the settings to every driver and checks that each one answers with its version.
// Prints one line per driver to `out`. Returns true if all answered.
// Drivers lose their settings when motor power goes off, so run this again after.
bool configureAll(Print& out);

void setEnabled(size_t driver, bool on);
bool isEnabled(size_t driver);

}  // namespace drivers
