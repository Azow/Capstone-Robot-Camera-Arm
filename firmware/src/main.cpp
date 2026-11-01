#include <Arduino.h>

#include "board_pins.h"
#include "drivers.h"
#include "shell.h"

void setup() {
  board::makeSafe();  // drivers disabled, heaters and fans off, before anything else
  Serial.begin(115200);  // USB serial: the baud rate is ignored
  drivers::begin();
}

void loop() { shell::poll(); }
