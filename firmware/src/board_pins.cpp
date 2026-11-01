#include "board_pins.h"

namespace board {

void makeSafe() {
  for (const auto& slot : kSlotPins) {
    digitalWrite(slot.enable, HIGH);  // set the level before switching to output: no glitch
    pinMode(slot.enable, OUTPUT);
  }
  for (uint32_t pin : kHeaterPins) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
  for (uint32_t pin : kFanPins) {
    digitalWrite(pin, LOW);
    pinMode(pin, OUTPUT);
  }
  // Pulled up so a disconnected optical endstop reads as triggered (D-24).
  for (uint32_t pin : kEndstopPins) {
    pinMode(pin, INPUT_PULLUP);
  }
}

}  // namespace board
