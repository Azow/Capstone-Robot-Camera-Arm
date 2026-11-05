#pragma once

#include <Arduino.h>

// Pin map for the BigTreeTech Octopus with the STM32F446ZE. These are the Octopus V1.0/V1.1
// and Octopus Pro V1.0/V1.0.1 pins (Klipper's generic-bigtreetech-octopus-v1.1.cfg),
// checked against the team's driver test sketch that ran on this board.
//
// Do not copy pins from the Octopus Pro v1.1 (H723) config: some differ. Driver3's enable
// pin is PA0 here; on the Pro v1.1 it is PA2, which here is the HE0 heater output.

namespace board {

struct DriverSlotPins {
  uint32_t step;
  uint32_t dir;
  uint32_t enable;   // active low
  uint32_t cs_uart;  // SPI chip select (TMC5160) or single-wire UART (TMC2209)
};

// Indexed by the slot number printed on the board (Driver0 .. Driver7).
constexpr DriverSlotPins kSlotPins[8] = {
    {PF13, PF12, PF14, PC4},   // Driver0
    {PG0, PG1, PF15, PD11},    // Driver1
    {PF11, PG3, PG5, PC6},     // Driver2
    {PG4, PC1, PA0, PC7},      // Driver3
    {PF9, PF10, PG2, PF2},     // Driver4
    {PC13, PF0, PF1, PE4},     // Driver5
    {PE2, PE3, PD4, PE1},      // Driver6
    {PE6, PA14, PE0, PD3},     // Driver7 (dir PA14 is also the SWD clock)
};

// Endstop inputs STOP_0 .. STOP_7.
constexpr uint32_t kEndstopPins[8] = {PG6, PG9, PG10, PG11, PG12, PG13, PG14, PG15};

// SPI bus shared by all TMC5160 slots (SPI1, also wired to the EXP2 header).
constexpr uint32_t kSpiSck = PA5;
constexpr uint32_t kSpiMiso = PA6;
constexpr uint32_t kSpiMosi = PA7;

// Outputs we never use. Driven off at startup so nothing heats or spins by accident.
constexpr uint32_t kHeaterPins[] = {PA2, PA3, PB10, PB11, PA1};  // HE0..HE3, bed
constexpr uint32_t kFanPins[] = {PA8, PE5, PD12, PD13, PD14, PD15};  // FAN0..FAN5

// Disables every driver slot, turns heaters and fans off, and sets up the endstop inputs.
// Call first thing in setup().
void makeSafe();

}  // namespace board
