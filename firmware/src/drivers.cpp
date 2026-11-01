#include "drivers.h"

#include <SPI.h>
#include <SoftwareSerial.h>
#include <TMCStepper.h>

#include "board_pins.h"
#include "config.h"

namespace drivers {
namespace {

constexpr uint8_t kVersion5160 = 0x30;
constexpr uint8_t kVersion2209 = 0x21;

// Only the entry matching each driver's type is set.
TMC5160Stepper* spiDrivers[kDriverCount] = {};
TMC2209Stepper* uartDrivers[kDriverCount] = {};
SoftwareSerial* uartPorts[kDriverCount] = {};
bool enabled[kDriverCount] = {};

uint8_t configure5160(TMC5160Stepper& d, const DriverConfig& c) {
  d.begin();
  d.toff(3);
  d.blank_time(24);
  d.en_pwm_mode(false);  // SpreadCycle
  d.rms_current(c.run_current_ma);
  d.microsteps(c.microsteps);
  return d.version();
}

uint8_t configure2209(TMC2209Stepper& d, SoftwareSerial& port, const DriverConfig& c) {
  port.listen();  // only one software UART can receive at a time
  d.begin();
  d.pdn_disable(true);       // the PDN_UART pin is used for UART, not power-down
  d.mstep_reg_select(true);  // microsteps from the register, not the MS pins
  d.I_scale_analog(false);   // current from the register, not the VREF potentiometer
  d.toff(3);
  d.blank_time(24);
  d.en_spreadCycle(false);  // StealthChop
  d.pwm_autoscale(true);
  d.rms_current(c.run_current_ma);
  d.microsteps(c.microsteps);
  return d.version();
}

}  // namespace

void begin() {
  SPI.setSCLK(board::kSpiSck);
  SPI.setMISO(board::kSpiMiso);
  SPI.setMOSI(board::kSpiMosi);

  for (size_t i = 0; i < kDriverCount; ++i) {
    const DriverConfig& c = kDrivers[i];
    const board::DriverSlotPins& p = board::kSlotPins[c.slot];
    digitalWrite(p.step, LOW);
    pinMode(p.step, OUTPUT);
    pinMode(p.dir, OUTPUT);

    if (c.type == DriverType::TMC5160) {
      // Every chip select must be high before the shared SPI bus is used.
      digitalWrite(p.cs_uart, HIGH);
      pinMode(p.cs_uart, OUTPUT);
      spiDrivers[i] = new TMC5160Stepper(p.cs_uart, kRSense5160);
    } else {
      uartPorts[i] = new SoftwareSerial(p.cs_uart, p.cs_uart);  // same pin: half duplex
      uartPorts[i]->begin(kTmc2209Baud);
      uartDrivers[i] = new TMC2209Stepper(uartPorts[i], kRSense2209, 0);
    }
  }
  SPI.begin();
}

bool configureAll(Print& out) {
  bool allOk = true;
  for (size_t i = 0; i < kDriverCount; ++i) {
    const DriverConfig& c = kDrivers[i];
    uint8_t version;
    uint8_t expected;
    if (c.type == DriverType::TMC5160) {
      version = configure5160(*spiDrivers[i], c);
      expected = kVersion5160;
    } else {
      version = configure2209(*uartDrivers[i], *uartPorts[i], c);
      expected = kVersion2209;
    }
    bool ok = version == expected;
    allOk = allOk && ok;
    out.printf("DRIVER %s slot=%u type=%s version=0x%02X %s\n", c.name, c.slot,
               c.type == DriverType::TMC5160 ? "TMC5160" : "TMC2209", version,
               ok ? "ok" : "NO ANSWER (motor power off, wrong jumpers, or wrong slot?)");
  }
  return allOk;
}

void setEnabled(size_t driver, bool on) {
  digitalWrite(board::kSlotPins[kDrivers[driver].slot].enable, on ? LOW : HIGH);
  enabled[driver] = on;
}

bool isEnabled(size_t driver) { return enabled[driver]; }

}  // namespace drivers
