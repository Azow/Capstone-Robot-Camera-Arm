// Arm firmware for the BigTreeTech Octopus with the STM32F446ZE.
//
// Takes protocol lines over USB serial, queues timed move segments, and steps every joint
// from a 50 kHz timer interrupt. Design decisions and the serial protocol (section 4):
// docs/control_system_decisions.md. How to build and flash: ../README.md.
//
// build_opt.h, next to this file, tells the compiler the board has a 12 MHz crystal.
//
// Sections, in order:
//   1. Settings        joints, drivers, currents: the part you're most likely to edit
//   2. Pin map         Octopus F446 pins
//   3. Types           motion queue, protocol
//   4. Hardware        safe startup, STEP/DIR pins, TMC driver setup, clock
//   5. Motion          segment queue and the Bresenham step generator (interrupt side)
//   6. Protocol        serial commands, checks, replies, events, watchdog
//   7. Bring-up        HELP, INFO, DRIVERS, ENDSTOPS, SPIN (for testing by hand)
//   8. setup / loop
//
// The other end of the protocol is src/arm_bridge (protocol.py), and the Python fake board
// (fake_board.py) implements the same rules: keep all three in step.

#include <Arduino.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TMCStepper.h>  // Library Manager: TMCStepper by teemuatlut, 0.7.3
#include <ctype.h>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Catch the Arduino IDE settings that would build fine but not work on the board.
#if !defined(STM32F446xx)
#error "Wrong board: Tools > Board > STM32 MCU based boards > Generic STM32F4 series, Board part number > Generic F446ZETx"
#endif
#if !defined(USBD_USE_CDC)
#error "USB serial is off: Tools > USB support > CDC (generic 'Serial' supersede U(S)ART)"
#endif
// The HAL computes every clock frequency (millis, delays, baud rates) from HSE_VALUE.
#if !defined(HSE_VALUE) || HSE_VALUE != 12000000U
#error "HSE_VALUE is not 12 MHz: build_opt.h must be in the sketch folder"
#endif

// =============================================================================================
// 1. Settings
// =============================================================================================
// Decisions D-10, D-12, D-14, D-16, D-35. Currents follow the motors in D-14: about 70-75% of
// each motor's rated current (RMS).

const char* const kFirmwareVersion = "0.4";

// Protocol order. FOCUS is the lens's focus ring (D-35): the PC turns camera-to-subject
// distance into a ring angle and sends it in steps like any other joint.
enum Joint : uint8_t { THETA, Z, R, YAW, PITCH, FOCUS, kJointCount };

enum class DriverType : uint8_t { TMC5160, TMC2209 };

struct JointConfig {
  const char* name;
  uint8_t endstop;  // index into kEndstopPins (STOP_n)
};

struct DriverConfig {
  const char* name;
  Joint joint;
  uint8_t slot;  // index into kSlotPins (Driver n)
  DriverType type;
  bool invert_dir;
  uint16_t run_current_ma;  // RMS
  uint16_t microsteps;
};

// Each joint's switch is on the STOP_n that matches its driver slot (Z: Z1's slot), so the
// DIAG jumper rule is simple: remove it on every slot except FOCUS's. Keep this in step with
// kDrivers below.
const JointConfig kJoints[kJointCount] = {
    {"THETA", 7},
    {"Z", 6},
    {"R", 4},
    {"YAW", 3},
    {"PITCH", 2},
    // No optical switch on focus: it will home into the lens's infinity stop by stall
    // detection (D-36). With Driver1's DIAG jumper fitted, STOP_1 reads the driver's stall
    // output, so ENDSTOPS shows FOCUS=1 while stalled. Must equal FOCUS's driver slot.
    {"FOCUS", 1},
};

// Z1 and Z2 always get the same steps (D-12). Their settings must match; if the two motors
// face opposite ways, flip invert_dir on one of them.
// All motors are 1.8 deg (200 steps/rev). Hold current is half the run current (TMCStepper's
// default hold multiplier).
const DriverConfig kDrivers[] = {
    // StepperOnline 23HS22-2804S NEMA 23, 1.26 N*m (178 oz-in), 2.8 A rated, 0.9 ohm, 2.5 mH.
    // 2000 mA = 71% of rated.
    // Driver7's DIR pin is PA14, the SWD clock: once the firmware runs, flash over SWD with
    // "connect under reset" (or use USB DFU).
    {"THETA", THETA, 7, DriverType::TMC5160, false, 2000, 16},
    // RatRig / LDO 57STH86-3004A NEMA 23, 345 oz-in, 3.0 A rated. The 5160's 0.075 ohm sense
    // resistor tops out near 2.4 A RMS.
    {"Z1", Z, 6, DriverType::TMC5160, false, 2000, 16},
    {"Z2", Z, 5, DriverType::TMC5160, false, 2000, 16},
    // Lin Engineering X5718-077-20 NEMA 23, 467 oz-in, 2.0 A rated, 2.3 ohm, bipolar 4-lead.
    // 1500 mA = 75% of rated.
    {"R", R, 4, DriverType::TMC5160, false, 1500, 16},
    // 17HS4401 NEMA 17, 40 N*cm, 1.5 A rated.
    {"YAW", YAW, 3, DriverType::TMC2209, false, 1100, 16},
    // 17HS4401 NEMA 17, 1.5 A rated. The CAD has two pitch motors but this is one driver: if
    // both are wired to it in parallel, each gets only half of this current.
    {"PITCH", PITCH, 2, DriverType::TMC2209, false, 1100, 16},
    // NEMA 14 on the follow-focus mount (Amazon B0FHHS5T9M): 35x35x28.2 mm, 12 N*cm,
    // 0.6 A rated, 4.5 ohm, 6.5 mH. 450 mA = 75% of rated.
    // Slot must be on the 24 V motor supply (TMC2209).
    // Flip invert_dir if + steps turn the ring toward infinity instead of close focus.
    {"FOCUS", FOCUS, 1, DriverType::TMC2209, false, 450, 16},
};
const size_t kDriverCount = sizeof(kDrivers) / sizeof(kDrivers[0]);

// Sense resistors set the current scale. VERIFY against the driver boards: if the value here
// is higher than the real resistor, the motors get MORE current than requested.
const float kRSense5160 = 0.075f;  // BTT TMC5160T Pro (BTT wiki; used by the driver test sketch)
const float kRSense2209 = 0.110f;  // BTT TMC2209

const uint32_t kTmc2209Baud = 19200;  // software UART; the TMC2209 detects the baud rate

// =============================================================================================
// 2. Pin map
// =============================================================================================
// The Octopus V1.0/V1.1 and Octopus Pro V1.0/V1.0.1 pins (Klipper's
// generic-bigtreetech-octopus-v1.1.cfg), checked against the team's driver test sketch that
// ran on this board.
//
// Do not copy pins from the Octopus Pro v1.1 (H723) config: some differ. Driver3's enable
// pin is PA0 here; on the Pro v1.1 it is PA2, which here is the HE0 heater output.

struct DriverSlotPins {
  uint32_t step;
  uint32_t dir;
  uint32_t enable;   // active low
  uint32_t cs_uart;  // SPI chip select (TMC5160) or single-wire UART (TMC2209)
};

// Indexed by the slot number printed on the board (Driver0 .. Driver7).
const DriverSlotPins kSlotPins[8] = {
    {PF13, PF12, PF14, PC4},  // Driver0
    {PG0, PG1, PF15, PD11},   // Driver1
    {PF11, PG3, PG5, PC6},    // Driver2
    {PG4, PC1, PA0, PC7},     // Driver3
    {PF9, PF10, PG2, PF2},    // Driver4
    {PC13, PF0, PF1, PE4},    // Driver5
    {PE2, PE3, PD4, PE1},     // Driver6
    {PE6, PA14, PE0, PD3},    // Driver7 (dir PA14 is also the SWD clock)
};

// Endstop inputs STOP_0 .. STOP_7.
const uint32_t kEndstopPins[8] = {PG6, PG9, PG10, PG11, PG12, PG13, PG14, PG15};

// SPI bus shared by all TMC5160 slots (SPI1, also wired to the EXP2 header).
const uint32_t kSpiSck = PA5;
const uint32_t kSpiMiso = PA6;
const uint32_t kSpiMosi = PA7;

// Outputs we never use. Driven off at startup so nothing heats or spins by accident.
const uint32_t kHeaterPins[] = {PA2, PA3, PB10, PB11, PA1};          // HE0..HE3, bed
const uint32_t kFanPins[] = {PA8, PE5, PD12, PD13, PD14, PD15};      // FAN0..FAN5

// =============================================================================================
// 3. Types
// =============================================================================================

// ----- Motion constants (D-05, protocol section 4) -----
const uint32_t kTickHz = 50000;                 // step interrupt rate
const uint32_t kUsPerTick = 1000000 / kTickHz;
const uint32_t kMaxRate = 40000;                // steps/s per joint; at most one step per tick
const uint32_t kQueueSize = 64;
const uint32_t kRampTicks = kTickHz / 10;       // 100 ms ramp down after a stop or underrun
const uint64_t kOneStep = 1ULL << 32;           // ramp speeds are in 1/2^32 steps per tick

// ----- Protocol constants -----
const int kProtocolVersion = 2;  // v2 added FOCUS
const uint32_t kWatchdogMs = 250;
const uint32_t kMinSegmentUs = 1000;
const uint32_t kMaxSegmentUs = 1000000;
const int kMaxLine = 120;
const int kMaxTokens = 10;  // SEG with a seq: 3 + kJointCount

// ----- Hardware constants -----
const uint32_t kPulseNs = 200;    // STEP high time; TMC5160/2209 need about 100 ns
const long kMaxSpinRate = 20000;  // steps/s; SPIN has no acceleration, keep it slow
const long kDefaultSpinRate = 400;
const uint8_t kVersion5160 = 0x30;
const uint8_t kVersion2209 = 0x21;

enum class State : uint8_t { kIdle, kRunning, kStopping, kHoming, kFault };

enum Event : uint8_t { kEventDone, kEventStopped, kEventUnderrun, kEventWatchdog, kEventCount };
const char* const kEventNames[kEventCount] = {"DONE", "STOPPED", "UNDERRUN", "WATCHDOG"};

struct Segment {
  int32_t target[kJointCount];
  uint32_t ticks;
  uint32_t seq;
};

struct JointLimits {
  int32_t min;
  int32_t max;
  uint32_t max_rate;  // steps/s
};

// What a bring-up command did, so the protocol can keep its state right.
enum class BringUpResult : uint8_t {
  kUnknown,    // not a bring-up command
  kDone,       // handled; it printed its own reply
  kDoneMoved,  // handled and moved motors outside the step generator: position is lost
};

// A STEP or DIR pin as GPIO register + bit, so the interrupt can set it with one store
// instead of digitalWrite().
struct FastPin {
  GPIO_TypeDef* port;
  uint32_t bit;
};

// The step generator and segment queue.
//
// The main loop puts segments in the queue; tick(), called from the timer interrupt kTickHz
// times a second, plays them. Each segment moves every joint in a straight line to its
// target over a whole number of ticks, using Bresenham's line algorithm, so every joint lands
// exactly on its target at exactly the end of the segment.
//
// Thread rules (one interrupt, one main loop, no locks):
// - The queue is single producer (main loop) / single consumer (tick).
// - tick() only changes the state out of RUNNING and STOPPING; the main loop only changes
//   it out of IDLE, HOMING and FAULT. A stop is a request that tick() carries out.
// - tick() reports events by counting them; the main loop compares with what it has seen.
class Motion {
 public:
  // ----- interrupt side -----
  void tick();

  // ----- main loop side -----
  State state() const { return state_; }
  void setState(State s) { state_ = s; }  // only out of IDLE, HOMING, FAULT
  bool stopPending() const { return stopRequest_ != kNoStop; }

  uint32_t freeSlots() const { return kQueueSize - (head_ - tail_); }
  bool queueEmpty() const { return head_ == tail_; }
  bool push(const Segment& s);  // false if full
  void markEnd() { ended_ = true; }

  // Ramp down to a stop and report `event` (kEventStopped or kEventWatchdog) when still.
  // While IDLE it only empties the queue.
  void requestStop(Event event);

  int32_t position(int joint) const { return position_[joint]; }
  void setPosition(const int32_t* steps);  // IDLE only
  uint32_t playingSeq() const { return playingSeq_; }

  void setLimits(const JointLimits* limits);  // the ramp-down never steps past these

  // Something else drove the DIR pins (a bring-up command): set them again before the
  // next step. IDLE only.
  void forgetDirections();

  // How many times `event` has happened since boot.
  uint32_t eventCount(Event event) const { return events_[event]; }

 private:
  static const uint8_t kNoStop = 0xFF;

  bool loadNext();
  void runTick();
  void rampTick();
  void startRamp(Event event);
  void finish(Event event);

  volatile State state_ = State::kIdle;

  Segment queue_[kQueueSize];
  volatile uint32_t head_ = 0;  // written by the main loop
  volatile uint32_t tail_ = 0;  // written by tick()
  volatile bool ended_ = false;
  volatile uint8_t stopRequest_ = kNoStop;
  volatile uint32_t events_[kEventCount] = {};

  volatile int32_t position_[kJointCount] = {};
  volatile uint32_t playingSeq_ = 0;
  JointLimits limits_[kJointCount] = {};

  // The segment being played.
  bool active_ = false;
  uint32_t ticks_ = 0;
  uint32_t elapsed_ = 0;
  uint32_t delta_[kJointCount] = {};  // |steps| this segment
  uint32_t error_[kJointCount] = {};  // Bresenham accumulators
  int8_t dir_[kJointCount] = {};      // +1 / -1
  // Speed at the end of the last segment, in 1/2^32 steps per tick, for the ramp down.
  uint64_t lastRate_[kJointCount] = {};

  // Ramp down.
  Event rampEvent_ = kEventStopped;
  uint32_t rampLeft_ = 0;
  uint64_t rampRate_[kJointCount] = {};
  uint64_t rampDecrement_[kJointCount] = {};
  uint64_t rampPhase_[kJointCount] = {};
};

// The serial protocol, v2: docs/control_system_decisions.md, section 4.
class Protocol {
 public:
  // Runs one request line (modified in place) and writes the reply.
  void handleLine(char* line, uint32_t nowMs);

  // Call often from the main loop: sends events and runs the watchdog.
  void poll(uint32_t nowMs);

 private:
  typedef void (Protocol::*Handler)(const char* seq, int argc, char** argv);
  struct Command {
    const char* name;
    Handler handler;
  };
  static const Command kCommands[];

  void cmdHello(const char* seq, int argc, char** argv);
  void cmdLimits(const char* seq, int argc, char** argv);
  void cmdEnable(const char* seq, int argc, char** argv);
  void cmdDisable(const char* seq, int argc, char** argv);
  void cmdHome(const char* seq, int argc, char** argv);
  void cmdSetpos(const char* seq, int argc, char** argv);
  void cmdSeg(const char* seq, int argc, char** argv);
  void cmdGo(const char* seq, int argc, char** argv);
  void cmdEnd(const char* seq, int argc, char** argv);
  void cmdStop(const char* seq, int argc, char** argv);
  void cmdStatus(const char* seq, int argc, char** argv);
  void cmdClear(const char* seq, int argc, char** argv);
  void cmdPing(const char* seq, int argc, char** argv);

  void ok(const char* seq, const char* extra = "");
  void err(const char* seq, const char* code, const char* message = "");
  bool needState(const char* seq, bool idle, bool running, bool fault);
  bool needReady(const char* seq);
  bool hasLimits() const;
  State visibleState() const;

  bool limitsSet_[kJointCount] = {};
  JointLimits limits_[kJointCount] = {};
  bool homed_ = false;
  bool enabled_ = false;
  int32_t lastTarget_[kJointCount] = {};  // end of the last queued segment
  uint32_t usRemainder_ = 0;              // segment time not yet turned into ticks
  uint32_t lastRxMs_ = 0;
  uint32_t seenEvents_[kEventCount] = {};
};

// ----- Globals -----
Motion motion;
Protocol protocol;
// Created in setup(): its constructor sets up the timer, which must wait for the clocks.
HardwareTimer* stepTimer = nullptr;

FastPin stepPins[kDriverCount];
FastPin dirPins[kDriverCount];
uint32_t pulseCycles = 0;            // CPU cycles a STEP pin stays high
volatile uint32_t tickMaxCycles = 0;  // longest step interrupt since boot (INFO)

// Only the entry matching each driver's type is set.
TMC5160Stepper* spiDrivers[kDriverCount] = {};
TMC2209Stepper* uartDrivers[kDriverCount] = {};
SoftwareSerial* uartPorts[kDriverCount] = {};
bool driverEnabled[kDriverCount] = {};

char line[kMaxLine + 1];
size_t lineLength = 0;
bool lineTooLong = false;

// Defined in section 4. Declared here so Arduino's generated prototype keeps C linkage.
extern "C" void SystemClock_Config(void);

// Writes formatted text to the USB serial port.
void say(const char* format, ...) __attribute__((format(printf, 1, 2)));
void say(const char* format, ...) {
  char buffer[256];
  va_list args;
  va_start(args, format);
  int n = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  if (n > 0) {
    Serial.write(reinterpret_cast<const uint8_t*>(buffer),
                 n < static_cast<int>(sizeof(buffer)) ? n : sizeof(buffer) - 1);
  }
}

// =============================================================================================
// 4. Hardware
// =============================================================================================

// Disables every driver slot, turns heaters and fans off, and sets up the endstop inputs.
// First thing in setup().
void makeSafe() {
  for (const DriverSlotPins& slot : kSlotPins) {
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

FastPin fastPin(uint32_t pin) {
  FastPin p = {digitalPinToPort(pin), STM_GPIO_PIN(digitalPinToPinName(pin))};
  return p;
}

inline void setPin(const FastPin& p) { p.port->BSRR = p.bit; }
inline void clearPin(const FastPin& p) { p.port->BSRR = p.bit << 16; }

inline uint32_t cycleCount() { return DWT->CYCCNT; }

// Sets up the step/dir/CS pins and the SPI and UART links. Does not talk to the drivers:
// they only answer when motor power is on.
void beginDrivers() {
  SPI.setSCLK(kSpiSck);
  SPI.setMISO(kSpiMiso);
  SPI.setMOSI(kSpiMosi);

  for (size_t i = 0; i < kDriverCount; ++i) {
    const DriverConfig& c = kDrivers[i];
    const DriverSlotPins& p = kSlotPins[c.slot];
    digitalWrite(p.step, LOW);
    pinMode(p.step, OUTPUT);
    pinMode(p.dir, OUTPUT);
    stepPins[i] = fastPin(p.step);
    dirPins[i] = fastPin(p.dir);

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

  // The CPU cycle counter times the step pulse and the step interrupt.
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  pulseCycles = SystemCoreClock / 1000000 * kPulseNs / 1000;
}

uint8_t configure5160(TMC5160Stepper& d, const DriverConfig& c) {
  d.begin();
  d.toff(3);
  d.blank_time(24);
  d.en_pwm_mode(false);  // SpreadCycle (D-37)
  d.rms_current(c.run_current_ma);
  d.microsteps(c.microsteps);
  d.intpol(true);  // smooth each step into 256 microsteps (D-37)
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
  d.en_spreadCycle(false);  // StealthChop (D-37)
  d.pwm_autoscale(true);
  d.rms_current(c.run_current_ma);
  d.microsteps(c.microsteps);
  d.intpol(true);  // smooth each step into 256 microsteps (D-37)
  return d.version();
}

// Writes the settings to every driver and checks that each one answers with its version.
// Prints one line per driver. Returns true if all answered.
// Drivers lose their settings when motor power goes off, so run this again after.
bool configureDrivers() {
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
    say("DRIVER %s slot=%u type=%s version=0x%02X %s\n", c.name, c.slot,
        c.type == DriverType::TMC5160 ? "TMC5160" : "TMC2209", version,
        ok ? "ok" : "NO ANSWER (motor power off, wrong jumpers, or wrong slot?)");
  }
  return allOk;
}

// ENABLE / DISABLE switch every driver at once.
void setDriversEnabled(bool on) {
  for (size_t d = 0; d < kDriverCount; ++d) {
    digitalWrite(kSlotPins[kDrivers[d].slot].enable, on ? LOW : HIGH);
    driverEnabled[d] = on;
  }
}

// Called from the step interrupt (and SPIN). Always before the stepPulse() that uses it.
void setDirection(int joint, bool forward) {
  for (size_t d = 0; d < kDriverCount; ++d) {
    if (kDrivers[d].joint == joint) {
      if (forward != kDrivers[d].invert_dir) {
        setPin(dirPins[d]);
      } else {
        clearPin(dirPins[d]);
      }
    }
  }
}

// One step to every joint whose bit is set in `mask`.
void stepPulse(uint32_t mask) {
  for (size_t d = 0; d < kDriverCount; ++d) {
    if (mask & (1U << kDrivers[d].joint)) {
      setPin(stepPins[d]);
    }
  }
  uint32_t start = cycleCount();
  while (cycleCount() - start < pulseCycles) {
  }
  for (size_t d = 0; d < kDriverCount; ++d) {
    if (mask & (1U << kDrivers[d].joint)) {
      clearPin(stepPins[d]);
    }
  }
}

// Clock setup for the Octopus's 12 MHz crystal. Replaces the generic F446ZE variant's
// SystemClock_Config (declared weak), which runs from the internal 16 MHz oscillator.
//
//   12 MHz HSE / 6 = 2 MHz  ->  x180 = 360 MHz VCO  ->  /2 = 180 MHz CPU (the chip's maximum)
//   AHB 180 MHz, APB1 45 MHz, APB2 90 MHz (timer clocks 90 / 180 MHz)
//   USB 48 MHz from the second PLL (PLLSAI): 2 MHz x96 = 192 MHz VCO, /4 = 48 MHz
//
// The INFO command prints the resulting frequency; it should read sysclk=180000000.
extern "C" void SystemClock_Config(void) {
  RCC_OscInitTypeDef osc = {};
  RCC_ClkInitTypeDef clk = {};
  RCC_PeriphCLKInitTypeDef periph = {};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;  // a crystal, not an external clock (no bypass)
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLM = 6;
  osc.PLL.PLLN = 180;
  osc.PLL.PLLP = RCC_PLLP_DIV2;
  osc.PLL.PLLQ = 8;  // unused (USB runs from PLLSAI)
  osc.PLL.PLLR = 2;  // unused
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
    Error_Handler();
  }

  // 180 MHz needs the regulator's over-drive mode (above 168 MHz).
  if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
    Error_Handler();
  }

  clk.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV4;
  clk.APB2CLKDivider = RCC_HCLK_DIV2;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5) != HAL_OK) {  // 5 wait states at 180 MHz, 3.3 V
    Error_Handler();
  }

  periph.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
  periph.PLLSAI.PLLSAIM = 6;
  periph.PLLSAI.PLLSAIN = 96;
  periph.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
  periph.PLLSAI.PLLSAIQ = 2;  // unused
  periph.PLLSAIDivQ = 1;      // unused
  periph.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLSAIP;
  if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK) {
    Error_Handler();
  }
}

// =============================================================================================
// 5. Motion
// =============================================================================================

// Publish a queue slot only after its contents are written (and read it only after).
inline void barrier() { __sync_synchronize(); }

bool Motion::push(const Segment& s) {
  if (head_ - tail_ >= kQueueSize) {
    return false;
  }
  queue_[head_ % kQueueSize] = s;
  barrier();
  head_ = head_ + 1;
  return true;
}

void Motion::requestStop(Event event) {
  if (state_ == State::kIdle) {
    // tick() leaves the queue alone while IDLE, so it's safe to empty it here.
    tail_ = head_;
    ended_ = false;
  } else {
    stopRequest_ = event;
  }
}

void Motion::setPosition(const int32_t* steps) {
  for (int j = 0; j < kJointCount; ++j) {
    position_[j] = steps[j];
  }
}

void Motion::setLimits(const JointLimits* limits) {
  for (int j = 0; j < kJointCount; ++j) {
    limits_[j] = limits[j];
  }
}

void Motion::forgetDirections() {
  for (int j = 0; j < kJointCount; ++j) {
    dir_[j] = 0;
  }
}

void Motion::tick() {
  // Take a stop request in any state, so a stale one can't fire after the next GO.
  uint8_t stop = stopRequest_;
  if (stop != kNoStop) {
    stopRequest_ = kNoStop;
    if (state_ == State::kRunning) {
      startRamp(static_cast<Event>(stop));
    } else if (state_ != State::kStopping) {
      // Finished on its own in the same moment; still answer the STOP.
      events_[stop] = events_[stop] + 1;
    }
  }
  if (state_ == State::kRunning) {
    runTick();
  } else if (state_ == State::kStopping) {
    rampTick();
  }
}

bool Motion::loadNext() {
  if (head_ == tail_) {
    if (ended_) {
      ended_ = false;
      finish(kEventDone);
    } else {
      startRamp(kEventUnderrun);
    }
    return false;
  }
  barrier();
  const Segment& s = queue_[tail_ % kQueueSize];
  ticks_ = s.ticks;
  elapsed_ = 0;
  playingSeq_ = s.seq;
  for (int j = 0; j < kJointCount; ++j) {
    int32_t d = s.target[j] - position_[j];
    if (d != 0) {
      int8_t dir = d > 0 ? 1 : -1;
      if (dir != dir_[j]) {
        setDirection(j, dir > 0);
        dir_[j] = dir;
      }
    }
    delta_[j] = static_cast<uint32_t>(d >= 0 ? d : -d);
    error_[j] = ticks_ / 2;  // centres the steps in their tick windows
    lastRate_[j] = (static_cast<uint64_t>(delta_[j]) << 32) / ticks_;
  }
  barrier();
  tail_ = tail_ + 1;
  active_ = true;
  return true;
}

void Motion::runTick() {
  if (!active_ && !loadNext()) {
    return;
  }
  uint32_t mask = 0;
  for (int j = 0; j < kJointCount; ++j) {
    error_[j] += delta_[j];
    if (error_[j] >= ticks_) {
      error_[j] -= ticks_;
      position_[j] = position_[j] + dir_[j];
      mask |= 1U << j;
    }
  }
  if (mask) {
    stepPulse(mask);
  }
  if (++elapsed_ >= ticks_) {
    active_ = false;  // the next tick loads the next segment, so no tick is lost
  }
}

void Motion::startRamp(Event event) {
  // Each joint keeps going the way it was, slowing linearly to 0 over kRampTicks: an
  // abrupt stop at speed would make the motors skip steps.
  for (int j = 0; j < kJointCount; ++j) {
    rampRate_[j] = lastRate_[j];
    rampDecrement_[j] = lastRate_[j] / kRampTicks;
    rampPhase_[j] = 0;
    lastRate_[j] = 0;
  }
  active_ = false;
  tail_ = head_;
  ended_ = false;
  playingSeq_ = 0;
  rampEvent_ = event;
  rampLeft_ = kRampTicks;
  state_ = State::kStopping;
}

void Motion::rampTick() {
  uint32_t mask = 0;
  for (int j = 0; j < kJointCount; ++j) {
    uint64_t rate = rampRate_[j];
    if (rate == 0) {
      continue;
    }
    rampPhase_[j] += rate;
    if (rampPhase_[j] >= kOneStep) {
      rampPhase_[j] -= kOneStep;
      int32_t next = position_[j] + dir_[j];
      if (next >= limits_[j].min && next <= limits_[j].max) {  // never past a soft limit
        position_[j] = next;
        mask |= 1U << j;
      }
    }
    rampRate_[j] = rate > rampDecrement_[j] ? rate - rampDecrement_[j] : 0;
  }
  if (mask) {
    stepPulse(mask);
  }
  if (--rampLeft_ == 0) {
    // A SEG the main loop added just as the ramp started would still be queued.
    tail_ = head_;
    finish(rampEvent_);
  }
}

void Motion::finish(Event event) {
  active_ = false;
  playingSeq_ = 0;
  for (int j = 0; j < kJointCount; ++j) {
    lastRate_[j] = 0;
  }
  state_ = State::kIdle;
  events_[event] = events_[event] + 1;
}

// The step interrupt, kTickHz times a second.
void onStepTimer() {
  uint32_t start = cycleCount();
  motion.tick();
  uint32_t cycles = cycleCount() - start;
  if (cycles > tickMaxCycles) {
    tickMaxCycles = cycles;
  }
}

// =============================================================================================
// 6. Protocol
// =============================================================================================

BringUpResult bringUpCommand(const char* seq, const char* command, int argc, char** argv);

const char* stateName(State s) {
  switch (s) {
    case State::kIdle: return "IDLE";
    case State::kRunning: return "RUNNING";
    case State::kStopping: return "STOPPING";
    case State::kHoming: return "HOMING";
    case State::kFault: return "FAULT";
  }
  return "?";
}

bool isNumber(const char* s) {
  if (*s == '-' || *s == '+') {
    ++s;
  }
  if (!*s) {
    return false;
  }
  for (; *s; ++s) {
    if (!isdigit(static_cast<unsigned char>(*s))) {
      return false;
    }
  }
  return true;
}

// Parses `count` whole numbers that fit in an int32. False if any doesn't.
bool parseInts(char** argv, int count, int32_t* values) {
  for (int i = 0; i < count; ++i) {
    if (!isNumber(argv[i])) {
      return false;
    }
    errno = 0;
    long long v = strtoll(argv[i], nullptr, 10);
    if (errno || v < INT32_MIN || v > INT32_MAX) {
      return false;
    }
    values[i] = static_cast<int32_t>(v);
  }
  return true;
}

// Index into kJoints, or -1.
int parseJoint(const char* name) {
  for (int j = 0; j < kJointCount; ++j) {
    if (strcmp(name, kJoints[j].name) == 0) {
      return j;
    }
  }
  return -1;
}

const Protocol::Command Protocol::kCommands[] = {
    {"HELLO", &Protocol::cmdHello},   {"LIMITS", &Protocol::cmdLimits},
    {"ENABLE", &Protocol::cmdEnable}, {"DISABLE", &Protocol::cmdDisable},
    {"HOME", &Protocol::cmdHome},     {"SETPOS", &Protocol::cmdSetpos},
    {"SEG", &Protocol::cmdSeg},       {"GO", &Protocol::cmdGo},
    {"END", &Protocol::cmdEnd},       {"STOP", &Protocol::cmdStop},
    {"STATUS", &Protocol::cmdStatus}, {"CLEAR", &Protocol::cmdClear},
    {"PING", &Protocol::cmdPing},
};

void Protocol::handleLine(char* line, uint32_t nowMs) {
  lastRxMs_ = nowMs;  // any line feeds the watchdog
  for (char* p = line; *p; ++p) {
    *p = static_cast<char>(toupper(static_cast<unsigned char>(*p)));
  }
  char* tokens[kMaxTokens];
  int count = 0;
  for (char* t = strtok(line, " \t\r\n"); t; t = strtok(nullptr, " \t\r\n")) {
    if (count == kMaxTokens) {
      err("-", "BAD_ARGS", "too many arguments");
      return;
    }
    tokens[count++] = t;
  }
  if (count == 0) {
    return;
  }
  const char* seq = "-";
  int first = 0;
  if (isdigit(static_cast<unsigned char>(tokens[0][0])) && isNumber(tokens[0])) {
    seq = tokens[0];
    first = 1;
  }
  if (first == count) {
    err(seq, "BAD_CMD", "no command");
    return;
  }
  const char* command = tokens[first];
  int argc = count - first - 1;
  char** argv = tokens + first + 1;

  for (const Command& c : kCommands) {
    if (strcmp(command, c.name) == 0) {
      (this->*c.handler)(seq, argc, argv);
      return;
    }
  }
  if (motion.state() == State::kIdle) {
    BringUpResult r = bringUpCommand(seq, command, argc, argv);
    if (r != BringUpResult::kUnknown) {
      motion.forgetDirections();
      if (r == BringUpResult::kDoneMoved) {
        homed_ = false;
      }
      return;
    }
  }
  err(seq, "BAD_CMD", "unknown command, or not allowed while moving");
}

void Protocol::poll(uint32_t nowMs) {
  if (motion.state() == State::kRunning && !motion.stopPending() &&
      nowMs - lastRxMs_ > kWatchdogMs) {
    motion.requestStop(kEventWatchdog);
  }
  for (int e = 0; e < kEventCount; ++e) {
    uint32_t count = motion.eventCount(static_cast<Event>(e));
    while (seenEvents_[e] != count) {
      ++seenEvents_[e];
      say("! %s\n", kEventNames[e]);
    }
  }
}

// ----- helpers -----

void Protocol::ok(const char* seq, const char* extra) {
  say("%s OK q=%lu%s%s\n", seq, static_cast<unsigned long>(motion.freeSlots()),
      *extra ? " " : "", extra);
}

void Protocol::err(const char* seq, const char* code, const char* message) {
  say("%s ERR %s q=%lu%s%s\n", seq, code, static_cast<unsigned long>(motion.freeSlots()),
      *message ? " " : "", message);
}

State Protocol::visibleState() const {
  State s = motion.state();
  return (s == State::kRunning && motion.stopPending()) ? State::kStopping : s;
}

bool Protocol::needState(const char* seq, bool idle, bool running, bool fault) {
  State s = visibleState();
  if ((s == State::kIdle && idle) || (s == State::kRunning && running) ||
      (s == State::kFault && fault)) {
    return true;
  }
  char message[40];
  snprintf(message, sizeof(message), "not allowed while %s", stateName(s));
  err(seq, "BAD_STATE", message);
  return false;
}

bool Protocol::hasLimits() const {
  for (bool set : limitsSet_) {
    if (!set) {
      return false;
    }
  }
  return true;
}

bool Protocol::needReady(const char* seq) {
  if (!hasLimits()) {
    err(seq, "NO_LIMITS", "send LIMITS for every joint first");
  } else if (!homed_) {
    err(seq, "NOT_HOMED", "home first");
  } else if (!enabled_) {
    err(seq, "DISABLED", "drivers are off");
  } else {
    return true;
  }
  return false;
}

// ----- commands -----

void Protocol::cmdHello(const char* seq, int, char**) {
  // A new session: stop whatever the last one left running, and forget its limits.
  if (motion.state() == State::kRunning || motion.state() == State::kIdle) {
    motion.requestStop(kEventStopped);
  }
  for (bool& set : limitsSet_) {
    set = false;
  }
  char extra[80];
  snprintf(extra, sizeof(extra), "proto=%d fw=%s queue=%lu max_rate=%lu", kProtocolVersion,
           kFirmwareVersion, static_cast<unsigned long>(kQueueSize),
           static_cast<unsigned long>(kMaxRate));
  ok(seq, extra);
}

void Protocol::cmdLimits(const char* seq, int argc, char** argv) {
  if (!needState(seq, true, false, true)) {
    return;
  }
  int joint = argc == 4 ? parseJoint(argv[0]) : -1;
  int32_t v[3];
  if (joint < 0 || !parseInts(argv + 1, 3, v)) {
    err(seq, "BAD_ARGS", "LIMITS <joint> <min> <max> <max_rate>");
    return;
  }
  if (v[0] > v[1] || v[2] <= 0 || static_cast<uint32_t>(v[2]) > kMaxRate) {
    char message[64];
    snprintf(message, sizeof(message), "need min <= max and 0 < max_rate <= %lu",
             static_cast<unsigned long>(kMaxRate));
    err(seq, "BAD_ARGS", message);
    return;
  }
  limits_[joint] = {v[0], v[1], static_cast<uint32_t>(v[2])};
  limitsSet_[joint] = true;
  if (hasLimits()) {
    motion.setLimits(limits_);
  }
  ok(seq);
}

void Protocol::cmdEnable(const char* seq, int, char**) {
  if (!needState(seq, true, false, true)) {
    return;
  }
  setDriversEnabled(true);
  enabled_ = true;
  ok(seq);
}

void Protocol::cmdDisable(const char* seq, int, char**) {
  if (!needState(seq, true, false, true)) {
    return;
  }
  setDriversEnabled(false);
  enabled_ = false;
  homed_ = false;  // unpowered motors can be moved, and Z falls (D-31)
  ok(seq);
}

void Protocol::cmdHome(const char* seq, int, char**) {
  err(seq, "NOT_IMPLEMENTED", "homing is build step 5; use SETPOS for bench tests");
}

void Protocol::cmdSetpos(const char* seq, int argc, char** argv) {
  if (!needState(seq, true, false, false)) {
    return;
  }
  int32_t steps[kJointCount];
  if (argc != kJointCount || !parseInts(argv, kJointCount, steps)) {
    char message[32];
    snprintf(message, sizeof(message), "SETPOS <%d positions>", kJointCount);
    err(seq, "BAD_ARGS", message);
    return;
  }
  motion.setPosition(steps);
  homed_ = true;
  ok(seq);
}

void Protocol::cmdSeg(const char* seq, int argc, char** argv) {
  if (!needState(seq, true, true, false) || !needReady(seq)) {
    return;
  }
  int32_t v[1 + kJointCount];
  if (argc != 1 + kJointCount || !parseInts(argv, 1 + kJointCount, v)) {
    char message[32];
    snprintf(message, sizeof(message), "SEG <dur_us> <%d targets>", kJointCount);
    err(seq, "BAD_ARGS", message);
    return;
  }
  if (v[0] < static_cast<int32_t>(kMinSegmentUs) || v[0] > static_cast<int32_t>(kMaxSegmentUs)) {
    err(seq, "BAD_ARGS", "dur_us must be 1000..1000000");
    return;
  }
  if (motion.freeSlots() == 0) {
    err(seq, "QUEUE_FULL");
    return;
  }
  uint32_t dur = static_cast<uint32_t>(v[0]);
  const int32_t* targets = v + 1;
  bool fresh = motion.state() == State::kIdle && motion.queueEmpty();
  char message[64];
  for (int j = 0; j < kJointCount; ++j) {
    int32_t from = fresh ? motion.position(j) : lastTarget_[j];
    const JointLimits& l = limits_[j];
    if (targets[j] < l.min || targets[j] > l.max) {
      snprintf(message, sizeof(message), "%s %ld not in [%ld, %ld]", kJoints[j].name,
               static_cast<long>(targets[j]), static_cast<long>(l.min), static_cast<long>(l.max));
      err(seq, "OUT_OF_LIMITS", message);
      return;
    }
    uint64_t steps = static_cast<uint64_t>(llabs(static_cast<int64_t>(targets[j]) - from));
    if (steps * 1000000ULL > static_cast<uint64_t>(l.max_rate) * dur) {
      snprintf(message, sizeof(message), "%s needs %lu steps/s, max %lu", kJoints[j].name,
               static_cast<unsigned long>(steps * 1000000ULL / dur),
               static_cast<unsigned long>(l.max_rate));
      err(seq, "TOO_FAST", message);
      return;
    }
  }

  // Whole ticks only; carry the leftover microseconds into the next segment so a long
  // path doesn't drift.
  if (fresh) {
    usRemainder_ = 0;
  }
  uint32_t total = dur + usRemainder_;
  Segment s;
  s.ticks = total / kUsPerTick;
  usRemainder_ = total % kUsPerTick;
  s.seq = seq[0] == '-' ? 0 : static_cast<uint32_t>(strtoul(seq, nullptr, 10));
  for (int j = 0; j < kJointCount; ++j) {
    s.target[j] = targets[j];
  }
  motion.push(s);
  for (int j = 0; j < kJointCount; ++j) {
    lastTarget_[j] = targets[j];
  }
  ok(seq);
}

void Protocol::cmdGo(const char* seq, int, char**) {
  if (!needState(seq, true, true, false) || !needReady(seq)) {
    return;
  }
  if (motion.state() == State::kIdle) {
    if (motion.queueEmpty()) {
      err(seq, "BAD_STATE", "queue is empty");
      return;
    }
    motion.setState(State::kRunning);
  }
  ok(seq);
}

void Protocol::cmdEnd(const char* seq, int, char**) {
  if (!needState(seq, true, true, false)) {
    return;
  }
  motion.markEnd();
  ok(seq);
}

void Protocol::cmdStop(const char* seq, int, char**) {
  // Never refused.
  State s = motion.state();
  motion.requestStop(kEventStopped);
  ok(seq);
  if (s == State::kIdle) {
    say("! STOPPED\n");  // already still
  }
}

void Protocol::cmdStatus(const char* seq, int, char**) {
  char extra[200];
  int n = snprintf(extra, sizeof(extra), "state=%s homed=%d enabled=%d limits=%d pos=",
                   stateName(visibleState()), homed_, enabled_, hasLimits());
  for (int j = 0; j < kJointCount; ++j) {
    n += snprintf(extra + n, sizeof(extra) - n, j ? ",%ld" : "%ld",
                  static_cast<long>(motion.position(j)));
  }
  snprintf(extra + n, sizeof(extra) - n, " seg=%lu",
           static_cast<unsigned long>(motion.playingSeq()));
  ok(seq, extra);
}

void Protocol::cmdClear(const char* seq, int, char**) {
  if (motion.state() == State::kFault) {
    motion.setState(State::kIdle);
  }
  ok(seq);
}

void Protocol::cmdPing(const char* seq, int, char**) { ok(seq); }

// =============================================================================================
// 7. Bring-up commands (only while IDLE)
// =============================================================================================

const char* const kHelp =
    "Protocol commands (docs/control_system_decisions.md, section 4):\n"
    "  HELLO, LIMITS <joint> <min> <max> <max_rate>, ENABLE, DISABLE, HOME,\n"
    "  SETPOS <6 steps>, SEG <dur_us> <6 steps>, GO, END, STOP, STATUS, CLEAR, PING\n"
    "Bring-up commands (IDLE only):\n"
    "  INFO                         firmware version, CPU clock, and the longest step\n"
    "                               interrupt since boot (tick_max) vs its budget\n"
    "  DRIVERS                      configure all drivers and check they answer\n"
    "                               (needs motor power; run again after it's cycled)\n"
    "  SPIN <joint> <steps> [rate]  move one joint at a constant rate (default 400\n"
    "                               steps/s); ENABLE first. Any input aborts.\n"
    "  ENDSTOPS                     raw endstop input levels\n"
    "Joints: THETA Z R YAW PITCH FOCUS. A leading number is the sequence number (optional).\n";

// Blocking, constant speed, no acceleration. Moves the motor outside the step generator,
// so the board forgets its position (homed=0).
BringUpResult spin(const char* seq, unsigned long q, int argc, char** argv) {
  int joint = argc >= 2 ? parseJoint(argv[0]) : -1;
  if (joint < 0) {
    say("%s ERR BAD_ARGS q=%lu SPIN <joint> <steps> [rate]\n", seq, q);
    return BringUpResult::kDone;
  }
  long steps = strtol(argv[1], nullptr, 10);
  long rate = argc >= 3 ? strtol(argv[2], nullptr, 10) : kDefaultSpinRate;
  if (rate <= 0 || rate > kMaxSpinRate) {
    say("%s ERR BAD_ARGS q=%lu rate must be 1..%ld steps/s\n", seq, q, kMaxSpinRate);
    return BringUpResult::kDone;
  }
  for (size_t d = 0; d < kDriverCount; ++d) {
    if (kDrivers[d].joint == joint && !driverEnabled[d]) {
      say("%s ERR DISABLED q=%lu ENABLE first\n", seq, q);
      return BringUpResult::kDone;
    }
  }

  setDirection(joint, steps >= 0);
  delayMicroseconds(5);  // direction setup time before the first step
  long total = labs(steps);
  uint32_t interval = 1000000UL / rate;
  uint32_t next = micros();
  long done = 0;
  for (; done < total && !Serial.available(); ++done) {
    while (static_cast<int32_t>(micros() - next) < 0) {
    }
    next += interval;
    stepPulse(1U << joint);
  }
  if (done < total) {
    say("%s ERR ABORTED q=%lu after %ld of %ld steps\n", seq, q, done, total);
  } else {
    say("%s OK q=%lu %s moved %ld steps\n", seq, q, kJoints[joint].name, steps);
  }
  return BringUpResult::kDoneMoved;
}

BringUpResult bringUpCommand(const char* seq, const char* command, int argc, char** argv) {
  unsigned long q = motion.freeSlots();
  if (strcmp(command, "HELP") == 0) {
    Serial.print(kHelp);
    say("%s OK q=%lu\n", seq, q);
  } else if (strcmp(command, "INFO") == 0) {
    say("%s OK q=%lu firmware=%s sysclk=%lu tick_max=%lu tick_budget=%lu\n", seq, q,
        kFirmwareVersion, static_cast<unsigned long>(HAL_RCC_GetSysClockFreq()),
        static_cast<unsigned long>(tickMaxCycles),
        static_cast<unsigned long>(SystemCoreClock / kTickHz));
  } else if (strcmp(command, "DRIVERS") == 0) {
    if (configureDrivers()) {
      say("%s OK q=%lu all drivers answered\n", seq, q);
    } else {
      say("%s ERR NO_ANSWER q=%lu some drivers did not answer\n", seq, q);
    }
  } else if (strcmp(command, "ENDSTOPS") == 0) {
    say("%s OK q=%lu", seq, q);
    for (const JointConfig& j : kJoints) {
      say(" %s=%d", j.name, digitalRead(kEndstopPins[j.endstop]));
    }
    say("\n");
  } else if (strcmp(command, "SPIN") == 0) {
    return spin(seq, q, argc, argv);
  } else {
    return BringUpResult::kUnknown;
  }
  return BringUpResult::kDone;
}

// =============================================================================================
// 8. setup / loop
// =============================================================================================

// Collects USB serial bytes into lines and hands each to the protocol.
void readSerial() {
  while (Serial.available()) {
    char c = static_cast<char>(Serial.read());
    if (c == '\n' || c == '\r') {
      if (lineTooLong) {
        say("- ERR BAD_ARGS q=%lu line too long\n", static_cast<unsigned long>(motion.freeSlots()));
      } else if (lineLength > 0) {
        line[lineLength] = '\0';
        protocol.handleLine(line, millis());
      }
      lineLength = 0;
      lineTooLong = false;
    } else if (lineLength < static_cast<size_t>(kMaxLine)) {
      line[lineLength++] = c;
    } else {
      lineTooLong = true;
    }
  }
}

void setup() {
  makeSafe();            // drivers disabled, heaters and fans off, before anything else
  Serial.begin(115200);  // USB serial: the baud rate is ignored
  beginDrivers();

  // Above USB (priority 1), so serial traffic never delays a step.
  stepTimer = new HardwareTimer(TIM6);
  stepTimer->setOverflow(kTickHz, HERTZ_FORMAT);
  stepTimer->setInterruptPriority(0, 0);
  stepTimer->attachInterrupt(onStepTimer);
  stepTimer->resume();
}

void loop() {
  readSerial();
  protocol.poll(millis());
}
