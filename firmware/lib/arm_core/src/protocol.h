#pragma once

#include <stdint.h>

#include "hal.h"
#include "motion.h"

// The serial protocol, v1: docs/control_system_decisions.md, section 4.
// arm_bridge (src/arm_bridge/arm_bridge/protocol.py) is the other end, and the Python fake
// board (fake_board.py) implements the same rules; keep all three in step.

namespace arm {

constexpr int kProtocolVersion = 1;
constexpr const char* kFirmwareVersion = "0.2";
constexpr uint32_t kWatchdogMs = 250;
constexpr uint32_t kMinSegmentUs = 1000;
constexpr uint32_t kMaxSegmentUs = 1000000;
constexpr int kMaxLine = 120;

class Protocol {
 public:
  Protocol(Hal& hal, Motion& motion) : hal_(hal), motion_(motion) {}

  // Runs one request line (modified in place) and writes the reply.
  void handleLine(char* line, uint32_t nowMs, Output& out);

  // Call often from the main loop: sends events and runs the watchdog.
  void poll(uint32_t nowMs, Output& out);

 private:
  using Handler = void (Protocol::*)(const char* seq, int argc, char** argv, Output& out);
  struct Command {
    const char* name;
    Handler handler;
  };
  static const Command kCommands[];

  void cmdHello(const char* seq, int argc, char** argv, Output& out);
  void cmdLimits(const char* seq, int argc, char** argv, Output& out);
  void cmdEnable(const char* seq, int argc, char** argv, Output& out);
  void cmdDisable(const char* seq, int argc, char** argv, Output& out);
  void cmdHome(const char* seq, int argc, char** argv, Output& out);
  void cmdSetpos(const char* seq, int argc, char** argv, Output& out);
  void cmdSeg(const char* seq, int argc, char** argv, Output& out);
  void cmdGo(const char* seq, int argc, char** argv, Output& out);
  void cmdEnd(const char* seq, int argc, char** argv, Output& out);
  void cmdStop(const char* seq, int argc, char** argv, Output& out);
  void cmdStatus(const char* seq, int argc, char** argv, Output& out);
  void cmdClear(const char* seq, int argc, char** argv, Output& out);
  void cmdPing(const char* seq, int argc, char** argv, Output& out);

  void ok(const char* seq, Output& out, const char* extra = "");
  void err(const char* seq, const char* code, Output& out, const char* message = "");
  bool needState(const char* seq, Output& out, bool idle, bool running, bool fault);
  bool needReady(const char* seq, Output& out);
  bool hasLimits() const;
  State visibleState() const;

  Hal& hal_;
  Motion& motion_;
  bool limitsSet_[kJointCount] = {};
  JointLimits limits_[kJointCount] = {};
  bool homed_ = false;
  bool enabled_ = false;
  int32_t lastTarget_[kJointCount] = {};  // end of the last queued segment
  uint32_t usRemainder_ = 0;              // segment time not yet turned into ticks
  uint32_t lastRxMs_ = 0;
  uint32_t seenEvents_[kEventCount] = {};
};

}  // namespace arm
