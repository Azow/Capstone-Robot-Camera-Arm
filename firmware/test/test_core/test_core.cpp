// Unit tests for lib/arm_core, run on the PC: pio test -e native
//
// tick() is called directly, so "time" here is a tick count (kTickHz per second).

#include <motion.h>
#include <protocol.h>
#include <stdlib.h>
#include <string.h>
#include <unity.h>

#include <string>

using namespace arm;

namespace {

struct TestHal : Hal {
  int32_t steps[kJointCount] = {};  // signed step count from the pulses and DIR pins
  bool forward[kJointCount] = {};
  bool dirSet[kJointCount] = {};
  uint32_t pulses = 0;
  bool enabled = false;
  bool doubleStep = false;  // a joint pulsed twice in one tick (impossible by design)

  void setDirection(int joint, bool fwd) override {
    forward[joint] = fwd;
    dirSet[joint] = true;
  }
  void stepPulse(uint32_t mask) override {
    for (int j = 0; j < kJointCount; ++j) {
      if (mask & (1U << j)) {
        TEST_ASSERT_TRUE_MESSAGE(dirSet[j], "stepped before DIR was set");
        steps[j] += forward[j] ? 1 : -1;
        ++pulses;
      }
    }
  }
  void setEnabled(bool on) override { enabled = on; }
  ExtraResult extraCommand(const char* seq, uint32_t, const char* command, int, char**,
                           Output& out) override {
    if (strcmp(command, "SPIN") == 0) {
      out.printf("%s OK spun\n", seq);
      return ExtraResult::kDoneMoved;
    }
    return ExtraResult::kUnknown;
  }
};

struct StringOut : Output {
  std::string text;
  void write(const char* t, size_t n) override { text.append(t, n); }
};

struct Board {
  TestHal hal;
  Motion motion{hal};
  Protocol protocol{hal, motion};
  uint32_t nowMs = 0;
  StringOut events;

  std::string send(const char* line) {
    char buffer[200];
    strncpy(buffer, line, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';
    StringOut out;
    protocol.handleLine(buffer, nowMs, out);
    return out.text;
  }

  // Runs the interrupt for `ticks` ticks, with the main loop polling every millisecond
  // and (if feed) the PC sending PING every 20 ms.
  void run(uint32_t ticks, bool feed = true) {
    for (uint32_t t = 0; t < ticks; ++t) {
      motion.tick();
      if ((t + 1) % (kTickHz / 1000) == 0) {
        ++nowMs;
        if (feed && nowMs % 20 == 0) {
          send("PING");
        }
        protocol.poll(nowMs, events);
      }
    }
  }

  void ready() {
    const char* lines[] = {"HELLO",
                           "LIMITS THETA -8000 8000 4000",
                           "LIMITS Z -60000 0 9000",
                           "LIMITS R -32000 0 9000",
                           "LIMITS YAW -2000 2000 2000",
                           "LIMITS PITCH -1000 1000 2000",
                           "ENABLE",
                           "SETPOS 0 0 0 0 0"};
    for (const char* line : lines) {
      std::string reply = send(line);
      TEST_ASSERT_TRUE_MESSAGE(reply.find(" OK ") != std::string::npos, reply.c_str());
    }
  }
};

bool has(const std::string& text, const char* part) { return text.find(part) != std::string::npos; }

}  // namespace

void setUp() {}
void tearDown() {}

void test_motion_needs_limits_homing_and_enable() {
  Board b;
  TEST_ASSERT_TRUE(has(b.send("1 SEG 20000 0 0 0 0 0"), "1 ERR NO_LIMITS q=64"));
  b.ready();
  b.send("DISABLE");
  TEST_ASSERT_TRUE(has(b.send("2 SEG 20000 0 0 0 0 0"), "NOT_HOMED"));
  b.send("SETPOS 0 0 0 0 0");
  TEST_ASSERT_TRUE(has(b.send("3 SEG 20000 0 0 0 0 0"), "DISABLED"));
}

void test_hello_reply() {
  Board b;
  TEST_ASSERT_EQUAL_STRING("7 OK q=64 proto=1 fw=0.2 queue=64 max_rate=40000\n",
                           b.send("7 hello").c_str());
  TEST_ASSERT_EQUAL_STRING("- OK q=64\n", b.send("ping").c_str());
}

void test_segment_checks() {
  Board b;
  b.ready();
  TEST_ASSERT_TRUE(has(b.send("1 SEG 20000 9000 0 0 0 0"), "OUT_OF_LIMITS"));
  TEST_ASSERT_TRUE(has(b.send("2 SEG 20000 100 0 0 0 0"), "TOO_FAST"));  // 5000 steps/s
  TEST_ASSERT_EQUAL_STRING("3 OK q=63\n", b.send("3 SEG 20000 50 0 0 0 0").c_str());
  // Checked against the previous segment's end, not the current position.
  TEST_ASSERT_TRUE(has(b.send("4 SEG 20000 -40 0 0 0 0"), "TOO_FAST"));
  TEST_ASSERT_TRUE(has(b.send("5 SEG 500 50 0 0 0 0"), "BAD_ARGS"));
  TEST_ASSERT_TRUE(has(b.send("6 SEG 20000 1 2 3"), "BAD_ARGS"));
  TEST_ASSERT_TRUE(has(b.send("7 SEG 20000 x 0 0 0 0"), "BAD_ARGS"));
}

void test_plays_segments_exactly_and_reports_done() {
  Board b;
  b.ready();
  b.send("1 SEG 100000 100 -300 0 7 0");  // 5000 ticks each
  b.send("2 SEG 100000 200 -300 -250 0 -1");
  b.send("3 END");
  b.send("4 GO");
  b.run(2500);
  TEST_ASSERT_TRUE(has(b.send("STATUS"), "state=RUNNING"));
  TEST_ASSERT_TRUE(has(b.send("STATUS"), "seg=1"));
  TEST_ASSERT_INT_WITHIN(1, 50, b.motion.position(0));
  b.run(5000);
  TEST_ASSERT_TRUE(has(b.send("STATUS"), "seg=2"));
  TEST_ASSERT_EQUAL_STRING("", b.events.text.c_str());
  b.run(2550);  // the last tick, one to see the queue is empty, then a main-loop poll
  TEST_ASSERT_EQUAL_STRING("! DONE\n", b.events.text.c_str());
  TEST_ASSERT_TRUE(has(b.send("STATUS"), "state=IDLE homed=1 enabled=1 limits=1 "
                                         "pos=200,-300,-250,0,-1 seg=0"));
  // The pulses really add up to the reported positions.
  int32_t expected[] = {200, -300, -250, 0, -1};
  TEST_ASSERT_EQUAL_INT32_ARRAY(expected, b.hal.steps, kJointCount);
}

void test_steps_are_even_and_never_double() {
  Board b;
  b.ready();
  // 37 steps over 1000 ticks: gaps between steps must be 27 or 28 ticks.
  b.send("1 SEG 20000 37 0 0 0 0");
  b.send("END");
  b.send("GO");
  int32_t last = 0;
  int lastTick = -1;
  int minGap = 1 << 30;
  int maxGap = 0;
  for (int t = 0; t < 1001; ++t) {
    b.motion.tick();
    if (b.hal.steps[0] != last) {
      if (lastTick >= 0) {
        minGap = std::min(minGap, t - lastTick);
        maxGap = std::max(maxGap, t - lastTick);
      }
      last = b.hal.steps[0];
      lastTick = t;
    }
  }
  TEST_ASSERT_EQUAL(37, b.hal.steps[0]);
  TEST_ASSERT_EQUAL(27, minGap);
  TEST_ASSERT_EQUAL(28, maxGap);
}

void test_timing_remainder_carries_over() {
  Board b;
  b.ready();
  // 20010 us is 1000.5 ticks; three of them must take 3001 ticks, not 3000.
  b.send("1 SEG 20010 10 0 0 0 0");
  b.send("2 SEG 20010 20 0 0 0 0");
  b.send("3 SEG 20010 30 0 0 0 0");
  b.send("END");
  b.send("GO");
  for (int t = 0; t < 3001; ++t) {
    b.motion.tick();
  }
  TEST_ASSERT_EQUAL(30, b.motion.position(0));
  TEST_ASSERT_TRUE(b.motion.state() == State::kRunning);  // last segment just ended
  b.motion.tick();
  TEST_ASSERT_TRUE(b.motion.state() == State::kIdle);
}

void test_underrun_ramps_down() {
  Board b;
  b.ready();
  b.send("1 SEG 100000 300 0 0 0 0");  // 3000 steps/s
  b.send("GO");
  b.run(5001);
  TEST_ASSERT_TRUE(has(b.send("STATUS"), "state=STOPPING"));
  b.run(kRampTicks);
  TEST_ASSERT_EQUAL_STRING("! UNDERRUN\n", b.events.text.c_str());
  // From 3000 steps/s to 0 over 0.1 s: about 150 more steps.
  TEST_ASSERT_INT_WITHIN(2, 450, b.motion.position(0));
  TEST_ASSERT_EQUAL(b.motion.position(0), b.hal.steps[0]);
}

void test_ramp_stops_at_soft_limit() {
  Board b;
  b.ready();
  // Run YAW (limit 2000) to 10 steps short of its limit at 1500 steps/s, then let the queue
  // run dry. Ramping down from that speed would take about 75 steps; it must stop at 2000.
  b.send("1 SEG 1000000 0 0 0 1960 0");
  b.send("2 SEG 20000 0 0 0 1990 0");
  b.send("GO");
  b.run(kTickHz + 1000 + 1);
  b.run(kRampTicks);
  TEST_ASSERT_EQUAL(2000, b.motion.position(3));
  TEST_ASSERT_EQUAL(2000, b.hal.steps[3]);
  TEST_ASSERT_EQUAL_STRING("! UNDERRUN\n", b.events.text.c_str());
}

void test_watchdog_stops_a_silent_run() {
  Board b;
  b.ready();
  b.send("1 SEG 1000000 1000 0 0 0 0");
  b.send("GO");
  b.run(kTickHz / 5, false);  // 200 ms of silence: still fine
  TEST_ASSERT_TRUE(has(b.send("STATUS"), "state=RUNNING"));
  b.run(kTickHz * 3 / 10, false);  // 300 ms of silence
  TEST_ASSERT_TRUE(b.motion.state() != State::kRunning);
  b.run(kRampTicks, false);
  TEST_ASSERT_EQUAL_STRING("! WATCHDOG\n", b.events.text.c_str());
}

void test_stop_clears_the_queue() {
  Board b;
  b.ready();
  b.send("1 SEG 100000 50 0 0 0 0");
  b.send("2 SEG 100000 100 0 0 0 0");
  b.send("3 SEG 100000 150 0 0 0 0");
  b.send("GO");
  b.run(2500);
  TEST_ASSERT_EQUAL_STRING("9 OK q=62\n", b.send("9 STOP").c_str());
  TEST_ASSERT_TRUE(has(b.send("STATUS"), "state=STOPPING"));
  TEST_ASSERT_TRUE(has(b.send("SEG 20000 0 0 0 0 0"), "BAD_STATE"));
  b.run(kRampTicks + 10);
  TEST_ASSERT_EQUAL_STRING("! STOPPED\n", b.events.text.c_str());
  TEST_ASSERT_TRUE(has(b.send("STATUS"), "state=IDLE"));
  TEST_ASSERT_TRUE(has(b.send("STATUS"), "q=64"));
  // 25 steps in, then ramping down from 500 steps/s over 0.1 s adds 25: well short of the
  // 150 the queue would have reached.
  TEST_ASSERT_INT_WITHIN(1, 50, b.motion.position(0));
}

void test_stop_while_idle_answers_at_once() {
  Board b;
  b.ready();
  TEST_ASSERT_EQUAL_STRING("5 OK q=64\n! STOPPED\n", b.send("5 STOP").c_str());
}

void test_direction_changes_between_segments() {
  Board b;
  b.ready();
  b.send("SEG 20000 30 0 0 0 0");
  b.send("SEG 20000 0 0 0 0 0");
  b.send("SEG 20000 -30 0 0 0 0");
  b.send("END");
  b.send("GO");
  b.run(3002);
  TEST_ASSERT_EQUAL(-30, b.hal.steps[0]);
  TEST_ASSERT_EQUAL(-30, b.motion.position(0));
}

void test_hardware_command_that_moves_clears_homed() {
  Board b;
  b.ready();
  TEST_ASSERT_EQUAL_STRING("4 OK spun\n", b.send("4 SPIN THETA 100").c_str());
  TEST_ASSERT_TRUE(has(b.send("STATUS"), "homed=0"));
  TEST_ASSERT_TRUE(has(b.send("NOPE"), "BAD_CMD"));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_motion_needs_limits_homing_and_enable);
  RUN_TEST(test_hello_reply);
  RUN_TEST(test_segment_checks);
  RUN_TEST(test_plays_segments_exactly_and_reports_done);
  RUN_TEST(test_steps_are_even_and_never_double);
  RUN_TEST(test_timing_remainder_carries_over);
  RUN_TEST(test_underrun_ramps_down);
  RUN_TEST(test_ramp_stops_at_soft_limit);
  RUN_TEST(test_watchdog_stops_a_silent_run);
  RUN_TEST(test_stop_clears_the_queue);
  RUN_TEST(test_stop_while_idle_answers_at_once);
  RUN_TEST(test_direction_changes_between_segments);
  RUN_TEST(test_hardware_command_that_moves_clears_homed);
  return UNITY_END();
}
