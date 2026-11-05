// The firmware's protocol and motion code (lib/arm_core) running on the PC.
//
// Serves the protocol on a pseudo-terminal, like the board's USB serial port, and calls the
// step interrupt kTickHz times per second of real time. arm_bridge connects to it exactly
// as it would to the board, so it tests the bridge against the real firmware logic.
//
//   pio run -e sim
//   .pio/build/sim/program --link /tmp/sim_octopus
//
// The motors are imaginary: steps only change the reported positions.

#include <errno.h>
#include <fcntl.h>
#include <pty.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <string>

#include "motion.h"
#include "protocol.h"

namespace {

volatile sig_atomic_t running = 1;

struct SimHal : arm::Hal {
  void setDirection(int, bool) override {}
  void stepPulse(uint32_t) override {}
  void setEnabled(bool) override {}
  arm::ExtraResult extraCommand(const char* seq, uint32_t freeSlots, const char* command, int,
                                char**, arm::Output& out) override {
    if (strcmp(command, "INFO") == 0) {
      out.printf("%s OK q=%lu firmware=%s sim=1\n", seq, static_cast<unsigned long>(freeSlots),
                 arm::kFirmwareVersion);
      return arm::ExtraResult::kDone;
    }
    return arm::ExtraResult::kUnknown;
  }
};

struct FdOutput : arm::Output {
  int fd;
  explicit FdOutput(int f) : fd(f) {}
  void write(const char* text, size_t length) override {
    while (length > 0) {
      ssize_t n = ::write(fd, text, length);
      if (n <= 0) {
        if (n < 0 && errno == EINTR) {
          continue;
        }
        return;  // nobody connected; the board would drop it too
      }
      text += n;
      length -= static_cast<size_t>(n);
    }
  }
};

uint64_t nowUs() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000 + static_cast<uint64_t>(ts.tv_nsec) / 1000;
}

}  // namespace

int main(int argc, char** argv) {
  const char* link = "/tmp/sim_octopus";
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "--link") == 0 && i + 1 < argc) {
      link = argv[++i];
    }
    // Anything else (e.g. --ros-args when started from a launch file) is ignored.
  }

  int master = -1;
  int slave = -1;
  if (openpty(&master, &slave, nullptr, nullptr, nullptr) != 0) {
    perror("openpty");
    return 1;
  }
  termios raw;
  tcgetattr(slave, &raw);
  cfmakeraw(&raw);
  tcsetattr(slave, TCSANOW, &raw);
  // Keep `slave` open so the master doesn't read EIO between client connections.
  unlink(link);
  if (symlink(ttyname(slave), link) != 0) {
    perror("symlink");
    return 1;
  }
  printf("Firmware simulator on %s, linked as %s\n", ttyname(slave), link);
  fflush(stdout);

  signal(SIGINT, [](int) { running = 0; });
  signal(SIGTERM, [](int) { running = 0; });

  SimHal hal;
  arm::Motion motion(hal);
  arm::Protocol protocol(hal, motion);
  FdOutput out(master);

  const uint64_t start = nowUs();
  uint64_t ticksDone = 0;
  std::string line;
  while (running) {
    fd_set readable;
    FD_ZERO(&readable);
    FD_SET(master, &readable);
    timeval wait = {0, 500};
    int ready = select(master + 1, &readable, nullptr, nullptr, &wait);

    // Catch the step interrupt up with real time.
    uint64_t elapsed = nowUs() - start;
    uint64_t due = elapsed * arm::kTickHz / 1000000;
    for (; ticksDone < due; ++ticksDone) {
      motion.tick();
    }
    uint32_t nowMs = static_cast<uint32_t>(elapsed / 1000);

    if (ready > 0) {
      char buffer[512];
      ssize_t n = read(master, buffer, sizeof(buffer));
      for (ssize_t i = 0; i < n; ++i) {
        char c = buffer[i];
        if (c == '\n' || c == '\r') {
          if (!line.empty()) {
            if (line.size() > static_cast<size_t>(arm::kMaxLine)) {
              out.print("- ERR BAD_ARGS q=0 line too long\n");
            } else {
              protocol.handleLine(&line[0], nowMs, out);
            }
            line.clear();
          }
        } else {
          line += c;
        }
      }
    }
    protocol.poll(nowMs, out);
  }
  unlink(link);
  return 0;
}
