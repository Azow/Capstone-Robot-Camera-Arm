#include "motion.h"

namespace arm {

namespace {

constexpr uint64_t kOneStep = 1ULL << 32;  // ramp speeds are in 1/2^32 steps per tick

// Publish a queue slot only after its contents are written (and read it only after).
inline void barrier() { __sync_synchronize(); }

}  // namespace

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
        hal_.setDirection(j, dir > 0);
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
    hal_.stepPulse(mask);
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
    hal_.stepPulse(mask);
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

}  // namespace arm
