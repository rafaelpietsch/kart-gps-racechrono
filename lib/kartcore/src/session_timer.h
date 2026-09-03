// SPDX-License-Identifier: MIT
//
// Monotonic session clock, zeroed by the SET button.
//
// Header only: the whole thing is a subtraction, and keeping it inline lets the
// compiler fold it into the callers on the hot path.

#ifndef KARTGPS_SESSION_TIMER_H
#define KARTGPS_SESSION_TIMER_H

#include <stdint.h>

namespace kart {

/// Time since the last reset, in milliseconds.
///
/// All arithmetic is done on unsigned values so it stays correct across the
/// 32-bit millisecond wrap the ESP32 hits after about 49.7 days of uptime --
/// far beyond a race weekend, but free to get right.
class SessionTimer {
public:
  /// Starts (or restarts) the session at `nowMs`.
  void reset(uint32_t nowMs) {
    startedAtMs_ = nowMs;
    running_ = true;
    resetCount_ = static_cast<uint16_t>(resetCount_ + 1);
  }

  /// Freezes the elapsed time at its current value.
  void stop(uint32_t nowMs) {
    if (running_) {
      frozenMs_ = elapsedMs(nowMs);
      running_ = false;
    }
  }

  uint32_t elapsedMs(uint32_t nowMs) const {
    if (!running_) {
      return frozenMs_;
    }
    return static_cast<uint32_t>(nowMs - startedAtMs_);
  }

  bool isRunning() const { return running_; }

  /// How many times the driver has zeroed the session. Handy in the status
  /// channel to confirm a button press actually landed.
  uint16_t resetCount() const { return resetCount_; }

  void clear() {
    startedAtMs_ = 0;
    frozenMs_ = 0;
    running_ = false;
    resetCount_ = 0;
  }

private:
  uint32_t startedAtMs_ = 0;
  uint32_t frozenMs_ = 0;
  bool running_ = false;
  uint16_t resetCount_ = 0;
};

} // namespace kart

#endif // KARTGPS_SESSION_TIMER_H
