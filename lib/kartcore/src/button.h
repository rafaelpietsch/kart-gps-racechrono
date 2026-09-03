// SPDX-License-Identifier: MIT
//
// Debouncing and press classification for the SET button.
//
// A kart is an unforgiving place for a mechanical switch: vibration bounces the
// contacts, and the driver is wearing gloves. The state machine here is fed a
// raw level and a millisecond clock, which makes every timing rule testable
// without any hardware in the loop.

#ifndef KARTGPS_BUTTON_H
#define KARTGPS_BUTTON_H

#include <stdint.h>

namespace kart {

class ButtonDebouncer {
public:
  enum class Event : uint8_t {
    kNone,
    kPressed,    ///< Debounced edge: the button just went down
    kReleased,   ///< Debounced edge: the button just came up
    kShortPress, ///< Released before the long press threshold
    kLongPress,  ///< Held past the threshold, reported without waiting for release
  };

  struct Config {
    uint16_t debounceMs = 25;
    uint16_t longPressMs = 1500;
  };

  ButtonDebouncer() = default;
  explicit ButtonDebouncer(const Config& config);

  void configure(const Config& config);

  /// Feeds one sample of the (already de-inverted) button level.
  /// Call it often -- every loop iteration is fine.
  Event update(bool rawPressed, uint32_t nowMs);

  bool isPressed() const { return stablePressed_; }

  /// How long the button has been held, or 0 when it is up.
  uint32_t heldForMs(uint32_t nowMs) const;

  void reset();

private:
  Config config_;
  bool stablePressed_ = false;
  bool candidateLevel_ = false;
  bool hasCandidate_ = false;
  uint32_t candidateSinceMs_ = 0;
  uint32_t pressedAtMs_ = 0;
  bool longPressReported_ = false;
};

/// Fires at most once per interval, wrap safe across the 49.7 day millisecond
/// rollover.
class RateLimiter {
public:
  explicit RateLimiter(uint32_t intervalMs = 0) : intervalMs_(intervalMs) {}

  void setIntervalMs(uint32_t intervalMs) { intervalMs_ = intervalMs; }
  uint32_t intervalMs() const { return intervalMs_; }

  /// Returns true and records the timestamp when the interval has elapsed.
  bool tryAcquire(uint32_t nowMs);

  void reset();

private:
  uint32_t intervalMs_ = 0;
  uint32_t lastMs_ = 0;
  bool primed_ = false;
};

} // namespace kart

#endif // KARTGPS_BUTTON_H
