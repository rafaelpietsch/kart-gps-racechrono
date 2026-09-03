// SPDX-License-Identifier: MIT

#include "button.h"

namespace kart {

ButtonDebouncer::ButtonDebouncer(const Config& config) : config_(config) {}

void ButtonDebouncer::configure(const Config& config) {
  config_ = config;
}

void ButtonDebouncer::reset() {
  stablePressed_ = false;
  candidateLevel_ = false;
  hasCandidate_ = false;
  candidateSinceMs_ = 0;
  pressedAtMs_ = 0;
  longPressReported_ = false;
}

uint32_t ButtonDebouncer::heldForMs(uint32_t nowMs) const {
  if (!stablePressed_) {
    return 0;
  }
  return static_cast<uint32_t>(nowMs - pressedAtMs_);
}

ButtonDebouncer::Event ButtonDebouncer::update(bool rawPressed, uint32_t nowMs) {
  if (rawPressed != stablePressed_) {
    if (!hasCandidate_ || candidateLevel_ != rawPressed) {
      // A new level appeared; start timing it rather than acting on it.
      hasCandidate_ = true;
      candidateLevel_ = rawPressed;
      candidateSinceMs_ = nowMs;
    } else if (static_cast<uint32_t>(nowMs - candidateSinceMs_) >= config_.debounceMs) {
      hasCandidate_ = false;
      stablePressed_ = rawPressed;
      if (stablePressed_) {
        pressedAtMs_ = nowMs;
        longPressReported_ = false;
        return Event::kPressed;
      }
      // A press that already reported kLongPress must not also report a short
      // one on release, otherwise a single hold triggers two actions.
      if (!longPressReported_) {
        return Event::kShortPress;
      }
      return Event::kReleased;
    }
  } else {
    hasCandidate_ = false;
  }

  if (stablePressed_ && !longPressReported_ &&
      static_cast<uint32_t>(nowMs - pressedAtMs_) >= config_.longPressMs) {
    longPressReported_ = true;
    return Event::kLongPress;
  }

  return Event::kNone;
}

// --- RateLimiter ------------------------------------------------------------

bool RateLimiter::tryAcquire(uint32_t nowMs) {
  if (primed_ && static_cast<uint32_t>(nowMs - lastMs_) < intervalMs_) {
    return false;
  }
  lastMs_ = nowMs;
  primed_ = true;
  return true;
}

void RateLimiter::reset() {
  primed_ = false;
  lastMs_ = 0;
}

} // namespace kart
