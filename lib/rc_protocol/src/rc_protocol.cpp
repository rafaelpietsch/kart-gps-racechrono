// SPDX-License-Identifier: MIT

#include "rc_protocol.h"

#include <math.h>
#include <string.h>

namespace rc {
namespace {

/// Highest altitude representable by the fine encoding: (0x7FFF / 10) - 500.
constexpr float kFineAltitudeCeilingMeters = 2776.7f;
/// Highest altitude representable at all: 0x7FFF - 500.
constexpr float kCoarseAltitudeCeilingMeters = 32267.0f;
constexpr float kAltitudeFloorMeters = -500.0f;

/// Highest speed representable by the fine encoding: 0x7FFF / 100.
constexpr float kFineSpeedCeilingKph = 327.67f;

constexpr uint16_t kFifteenBitMask = 0x7FFF;
constexpr uint16_t kCoarseFlag = 0x8000;

/// Rounds to the nearest integer and clamps into [lo, hi]. NaN maps to lo.
int32_t roundClamp(float value, int32_t lo, int32_t hi) {
  if (!(value == value)) {
    return lo;
  }
  const float rounded = (value >= 0.0f) ? floorf(value + 0.5f) : ceilf(value - 0.5f);
  if (rounded <= static_cast<float>(lo)) {
    return lo;
  }
  if (rounded >= static_cast<float>(hi)) {
    return hi;
  }
  return static_cast<int32_t>(rounded);
}

void writeBigEndian16(uint8_t* out, uint16_t value) {
  out[0] = static_cast<uint8_t>(value >> 8);
  out[1] = static_cast<uint8_t>(value);
}

void writeBigEndian32(uint8_t* out, uint32_t value) {
  out[0] = static_cast<uint8_t>(value >> 24);
  out[1] = static_cast<uint8_t>(value >> 16);
  out[2] = static_cast<uint8_t>(value >> 8);
  out[3] = static_cast<uint8_t>(value);
}

uint32_t readBigEndian32(const uint8_t* in) {
  return (static_cast<uint32_t>(in[0]) << 24) | (static_cast<uint32_t>(in[1]) << 16) |
         (static_cast<uint32_t>(in[2]) << 8) | static_cast<uint32_t>(in[3]);
}

uint16_t readBigEndian16(const uint8_t* in) {
  return static_cast<uint16_t>((static_cast<uint16_t>(in[0]) << 8) | in[1]);
}

/// True when now - last has reached interval, safe across the 32-bit
/// millisecond wrap that happens every ~49.7 days of uptime.
bool intervalElapsed(uint32_t nowMs, uint32_t lastMs, uint16_t intervalMs) {
  return static_cast<uint32_t>(nowMs - lastMs) >= intervalMs;
}

} // namespace

// --- Field encoders ---------------------------------------------------------

uint32_t encodeTimeSinceHourStart(uint8_t minute, uint8_t second, uint16_t millisecond) {
  const uint32_t m = (minute > 59) ? 59u : minute;
  const uint32_t s = (second > 59) ? 59u : second;
  const uint32_t ms = (millisecond > 999) ? 999u : millisecond;
  return m * 30000u + s * 500u + ms / 2u;
}

int32_t encodeDateAndHour(uint16_t year, uint8_t month, uint8_t day, uint8_t hour) {
  if (year < 2000 || month < 1 || month > 12 || day < 1 || day > 31 || hour > 23) {
    return -1;
  }
  return static_cast<int32_t>(year - 2000) * 8928 + static_cast<int32_t>(month - 1) * 744 +
         static_cast<int32_t>(day - 1) * 24 + static_cast<int32_t>(hour);
}

uint16_t encodeAltitude(float meters, bool valid) {
  if (!valid || meters != meters) {
    return kInvalidAltitude;
  }
  if (meters <= kFineAltitudeCeilingMeters) {
    const int32_t tenths =
        roundClamp((meters - kAltitudeFloorMeters) * 10.0f, 0, kFifteenBitMask);
    return static_cast<uint16_t>(tenths & kFifteenBitMask);
  }
  const int32_t whole =
      roundClamp(meters - kAltitudeFloorMeters, 0,
                 static_cast<int32_t>(kCoarseAltitudeCeilingMeters - kAltitudeFloorMeters));
  return static_cast<uint16_t>((whole & kFifteenBitMask) | kCoarseFlag);
}

uint16_t encodeSpeed(float kph, bool valid) {
  if (!valid || kph != kph) {
    return kInvalidSpeed;
  }
  if (kph <= kFineSpeedCeilingKph) {
    const int32_t hundredths = roundClamp(kph * 100.0f, 0, kFifteenBitMask);
    return static_cast<uint16_t>(hundredths & kFifteenBitMask);
  }
  const int32_t tenths = roundClamp(kph * 10.0f, 0, kFifteenBitMask);
  return static_cast<uint16_t>((tenths & kFifteenBitMask) | kCoarseFlag);
}

uint16_t encodeBearing(float degrees, bool valid) {
  if (!valid || degrees != degrees) {
    return kInvalidBearing;
  }
  float wrapped = fmodf(degrees, 360.0f);
  if (wrapped < 0.0f) {
    wrapped += 360.0f;
  }
  const int32_t hundredths = roundClamp(wrapped * 100.0f, 0, 35999);
  return static_cast<uint16_t>(hundredths);
}

uint8_t encodeDop(float dop, bool valid) {
  if (!valid || dop != dop || dop < 0.0f) {
    return kInvalidDop;
  }
  return static_cast<uint8_t>(roundClamp(dop * 10.0f, 0, kInvalidDop - 1));
}

int32_t encodeCoordinate(int32_t degreesE7, bool valid) {
  return valid ? degreesE7 : kInvalidCoordinate;
}

// --- GpsEncoder -------------------------------------------------------------

void GpsEncoder::reset() {
  syncBits_ = 0;
  lastDateAndHour_ = -1;
  hasDateAndHour_ = false;
}

GpsPackets GpsEncoder::encode(const telemetry::GnssFix& fix) {
  GpsPackets packets;

  const int32_t dateAndHour = (fix.dateValid && fix.timeValid)
                                  ? encodeDateAndHour(fix.year, fix.month, fix.day, fix.hour)
                                  : -1;

  if (dateAndHour >= 0 && (!hasDateAndHour_ || dateAndHour != lastDateAndHour_)) {
    lastDateAndHour_ = dateAndHour;
    hasDateAndHour_ = true;
    syncBits_ = static_cast<uint8_t>((syncBits_ + 1) & 0x07);
    packets.timeChanged = true;
  }

  const uint32_t timeSinceHourStart =
      fix.timeValid ? encodeTimeSinceHourStart(fix.minute, fix.second, fix.millisecond) : 0;

  packets.main[0] =
      static_cast<uint8_t>(((syncBits_ & 0x07) << 5) | ((timeSinceHourStart >> 16) & 0x1F));
  packets.main[1] = static_cast<uint8_t>(timeSinceHourStart >> 8);
  packets.main[2] = static_cast<uint8_t>(timeSinceHourStart);

  const uint8_t quality = (fix.fixQuality > 3) ? 3 : fix.fixQuality;
  // 0x3F is the reserved "unknown" value, so a real count is capped at 0x3E.
  const uint8_t satellites =
      fix.satellitesValid ? ((fix.satellites > 0x3E) ? 0x3E : fix.satellites) : kInvalidSatellites;
  packets.main[3] = static_cast<uint8_t>((quality << 6) | (satellites & 0x3F));

  writeBigEndian32(&packets.main[4],
                   static_cast<uint32_t>(encodeCoordinate(fix.latitudeE7, fix.positionValid)));
  writeBigEndian32(&packets.main[8],
                   static_cast<uint32_t>(encodeCoordinate(fix.longitudeE7, fix.positionValid)));
  writeBigEndian16(&packets.main[12], encodeAltitude(fix.altitudeMeters, fix.altitudeValid));
  writeBigEndian16(&packets.main[14], encodeSpeed(fix.speedKph, fix.speedValid));
  writeBigEndian16(&packets.main[16], encodeBearing(fix.bearingDeg, fix.bearingValid));
  packets.main[18] = encodeDop(fix.hdop, fix.hdopValid);
  packets.main[19] = encodeDop(fix.vdop, fix.vdopValid);

  const uint32_t dateField = (dateAndHour >= 0) ? static_cast<uint32_t>(dateAndHour) : 0u;
  packets.time[0] = static_cast<uint8_t>(((syncBits_ & 0x07) << 5) | ((dateField >> 16) & 0x1F));
  packets.time[1] = static_cast<uint8_t>(dateField >> 8);
  packets.time[2] = static_cast<uint8_t>(dateField);

  return packets;
}

// --- CAN main ---------------------------------------------------------------

size_t encodeCanPacket(uint32_t pid, const uint8_t* payload, size_t payloadLength, uint8_t* out) {
  if (out == nullptr || payload == nullptr || payloadLength == 0 ||
      payloadLength > kCanMaxPayloadSize) {
    return 0;
  }
  // The specification calls this out explicitly: the packet ID is little-endian
  // even though every other field in the API is big-endian.
  out[0] = static_cast<uint8_t>(pid);
  out[1] = static_cast<uint8_t>(pid >> 8);
  out[2] = static_cast<uint8_t>(pid >> 16);
  out[3] = static_cast<uint8_t>(pid >> 24);
  memcpy(out + kCanPidSize, payload, payloadLength);
  return kCanPidSize + payloadLength;
}

// --- CanFilter --------------------------------------------------------------

void CanFilter::reset() {
  allowAll_ = false;
  defaultNotifyIntervalMs_ = 0;
  allowAllLastNotifiedMs_ = 0;
  allowAllHasBeenNotified_ = false;
  pidCount_ = 0;
}

CanFilter::PidEntry* CanFilter::find(uint32_t pid) {
  for (size_t i = 0; i < pidCount_; ++i) {
    if (pids_[i].pid == pid) {
      return &pids_[i];
    }
  }
  return nullptr;
}

const CanFilter::PidEntry* CanFilter::find(uint32_t pid) const {
  for (size_t i = 0; i < pidCount_; ++i) {
    if (pids_[i].pid == pid) {
      return &pids_[i];
    }
  }
  return nullptr;
}

bool CanFilter::handleWrite(const uint8_t* data, size_t length) {
  if (data == nullptr || length < 1) {
    return false;
  }

  switch (static_cast<FilterCommand>(data[0])) {
    case FilterCommand::kDenyAll:
      reset();
      return true;

    case FilterCommand::kAllowAll: {
      if (length < 3) {
        return false;
      }
      reset();
      allowAll_ = true;
      defaultNotifyIntervalMs_ = readBigEndian16(&data[1]);
      return true;
    }

    case FilterCommand::kAllowOnePid: {
      if (length < 7) {
        return false;
      }
      const uint16_t interval = readBigEndian16(&data[1]);
      const uint32_t pid = readBigEndian32(&data[3]);

      PidEntry* existing = find(pid);
      if (existing != nullptr) {
        existing->notifyIntervalMs = interval;
        return true;
      }
      if (pidCount_ >= kMaxAllowedPids) {
        return false;
      }
      pids_[pidCount_].pid = pid;
      pids_[pidCount_].notifyIntervalMs = interval;
      pids_[pidCount_].lastNotifiedMs = 0;
      pids_[pidCount_].hasBeenNotified = false;
      ++pidCount_;
      return true;
    }
  }
  return false;
}

bool CanFilter::isAllowed(uint32_t pid) const {
  return allowAll_ || find(pid) != nullptr;
}

uint16_t CanFilter::notifyIntervalMs(uint32_t pid) const {
  const PidEntry* entry = find(pid);
  if (entry != nullptr) {
    return entry->notifyIntervalMs;
  }
  return allowAll_ ? defaultNotifyIntervalMs_ : 0;
}

bool CanFilter::shouldNotify(uint32_t pid, uint32_t nowMs) {
  PidEntry* entry = find(pid);
  if (entry != nullptr) {
    if (entry->hasBeenNotified &&
        !intervalElapsed(nowMs, entry->lastNotifiedMs, entry->notifyIntervalMs)) {
      return false;
    }
    entry->lastNotifiedMs = nowMs;
    entry->hasBeenNotified = true;
    return true;
  }

  if (!allowAll_) {
    return false;
  }
  // Promiscuous mode shares one rate limiter, because RaceChrono only supplies
  // a single interval for the whole stream.
  if (allowAllHasBeenNotified_ &&
      !intervalElapsed(nowMs, allowAllLastNotifiedMs_, defaultNotifyIntervalMs_)) {
    return false;
  }
  allowAllLastNotifiedMs_ = nowMs;
  allowAllHasBeenNotified_ = true;
  return true;
}

} // namespace rc
