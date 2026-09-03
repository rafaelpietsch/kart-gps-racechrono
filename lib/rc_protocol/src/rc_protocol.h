// SPDX-License-Identifier: MIT
//
// Wire format of the RaceChrono "DIY BLE device" API.
//
// Reference: reference/racechrono-ble-diy-device/README.md (git submodule).
// Service 0x1FF8 with characteristics:
//   0x0001  CAN-Bus main    NOTIFY + READ   little-endian PID + 1..16 byte payload
//   0x0002  CAN-Bus filter  WRITE           the app tells us which PIDs it wants
//   0x0003  GPS main        NOTIFY + READ   20 byte fix packet
//   0x0004  GPS time        NOTIFY + READ   3 byte date/hour packet
//
// Every multi-byte field is big-endian *except* the CAN packet ID, which the
// specification defines as little-endian.

#ifndef KARTGPS_RC_PROTOCOL_H
#define KARTGPS_RC_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#include "telemetry_types.h"

namespace rc {

// --- BLE identifiers --------------------------------------------------------

constexpr uint16_t kServiceUuid = 0x1FF8;
constexpr uint16_t kCharCanMainUuid = 0x0001;
constexpr uint16_t kCharCanFilterUuid = 0x0002;
constexpr uint16_t kCharGpsMainUuid = 0x0003;
constexpr uint16_t kCharGpsTimeUuid = 0x0004;

// --- Packet sizes -----------------------------------------------------------

constexpr size_t kGpsMainPacketSize = 20;
constexpr size_t kGpsTimePacketSize = 3;
constexpr size_t kCanPidSize = 4;
constexpr size_t kCanMaxPayloadSize = 16;
constexpr size_t kCanMaxPacketSize = kCanPidSize + kCanMaxPayloadSize;

// --- Invalid-value sentinels defined by the specification -------------------

constexpr int32_t kInvalidCoordinate = 0x7FFFFFFF;
constexpr uint16_t kInvalidAltitude = 0xFFFF;
constexpr uint16_t kInvalidSpeed = 0xFFFF;
constexpr uint16_t kInvalidBearing = 0xFFFF;
constexpr uint8_t kInvalidDop = 0xFF;
constexpr uint8_t kInvalidSatellites = 0x3F;

// --- Field encoders ---------------------------------------------------------
//
// Exposed individually because the ranges and the fine/coarse switchover are
// the subtle part of this protocol, and they deserve their own tests.

/// 21-bit time from the start of the hour: minute*30000 + second*500 + ms/2.
uint32_t encodeTimeSinceHourStart(uint8_t minute, uint8_t second, uint16_t millisecond);

/// 21-bit date/hour: (year-2000)*8928 + (month-1)*744 + (day-1)*24 + hour.
/// Returns -1 when the date is not usable.
int32_t encodeDateAndHour(uint16_t year, uint8_t month, uint8_t day, uint8_t hour);

/// Altitude in the dual-resolution encoding.
/// Fine   (bit 15 clear): ((m + 500) * 10) & 0x7FFF -- 0.1 m step, -500..2776.7 m
/// Coarse (bit 15 set):   ((m + 500) & 0x7FFF) | 0x8000 -- 1 m step, -500..32267 m
uint16_t encodeAltitude(float meters, bool valid);

/// Speed in the dual-resolution encoding.
/// Fine   (bit 15 clear): (kph * 100) & 0x7FFF -- 0.01 km/h step, 0..327.67 km/h
/// Coarse (bit 15 set):   ((kph * 10) & 0x7FFF) | 0x8000 -- 0.1 km/h step
uint16_t encodeSpeed(float kph, bool valid);

/// Bearing as degrees * 100, wrapped into [0, 360).
uint16_t encodeBearing(float degrees, bool valid);

/// Dilution of precision as dop * 10, saturating at 0xFE.
uint8_t encodeDop(float dop, bool valid);

/// Latitude/longitude as degrees * 1e7.
int32_t encodeCoordinate(int32_t degreesE7, bool valid);

// --- GPS characteristics ----------------------------------------------------

/// The pair of packets produced for one fix.
struct GpsPackets {
  uint8_t main[kGpsMainPacketSize] = {0};
  uint8_t time[kGpsTimePacketSize] = {0};
  /// True when the date/hour rolled over, which advanced the sync counter. The
  /// 0x0004 characteristic only has to be notified when this is set.
  bool timeChanged = false;
};

/// Builds the GPS characteristic payloads and owns the 3-bit sync counter that
/// ties characteristic 0x0003 to 0x0004.
class GpsEncoder {
public:
  /// Encodes one fix. Advances the sync counter when the date/hour changed.
  GpsPackets encode(const telemetry::GnssFix& fix);

  /// Drops the sync state so the next fix is treated as the first one.
  void reset();

  uint8_t syncBits() const { return syncBits_; }

private:
  uint8_t syncBits_ = 0;
  int32_t lastDateAndHour_ = -1;
  bool hasDateAndHour_ = false;
};

// --- CAN-Bus main characteristic -------------------------------------------

/// Writes a CAN packet into `out` (which must hold kCanMaxPacketSize bytes).
/// Returns the number of bytes written, or 0 when the payload length is out of
/// the 1..16 range the specification allows.
size_t encodeCanPacket(uint32_t pid, const uint8_t* payload, size_t payloadLength,
                       uint8_t* out);

// --- CAN-Bus filter characteristic ------------------------------------------

/// Command IDs the app writes to characteristic 0x0002.
enum class FilterCommand : uint8_t {
  kDenyAll = 0,
  kAllowAll = 1,
  kAllowOnePid = 2,
};

/// Tracks which PIDs RaceChrono asked for and how often it wants them.
///
/// The device starts in "deny all" so nothing is published until the app has
/// stated its interest, which is what the reference implementation does and
/// what keeps a fresh connection quiet.
class CanFilter {
public:
  /// Number of individually allowed PIDs we can remember. Eight covers the
  /// channels this firmware publishes with room to spare.
  static constexpr size_t kMaxAllowedPids = 8;

  /// Back to "deny all", called on every disconnect.
  void reset();

  /// Applies one write to the filter characteristic.
  /// Returns false when the payload is malformed or the PID table is full.
  bool handleWrite(const uint8_t* data, size_t length);

  /// True when this PID may be published at all.
  bool isAllowed(uint32_t pid) const;

  /// Notify interval in milliseconds for this PID; 0 means "as fast as it
  /// arrives". Returns 0 for PIDs that are not allowed.
  uint16_t notifyIntervalMs(uint32_t pid) const;

  /// Combines isAllowed() with the per-PID rate limit. Records the timestamp
  /// when it returns true, so callers must only call it when they really are
  /// about to send.
  bool shouldNotify(uint32_t pid, uint32_t nowMs);

  bool allowAll() const { return allowAll_; }
  size_t allowedPidCount() const { return pidCount_; }

private:
  struct PidEntry {
    uint32_t pid = 0;
    uint16_t notifyIntervalMs = 0;
    uint32_t lastNotifiedMs = 0;
    bool hasBeenNotified = false;
  };

  PidEntry* find(uint32_t pid);
  const PidEntry* find(uint32_t pid) const;

  bool allowAll_ = false;
  uint16_t defaultNotifyIntervalMs_ = 0;
  uint32_t allowAllLastNotifiedMs_ = 0;
  bool allowAllHasBeenNotified_ = false;
  PidEntry pids_[kMaxAllowedPids];
  size_t pidCount_ = 0;
};

} // namespace rc

#endif // KARTGPS_RC_PROTOCOL_H
