// SPDX-License-Identifier: MIT
//
// NMEA 0183 handling for the u-blox NEO-6M.
//
// Split in two so both halves can be tested in isolation:
//   SentenceAssembler  bytes from the UART  ->  checksum validated sentences
//   FixAssembler       sentences            ->  a merged telemetry::GnssFix
//
// The parsers are allocation free and use integer arithmetic for latitude and
// longitude, so a coordinate keeps its full 1e-7 degree (about 1.1 cm)
// resolution instead of losing digits to a 32-bit float.

#ifndef KARTGPS_NMEA_H
#define KARTGPS_NMEA_H

#include <stddef.h>
#include <stdint.h>

#include "telemetry_types.h"

namespace nmea {

/// Reassembles UART bytes into complete sentences and verifies the checksum.
///
/// A sentence is delivered as its *body*: everything between the leading '$'
/// and the '*' of the checksum, NUL terminated. A sentence without a checksum
/// is rejected, which keeps a noisy or half-connected UART from injecting
/// garbage into the fix.
class SentenceAssembler {
public:
  /// NMEA 0183 caps a sentence at 82 characters, but u-blox emits longer GSV
  /// lines when many constellations are enabled, so leave headroom.
  static constexpr size_t kMaxSentenceLength = 127;

  struct Stats {
    uint32_t accepted = 0;       ///< Sentences delivered to the caller
    uint32_t checksumErrors = 0; ///< Complete sentences with a bad checksum
    uint32_t overflows = 0;      ///< Sentences longer than the buffer
  };

  /// Feeds one byte. Returns true when a complete, valid sentence is available
  /// from sentence(); the sentence stays valid until the next push().
  bool push(char c);

  const char* sentence() const { return ready_; }
  size_t length() const { return readyLength_; }
  const Stats& stats() const { return stats_; }

  void reset();

private:
  void discard();

  enum class State : uint8_t { kIdle, kBody, kChecksumHigh, kChecksumLow };

  State state_ = State::kIdle;
  char buffer_[kMaxSentenceLength + 1] = {0};
  char ready_[kMaxSentenceLength + 1] = {0};
  size_t length_ = 0;
  size_t readyLength_ = 0;
  uint8_t runningChecksum_ = 0;
  uint8_t expectedChecksum_ = 0;
  Stats stats_;
};

/// Computes the NMEA checksum (XOR of every byte of the body).
uint8_t computeChecksum(const char* body, size_t length);

/// Merges the sentences of one epoch into a single GnssFix.
///
/// u-blox emits one burst per navigation epoch. With the recommended message
/// set the order is GGA, GSA, RMC, so a fix is complete once RMC has been seen
/// for the same UTC timestamp as the GGA.
class FixAssembler {
public:
  enum class Update : uint8_t {
    kIgnored,   ///< Sentence type we do not care about
    kRmc,       ///< Position, speed, course, date
    kGga,       ///< Altitude, satellite count, HDOP, fix quality
    kGsa,       ///< PDOP / HDOP / VDOP
    kMalformed, ///< Recognised type but the fields did not parse
  };

  Update consume(const char* body, size_t length);

  /// True when the current epoch has both RMC and GGA and has not been taken.
  bool hasCompleteFix() const;

  /// Moves the completed fix out and arms the assembler for the next epoch.
  /// Returns false when no complete fix is pending.
  bool takeCompletedFix(telemetry::GnssFix& out);

  /// The fix as assembled so far, complete or not. Useful for diagnostics.
  const telemetry::GnssFix& partialFix() const { return fix_; }

  void reset();

private:
  /// Starts a new epoch when the sentence timestamp moved on, keeping the date
  /// because only RMC carries it.
  void beginEpochIfTimeChanged(uint32_t timeKey);

  telemetry::GnssFix fix_;
  uint32_t currentTimeKey_ = 0;
  bool hasTimeKey_ = false;
  bool seenRmc_ = false;
  bool seenGga_ = false;
  bool taken_ = false;
};

// --- Field level helpers, exposed for testing -------------------------------

/// Parses "ddmm.mmmm" plus a hemisphere character into degrees * 1e7.
bool parseCoordinate(const char* field, size_t length, char hemisphere, int32_t& degreesE7);

/// Parses "hhmmss" or "hhmmss.sss" (any number of fractional digits).
bool parseTimeOfDay(const char* field, size_t length, uint8_t& hour, uint8_t& minute,
                    uint8_t& second, uint16_t& millisecond);

/// Parses "ddmmyy" into a full year.
bool parseDate(const char* field, size_t length, uint16_t& year, uint8_t& month, uint8_t& day);

/// Parses a decimal number, optionally signed, into a float.
bool parseNumber(const char* field, size_t length, float& value);

/// Parses an unsigned decimal integer.
bool parseUnsigned(const char* field, size_t length, uint32_t& value);

constexpr float kKnotsToKph = 1.852f;

} // namespace nmea

#endif // KARTGPS_NMEA_H
