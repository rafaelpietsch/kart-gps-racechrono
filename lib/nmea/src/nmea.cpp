// SPDX-License-Identifier: MIT

#include "nmea.h"

#include <string.h>

namespace nmea {
namespace {

struct Field {
  const char* ptr = nullptr;
  size_t length = 0;
};

/// Returns field `index` of a comma separated body. Index 0 is the sentence
/// type itself, so RMC's time field is index 1, matching the specification
/// tables. Empty fields are returned with length 0 and are not an error.
bool getField(const char* body, size_t length, size_t index, Field& out) {
  size_t current = 0;
  size_t start = 0;
  for (size_t i = 0; i <= length; ++i) {
    const bool atEnd = (i == length);
    if (atEnd || body[i] == ',') {
      if (current == index) {
        out.ptr = body + start;
        out.length = i - start;
        return true;
      }
      ++current;
      start = i + 1;
    }
  }
  return false;
}

bool hexValue(char c, uint8_t& value) {
  if (c >= '0' && c <= '9') {
    value = static_cast<uint8_t>(c - '0');
    return true;
  }
  if (c >= 'A' && c <= 'F') {
    value = static_cast<uint8_t>(c - 'A' + 10);
    return true;
  }
  if (c >= 'a' && c <= 'f') {
    value = static_cast<uint8_t>(c - 'a' + 10);
    return true;
  }
  return false;
}

/// Powers of ten as int64, used to rescale fractional digits without floats.
int64_t powerOfTen(int exponent) {
  int64_t result = 1;
  for (int i = 0; i < exponent; ++i) {
    result *= 10;
  }
  return result;
}

/// Splits "123.456" into its integer part and its fractional digits.
bool splitDecimal(const char* field, size_t length, int64_t& integerPart, int64_t& fraction,
                  int& fractionDigits, bool& negative) {
  integerPart = 0;
  fraction = 0;
  fractionDigits = 0;
  negative = false;

  size_t i = 0;
  if (i < length && (field[i] == '+' || field[i] == '-')) {
    negative = (field[i] == '-');
    ++i;
  }

  size_t digitsBefore = 0;
  for (; i < length && field[i] != '.'; ++i) {
    if (field[i] < '0' || field[i] > '9') {
      return false;
    }
    integerPart = integerPart * 10 + (field[i] - '0');
    ++digitsBefore;
    if (digitsBefore > 18) {
      return false;
    }
  }

  size_t digitsAfter = 0;
  if (i < length && field[i] == '.') {
    ++i;
    for (; i < length; ++i) {
      if (field[i] < '0' || field[i] > '9') {
        return false;
      }
      if (digitsAfter < 9) {
        fraction = fraction * 10 + (field[i] - '0');
        ++digitsAfter;
      }
    }
  }

  fractionDigits = static_cast<int>(digitsAfter);
  return (digitsBefore + digitsAfter) > 0;
}

/// Rescales `fraction` (which has `digits` decimals) to exactly `targetDigits`.
int64_t rescaleFraction(int64_t fraction, int digits, int targetDigits) {
  if (digits == targetDigits) {
    return fraction;
  }
  if (digits < targetDigits) {
    return fraction * powerOfTen(targetDigits - digits);
  }
  const int64_t divisor = powerOfTen(digits - targetDigits);
  // Round to nearest rather than truncating, so a coordinate does not drift
  // systematically towards zero.
  return (fraction + divisor / 2) / divisor;
}

constexpr size_t kTypeTokenLength = 5; ///< e.g. "GPRMC" or "GNGGA"

/// Returns a pointer to the 3 character sentence type, or nullptr.
const char* sentenceType(const char* body, size_t length) {
  Field type;
  if (!getField(body, length, 0, type) || type.length != kTypeTokenLength) {
    return nullptr;
  }
  return type.ptr + 2;
}

bool typeIs(const char* type, const char* name) {
  return type != nullptr && memcmp(type, name, 3) == 0;
}

uint32_t timeKeyOf(uint8_t hour, uint8_t minute, uint8_t second, uint16_t millisecond) {
  return static_cast<uint32_t>(hour) * 10000000u + static_cast<uint32_t>(minute) * 100000u +
         static_cast<uint32_t>(second) * 1000u + millisecond;
}

} // namespace

// --- Checksum ---------------------------------------------------------------

uint8_t computeChecksum(const char* body, size_t length) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < length; ++i) {
    checksum ^= static_cast<uint8_t>(body[i]);
  }
  return checksum;
}

// --- SentenceAssembler ------------------------------------------------------

void SentenceAssembler::reset() {
  state_ = State::kIdle;
  length_ = 0;
  readyLength_ = 0;
  runningChecksum_ = 0;
  expectedChecksum_ = 0;
  ready_[0] = '\0';
  buffer_[0] = '\0';
}

void SentenceAssembler::discard() {
  state_ = State::kIdle;
  length_ = 0;
  runningChecksum_ = 0;
}

bool SentenceAssembler::push(char c) {
  // A '$' always starts a new sentence, whatever we were in the middle of.
  // Losing a truncated sentence beats emitting a spliced one.
  if (c == '$') {
    state_ = State::kBody;
    length_ = 0;
    runningChecksum_ = 0;
    return false;
  }

  switch (state_) {
    case State::kIdle:
      return false;

    case State::kBody: {
      if (c == '*') {
        state_ = State::kChecksumHigh;
        return false;
      }
      if (c == '\r' || c == '\n') {
        // Sentences without a checksum are legal NMEA but we refuse them: on a
        // noisy UART the checksum is the only thing standing between a bit flip
        // and a corrupt lap.
        ++stats_.checksumErrors;
        discard();
        return false;
      }
      if (length_ >= kMaxSentenceLength) {
        ++stats_.overflows;
        discard();
        return false;
      }
      buffer_[length_++] = c;
      runningChecksum_ ^= static_cast<uint8_t>(c);
      return false;
    }

    case State::kChecksumHigh: {
      uint8_t nibble = 0;
      if (!hexValue(c, nibble)) {
        ++stats_.checksumErrors;
        discard();
        return false;
      }
      expectedChecksum_ = static_cast<uint8_t>(nibble << 4);
      state_ = State::kChecksumLow;
      return false;
    }

    case State::kChecksumLow: {
      uint8_t nibble = 0;
      if (!hexValue(c, nibble)) {
        ++stats_.checksumErrors;
        discard();
        return false;
      }
      expectedChecksum_ = static_cast<uint8_t>(expectedChecksum_ | nibble);
      const bool valid = (expectedChecksum_ == runningChecksum_);
      if (!valid) {
        ++stats_.checksumErrors;
        discard();
        return false;
      }
      memcpy(ready_, buffer_, length_);
      ready_[length_] = '\0';
      readyLength_ = length_;
      ++stats_.accepted;
      discard();
      return true;
    }
  }
  return false;
}

// --- Field parsers ----------------------------------------------------------

bool parseUnsigned(const char* field, size_t length, uint32_t& value) {
  if (field == nullptr || length == 0 || length > 10) {
    return false;
  }
  uint32_t result = 0;
  for (size_t i = 0; i < length; ++i) {
    if (field[i] < '0' || field[i] > '9') {
      return false;
    }
    result = result * 10 + static_cast<uint32_t>(field[i] - '0');
  }
  value = result;
  return true;
}

bool parseNumber(const char* field, size_t length, float& value) {
  if (field == nullptr || length == 0) {
    return false;
  }
  int64_t integerPart = 0;
  int64_t fraction = 0;
  int fractionDigits = 0;
  bool negative = false;
  if (!splitDecimal(field, length, integerPart, fraction, fractionDigits, negative)) {
    return false;
  }
  float result = static_cast<float>(integerPart);
  if (fractionDigits > 0) {
    result += static_cast<float>(fraction) / static_cast<float>(powerOfTen(fractionDigits));
  }
  value = negative ? -result : result;
  return true;
}

bool parseCoordinate(const char* field, size_t length, char hemisphere, int32_t& degreesE7) {
  if (field == nullptr || length < 3) {
    return false;
  }

  int64_t integerPart = 0;
  int64_t fraction = 0;
  int fractionDigits = 0;
  bool negative = false;
  if (!splitDecimal(field, length, integerPart, fraction, fractionDigits, negative) || negative) {
    return false;
  }

  // The last two integer digits are whole minutes, everything before them is
  // whole degrees: 4807.038 is 48 degrees 07.038 minutes.
  const int64_t minutes = integerPart % 100;
  const int64_t degrees = integerPart / 100;
  if (minutes >= 60 || degrees > 180) {
    return false;
  }

  const int64_t minutesE7 = minutes * 10000000 + rescaleFraction(fraction, fractionDigits, 7);
  int64_t totalE7 = degrees * 10000000 + (minutesE7 + 30) / 60;

  switch (hemisphere) {
    case 'N':
    case 'E':
      break;
    case 'S':
    case 'W':
      totalE7 = -totalE7;
      break;
    default:
      return false;
  }

  if (totalE7 > 1800000000LL || totalE7 < -1800000000LL) {
    return false;
  }
  degreesE7 = static_cast<int32_t>(totalE7);
  return true;
}

bool parseTimeOfDay(const char* field, size_t length, uint8_t& hour, uint8_t& minute,
                    uint8_t& second, uint16_t& millisecond) {
  if (field == nullptr || length < 6) {
    return false;
  }
  int64_t integerPart = 0;
  int64_t fraction = 0;
  int fractionDigits = 0;
  bool negative = false;
  if (!splitDecimal(field, length, integerPart, fraction, fractionDigits, negative) || negative) {
    return false;
  }
  // Guard against a truncated "hhmm" style field slipping through.
  if (integerPart > 235960 || integerPart < 0) {
    return false;
  }

  const int64_t hh = integerPart / 10000;
  const int64_t mm = (integerPart / 100) % 100;
  const int64_t ss = integerPart % 100;
  if (hh > 23 || mm > 59 || ss > 60) {
    return false;
  }

  hour = static_cast<uint8_t>(hh);
  minute = static_cast<uint8_t>(mm);
  // A leap second reads as :60; clamp so downstream arithmetic stays in range.
  second = static_cast<uint8_t>(ss > 59 ? 59 : ss);
  millisecond = static_cast<uint16_t>(rescaleFraction(fraction, fractionDigits, 3));
  if (millisecond > 999) {
    millisecond = 999;
  }
  return true;
}

bool parseDate(const char* field, size_t length, uint16_t& year, uint8_t& month, uint8_t& day) {
  if (field == nullptr || length != 6) {
    return false;
  }
  uint32_t packed = 0;
  if (!parseUnsigned(field, length, packed)) {
    return false;
  }
  const uint32_t dd = packed / 10000;
  const uint32_t mm = (packed / 100) % 100;
  const uint32_t yy = packed % 100;
  if (dd < 1 || dd > 31 || mm < 1 || mm > 12) {
    return false;
  }
  // NMEA only carries two year digits. RMC has no century, so this rolls over
  // in 2100 -- acceptable for a lap timer, and called out here so it is not a
  // surprise.
  year = static_cast<uint16_t>(2000 + yy);
  month = static_cast<uint8_t>(mm);
  day = static_cast<uint8_t>(dd);
  return true;
}

// --- FixAssembler -----------------------------------------------------------

void FixAssembler::reset() {
  fix_ = telemetry::GnssFix();
  currentTimeKey_ = 0;
  hasTimeKey_ = false;
  seenRmc_ = false;
  seenGga_ = false;
  taken_ = false;
}

void FixAssembler::beginEpochIfTimeChanged(uint32_t timeKey) {
  if (hasTimeKey_ && timeKey == currentTimeKey_) {
    return;
  }
  telemetry::GnssFix fresh;
  // The date only ever arrives in RMC, so carry it into the new epoch instead
  // of dropping back to "unknown date" on every GGA.
  fresh.dateValid = fix_.dateValid;
  fresh.year = fix_.year;
  fresh.month = fix_.month;
  fresh.day = fix_.day;

  fix_ = fresh;
  currentTimeKey_ = timeKey;
  hasTimeKey_ = true;
  seenRmc_ = false;
  seenGga_ = false;
  taken_ = false;
}

FixAssembler::Update FixAssembler::consume(const char* body, size_t length) {
  const char* type = sentenceType(body, length);
  if (type == nullptr) {
    return Update::kIgnored;
  }

  if (typeIs(type, "RMC")) {
    Field time;
    Field status;
    Field lat;
    Field ns;
    Field lon;
    Field ew;
    Field speed;
    Field course;
    Field date;
    if (!getField(body, length, 1, time) || !getField(body, length, 2, status) ||
        !getField(body, length, 3, lat) || !getField(body, length, 4, ns) ||
        !getField(body, length, 5, lon) || !getField(body, length, 6, ew) ||
        !getField(body, length, 7, speed) || !getField(body, length, 8, course) ||
        !getField(body, length, 9, date)) {
      return Update::kMalformed;
    }

    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    uint16_t millisecond = 0;
    if (!parseTimeOfDay(time.ptr, time.length, hour, minute, second, millisecond)) {
      return Update::kMalformed;
    }
    beginEpochIfTimeChanged(timeKeyOf(hour, minute, second, millisecond));

    fix_.timeValid = true;
    fix_.hour = hour;
    fix_.minute = minute;
    fix_.second = second;
    fix_.millisecond = millisecond;
    fix_.navigationValid = (status.length == 1 && status.ptr[0] == 'A');

    int32_t latitudeE7 = 0;
    int32_t longitudeE7 = 0;
    if (ns.length == 1 && ew.length == 1 &&
        parseCoordinate(lat.ptr, lat.length, ns.ptr[0], latitudeE7) &&
        parseCoordinate(lon.ptr, lon.length, ew.ptr[0], longitudeE7)) {
      fix_.latitudeE7 = latitudeE7;
      fix_.longitudeE7 = longitudeE7;
      fix_.positionValid = fix_.navigationValid;
    }

    float knots = 0.0f;
    if (parseNumber(speed.ptr, speed.length, knots) && knots >= 0.0f) {
      fix_.speedKph = knots * kKnotsToKph;
      fix_.speedValid = true;
    }

    float bearing = 0.0f;
    if (parseNumber(course.ptr, course.length, bearing)) {
      fix_.bearingDeg = bearing;
      fix_.bearingValid = true;
    }

    uint16_t year = 0;
    uint8_t month = 0;
    uint8_t day = 0;
    if (parseDate(date.ptr, date.length, year, month, day)) {
      fix_.year = year;
      fix_.month = month;
      fix_.day = day;
      fix_.dateValid = true;
    }

    seenRmc_ = true;
    return Update::kRmc;
  }

  if (typeIs(type, "GGA")) {
    Field time;
    Field quality;
    Field satellites;
    Field hdop;
    Field altitude;
    if (!getField(body, length, 1, time) || !getField(body, length, 6, quality) ||
        !getField(body, length, 7, satellites) || !getField(body, length, 8, hdop) ||
        !getField(body, length, 9, altitude)) {
      return Update::kMalformed;
    }

    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    uint16_t millisecond = 0;
    if (!parseTimeOfDay(time.ptr, time.length, hour, minute, second, millisecond)) {
      return Update::kMalformed;
    }
    beginEpochIfTimeChanged(timeKeyOf(hour, minute, second, millisecond));

    fix_.timeValid = true;
    fix_.hour = hour;
    fix_.minute = minute;
    fix_.second = second;
    fix_.millisecond = millisecond;

    uint32_t value = 0;
    if (parseUnsigned(quality.ptr, quality.length, value)) {
      fix_.fixQuality = static_cast<uint8_t>(value > 255 ? 255 : value);
    }
    if (parseUnsigned(satellites.ptr, satellites.length, value)) {
      fix_.satellites = static_cast<uint8_t>(value > 255 ? 255 : value);
      fix_.satellitesValid = true;
    }

    float number = 0.0f;
    if (parseNumber(hdop.ptr, hdop.length, number) && number >= 0.0f) {
      fix_.hdop = number;
      fix_.hdopValid = true;
    }
    // Only trust the altitude once the receiver reports an actual fix; with
    // quality 0 the field is either empty or left over from the last epoch.
    if (fix_.fixQuality > 0 && parseNumber(altitude.ptr, altitude.length, number)) {
      fix_.altitudeMeters = number;
      fix_.altitudeValid = true;
    }

    seenGga_ = true;
    return Update::kGga;
  }

  if (typeIs(type, "GSA")) {
    Field pdop;
    Field hdop;
    Field vdop;
    if (!getField(body, length, 15, pdop) || !getField(body, length, 16, hdop) ||
        !getField(body, length, 17, vdop)) {
      return Update::kMalformed;
    }
    float number = 0.0f;
    if (parseNumber(pdop.ptr, pdop.length, number) && number >= 0.0f) {
      fix_.pdop = number;
      fix_.pdopValid = true;
    }
    if (parseNumber(hdop.ptr, hdop.length, number) && number >= 0.0f) {
      fix_.hdop = number;
      fix_.hdopValid = true;
    }
    if (parseNumber(vdop.ptr, vdop.length, number) && number >= 0.0f) {
      fix_.vdop = number;
      fix_.vdopValid = true;
    }
    return Update::kGsa;
  }

  return Update::kIgnored;
}

bool FixAssembler::hasCompleteFix() const {
  return seenRmc_ && seenGga_ && !taken_;
}

bool FixAssembler::takeCompletedFix(telemetry::GnssFix& out) {
  if (!hasCompleteFix()) {
    return false;
  }
  out = fix_;
  taken_ = true;
  return true;
}

} // namespace nmea
