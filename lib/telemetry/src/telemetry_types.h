// SPDX-License-Identifier: MIT
//
// Domain types shared by every layer of the pipeline.
//
// This header is deliberately free of any Arduino/ESP-IDF dependency so that
// the parsing, encoding and fusion logic can be compiled and unit tested on a
// desktop host. Only the code under src/ is allowed to touch the hardware.

#ifndef KARTGPS_TELEMETRY_TYPES_H
#define KARTGPS_TELEMETRY_TYPES_H

#include <stdint.h>

namespace telemetry {

/// A GNSS solution, assembled incrementally from several NMEA sentences.
///
/// Every optional group carries its own validity flag instead of relying on
/// sentinel values: a receiver that has a 2D fix reports a position but no
/// altitude, and the encoder has to be able to tell those apart.
struct GnssFix {
  // --- UTC time of the fix -------------------------------------------------
  bool timeValid = false;
  uint8_t hour = 0;         ///< 0..23
  uint8_t minute = 0;       ///< 0..59
  uint8_t second = 0;       ///< 0..59 (leap second 60 is clamped by the parser)
  uint16_t millisecond = 0; ///< 0..999

  bool dateValid = false;
  uint16_t year = 0; ///< Full year, e.g. 2026
  uint8_t month = 0; ///< 1..12
  uint8_t day = 0;   ///< 1..31

  // --- Position ------------------------------------------------------------
  bool positionValid = false;
  int32_t latitudeE7 = 0;  ///< degrees * 1e7, positive north
  int32_t longitudeE7 = 0; ///< degrees * 1e7, positive east

  bool altitudeValid = false;
  float altitudeMeters = 0.0f; ///< Above mean sea level

  // --- Motion --------------------------------------------------------------
  bool speedValid = false;
  float speedKph = 0.0f;

  bool bearingValid = false;
  float bearingDeg = 0.0f; ///< True course over ground, 0..360

  // --- Quality -------------------------------------------------------------
  /// GGA fix quality: 0 = invalid, 1 = GPS, 2 = DGPS, 4 = RTK fixed, 5 = RTK float.
  uint8_t fixQuality = 0;

  bool satellitesValid = false;
  uint8_t satellites = 0;

  bool hdopValid = false;
  float hdop = 0.0f;
  bool vdopValid = false;
  float vdop = 0.0f;
  bool pdopValid = false;
  float pdop = 0.0f;

  /// True when RMC reported status 'A' (data valid).
  bool navigationValid = false;

  /// Milliseconds on the device clock when this fix was completed. Used to age
  /// out a stale fix; it is not part of the wire format.
  uint32_t receivedAtMs = 0;
};

/// One raw sample straight out of the MPU-6050 burst read.
struct ImuRawSample {
  int16_t accel[3] = {0, 0, 0}; ///< X, Y, Z in ADC counts
  int16_t gyro[3] = {0, 0, 0};  ///< X, Y, Z in ADC counts
  int16_t temperature = 0;      ///< ADC counts
  uint32_t timestampMs = 0;
};

/// A sample converted to physical units and corrected for sensor bias.
struct ImuSample {
  float accelG[3] = {0.0f, 0.0f, 0.0f};   ///< g (1 g = 9.80665 m/s^2)
  float gyroDps[3] = {0.0f, 0.0f, 0.0f};  ///< degrees per second
  float temperatureC = 0.0f;
  uint32_t timestampMs = 0;
};

/// Axis indices, used throughout so call sites never index with magic numbers.
enum Axis : uint8_t { kAxisX = 0, kAxisY = 1, kAxisZ = 2 };

} // namespace telemetry

#endif // KARTGPS_TELEMETRY_TYPES_H
