// SPDX-License-Identifier: MIT
//
// The data channels this device publishes on the RaceChrono CAN-Bus
// characteristic, and their payload layouts.
//
// RaceChrono treats the CAN characteristic as "any sensor data", so the packet
// IDs below are ours to define. They are documented in docs/racechrono-setup.md
// together with the channel equations to paste into the app; keep the two in
// step or the app will decode garbage.
//
// Payload fields are big-endian, matching the rest of the RaceChrono API.

#ifndef KARTGPS_CHANNELS_H
#define KARTGPS_CHANNELS_H

#include <stddef.h>
#include <stdint.h>

#include "telemetry_types.h"

namespace kart {
namespace channels {

/// Motion channel: 3 axis acceleration and 3 axis rotation, published at the
/// IMU rate (25 Hz by default).
///
/// byte 0-1   accel X  int16, milli-g, positive forward
/// byte 2-3   accel Y  int16, milli-g, positive right
/// byte 4-5   accel Z  int16, milli-g, positive up
/// byte 6-7   gyro X   int16, 0.1 deg/s, roll rate
/// byte 8-9   gyro Y   int16, 0.1 deg/s, pitch rate
/// byte 10-11 gyro Z   int16, 0.1 deg/s, yaw rate
constexpr uint32_t kPidMotion = 0x100;
constexpr size_t kMotionPayloadSize = 12;

/// Status channel, published once per second.
///
/// byte 0-3  session time      uint32, milliseconds since the last SET press
/// byte 4    satellites        uint8, 0xFF when unknown
/// byte 5    fix quality       uint8, as reported by GGA
/// byte 6    status flags      uint8, see StatusFlag
/// byte 7-8  device temperature int16, 0.1 degrees Celsius
/// byte 9    session resets    uint8, wraps at 256
constexpr uint32_t kPidStatus = 0x101;
constexpr size_t kStatusPayloadSize = 10;

/// Bit positions inside the status flags byte.
enum StatusFlag : uint8_t {
  kFlagImuCalibrated = 1 << 0,
  kFlagGnssFixValid = 1 << 1,
  kFlagCalibrationRunning = 1 << 2,
  kFlagCalibrationRejected = 1 << 3,
  kFlagGnssStale = 1 << 4,
};

/// Scale factors, kept next to the layout so the encoder and the RaceChrono
/// channel definitions cannot drift apart.
constexpr float kAccelMilliGPerUnit = 1000.0f;
constexpr float kGyroUnitsPerDps = 10.0f;
constexpr float kTemperatureUnitsPerC = 10.0f;

/// Snapshot of everything the status channel reports.
struct DeviceStatus {
  uint32_t sessionTimeMs = 0;
  uint8_t satellites = 0xFF;
  bool satellitesValid = false;
  uint8_t fixQuality = 0;
  uint8_t flags = 0;
  float temperatureC = 0.0f;
  uint16_t sessionResets = 0;
};

/// Writes the motion payload. Returns bytes written, or 0 if the buffer is
/// too small.
size_t encodeMotionPayload(const telemetry::ImuSample& sample, uint8_t* out, size_t capacity);

/// Writes the status payload. Returns bytes written, or 0 if the buffer is
/// too small.
size_t encodeStatusPayload(const DeviceStatus& status, uint8_t* out, size_t capacity);

} // namespace channels
} // namespace kart

#endif // KARTGPS_CHANNELS_H
