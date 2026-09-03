// SPDX-License-Identifier: MIT

#include "channels.h"

#include <math.h>

namespace kart {
namespace channels {
namespace {

/// Rounds and saturates into the int16 range instead of wrapping, so a kerb
/// strike beyond full scale reads as "very large" rather than as a sign flip.
int16_t saturateToInt16(float value) {
  if (!(value == value)) {
    return 0;
  }
  const float rounded = (value >= 0.0f) ? floorf(value + 0.5f) : ceilf(value - 0.5f);
  if (rounded > 32767.0f) {
    return 32767;
  }
  if (rounded < -32768.0f) {
    return -32768;
  }
  return static_cast<int16_t>(rounded);
}

void writeInt16(uint8_t* out, int16_t value) {
  const uint16_t raw = static_cast<uint16_t>(value);
  out[0] = static_cast<uint8_t>(raw >> 8);
  out[1] = static_cast<uint8_t>(raw);
}

void writeUint32(uint8_t* out, uint32_t value) {
  out[0] = static_cast<uint8_t>(value >> 24);
  out[1] = static_cast<uint8_t>(value >> 16);
  out[2] = static_cast<uint8_t>(value >> 8);
  out[3] = static_cast<uint8_t>(value);
}

} // namespace

size_t encodeMotionPayload(const telemetry::ImuSample& sample, uint8_t* out, size_t capacity) {
  if (out == nullptr || capacity < kMotionPayloadSize) {
    return 0;
  }
  for (size_t axis = 0; axis < 3; ++axis) {
    writeInt16(&out[axis * 2], saturateToInt16(sample.accelG[axis] * kAccelMilliGPerUnit));
    writeInt16(&out[6 + axis * 2], saturateToInt16(sample.gyroDps[axis] * kGyroUnitsPerDps));
  }
  return kMotionPayloadSize;
}

size_t encodeStatusPayload(const DeviceStatus& status, uint8_t* out, size_t capacity) {
  if (out == nullptr || capacity < kStatusPayloadSize) {
    return 0;
  }
  writeUint32(&out[0], status.sessionTimeMs);
  out[4] = status.satellitesValid ? status.satellites : 0xFF;
  out[5] = status.fixQuality;
  out[6] = status.flags;
  writeInt16(&out[7], saturateToInt16(status.temperatureC * kTemperatureUnitsPerC));
  out[9] = static_cast<uint8_t>(status.sessionResets);
  return kStatusPayloadSize;
}

} // namespace channels
} // namespace kart
