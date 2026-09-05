// SPDX-License-Identifier: MIT

#include "imu.h"

#include <math.h>

namespace imu {
namespace {

constexpr float kTemperatureScaleLsbPerC = 340.0f;
constexpr float kTemperatureOffsetC = 36.53f;

int16_t readBigEndian16(const uint8_t* in) {
  return static_cast<int16_t>((static_cast<uint16_t>(in[0]) << 8) | in[1]);
}

float magnitude(const float v[3]) {
  return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

} // namespace

bool isCompatibleWhoAmI(uint8_t value) {
  switch (value) {
    case 0x68: // MPU-6050
    case 0x70: // MPU-6500
    case 0x71: // MPU-9250
    case 0x72: // reported by several MPU-6050 and MPU-9250 clone modules
    case 0x73: // MPU-9255
    case 0x74:
    case 0x75:
      return true;
    default:
      return false;
  }
}

float accelScaleLsbPerG(AccelRange range) {
  switch (range) {
    case AccelRange::k2G:
      return 16384.0f;
    case AccelRange::k4G:
      return 8192.0f;
    case AccelRange::k8G:
      return 4096.0f;
    case AccelRange::k16G:
      return 2048.0f;
  }
  return 16384.0f;
}

float gyroScaleLsbPerDps(GyroRange range) {
  switch (range) {
    case GyroRange::k250Dps:
      return 131.0f;
    case GyroRange::k500Dps:
      return 65.5f;
    case GyroRange::k1000Dps:
      return 32.8f;
    case GyroRange::k2000Dps:
      return 16.4f;
  }
  return 131.0f;
}

bool dlpfBypassesFilter(DlpfBandwidth bandwidth) {
  return bandwidth == DlpfBandwidth::k260Hz;
}

uint8_t sampleRateDivider(uint16_t desiredHz, DlpfBandwidth bandwidth) {
  if (desiredHz == 0) {
    return 0xFF;
  }
  const uint32_t baseHz = dlpfBypassesFilter(bandwidth) ? 8000u : 1000u;
  if (desiredHz >= baseHz) {
    return 0;
  }
  // Round the division up so the resulting rate never exceeds the request:
  // overshooting would push more samples than the BLE link was sized for.
  const uint32_t divider = (baseHz + desiredHz - 1u) / desiredHz - 1u;
  return static_cast<uint8_t>(divider > 0xFF ? 0xFF : divider);
}

float outputRateHz(uint8_t divider, DlpfBandwidth bandwidth) {
  const float baseHz = dlpfBypassesFilter(bandwidth) ? 8000.0f : 1000.0f;
  return baseHz / static_cast<float>(divider + 1);
}

bool decodeBurst(const uint8_t* raw, size_t length, telemetry::ImuRawSample& out) {
  if (raw == nullptr || length < kBurstLength) {
    return false;
  }
  out.accel[telemetry::kAxisX] = readBigEndian16(&raw[0]);
  out.accel[telemetry::kAxisY] = readBigEndian16(&raw[2]);
  out.accel[telemetry::kAxisZ] = readBigEndian16(&raw[4]);
  out.temperature = readBigEndian16(&raw[6]);
  out.gyro[telemetry::kAxisX] = readBigEndian16(&raw[8]);
  out.gyro[telemetry::kAxisY] = readBigEndian16(&raw[10]);
  out.gyro[telemetry::kAxisZ] = readBigEndian16(&raw[12]);
  return true;
}

telemetry::ImuSample toPhysicalUnits(const telemetry::ImuRawSample& raw, AccelRange accelRange,
                                     GyroRange gyroRange) {
  telemetry::ImuSample sample;
  const float accelScale = accelScaleLsbPerG(accelRange);
  const float gyroScale = gyroScaleLsbPerDps(gyroRange);
  for (size_t axis = 0; axis < 3; ++axis) {
    sample.accelG[axis] = static_cast<float>(raw.accel[axis]) / accelScale;
    sample.gyroDps[axis] = static_cast<float>(raw.gyro[axis]) / gyroScale;
  }
  sample.temperatureC =
      static_cast<float>(raw.temperature) / kTemperatureScaleLsbPerC + kTemperatureOffsetC;
  sample.timestampMs = raw.timestampMs;
  return sample;
}

// --- Mat3 -------------------------------------------------------------------

Mat3 Mat3::identity() {
  return Mat3();
}

void Mat3::apply(const float in[3], float out[3]) const {
  const float x = in[0];
  const float y = in[1];
  const float z = in[2];
  out[0] = m[0][0] * x + m[0][1] * y + m[0][2] * z;
  out[1] = m[1][0] * x + m[1][1] * y + m[1][2] * z;
  out[2] = m[2][0] * x + m[2][1] * y + m[2][2] * z;
}

Mat3 Mat3::multiply(const Mat3& rhs) const {
  Mat3 result;
  for (size_t r = 0; r < 3; ++r) {
    for (size_t c = 0; c < 3; ++c) {
      result.m[r][c] =
          m[r][0] * rhs.m[0][c] + m[r][1] * rhs.m[1][c] + m[r][2] * rhs.m[2][c];
    }
  }
  return result;
}

Mat3 rotationAligningVector(const float from[3], const float to[3]) {
  float a[3] = {from[0], from[1], from[2]};
  float b[3] = {to[0], to[1], to[2]};
  const float lenA = magnitude(a);
  const float lenB = magnitude(b);
  if (lenA <= 1e-6f || lenB <= 1e-6f) {
    return Mat3::identity();
  }
  for (size_t i = 0; i < 3; ++i) {
    a[i] /= lenA;
    b[i] /= lenB;
  }

  const float v[3] = {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
                      a[0] * b[1] - a[1] * b[0]};
  const float cosine = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

  if (cosine > 1.0f - 1e-6f) {
    return Mat3::identity();
  }
  if (cosine < -1.0f + 1e-6f) {
    // Antiparallel: rotate 180 degrees about any axis perpendicular to a.
    float axis[3] = {1.0f, 0.0f, 0.0f};
    if (fabsf(a[0]) > 0.9f) {
      axis[0] = 0.0f;
      axis[1] = 1.0f;
    }
    const float dot = axis[0] * a[0] + axis[1] * a[1] + axis[2] * a[2];
    for (size_t i = 0; i < 3; ++i) {
      axis[i] -= dot * a[i];
    }
    const float len = magnitude(axis);
    for (size_t i = 0; i < 3; ++i) {
      axis[i] /= len;
    }
    Mat3 result;
    for (size_t r = 0; r < 3; ++r) {
      for (size_t c = 0; c < 3; ++c) {
        result.m[r][c] = 2.0f * axis[r] * axis[c] - (r == c ? 1.0f : 0.0f);
      }
    }
    return result;
  }

  // Rodrigues: R = I + [v]x + [v]x^2 / (1 + cos)
  const float k = 1.0f / (1.0f + cosine);
  Mat3 result;
  result.m[0][0] = 1.0f + k * (-v[2] * v[2] - v[1] * v[1]);
  result.m[0][1] = -v[2] + k * (v[0] * v[1]);
  result.m[0][2] = v[1] + k * (v[0] * v[2]);
  result.m[1][0] = v[2] + k * (v[0] * v[1]);
  result.m[1][1] = 1.0f + k * (-v[2] * v[2] - v[0] * v[0]);
  result.m[1][2] = -v[0] + k * (v[1] * v[2]);
  result.m[2][0] = -v[1] + k * (v[0] * v[2]);
  result.m[2][1] = v[0] + k * (v[1] * v[2]);
  result.m[2][2] = 1.0f + k * (-v[1] * v[1] - v[0] * v[0]);
  return result;
}

Mat3 rotationAboutZ(float degrees) {
  const float radians = degrees * 3.14159265358979323846f / 180.0f;
  const float c = cosf(radians);
  const float s = sinf(radians);
  Mat3 result;
  result.m[0][0] = c;
  result.m[0][1] = -s;
  result.m[0][2] = 0.0f;
  result.m[1][0] = s;
  result.m[1][1] = c;
  result.m[1][2] = 0.0f;
  result.m[2][0] = 0.0f;
  result.m[2][1] = 0.0f;
  result.m[2][2] = 1.0f;
  return result;
}

// --- BiasCalibrator ---------------------------------------------------------

void BiasCalibrator::begin(uint16_t sampleCount) {
  state_ = State::kRunning;
  required_ = (sampleCount == 0) ? 1 : sampleCount;
  collected_ = 0;
  for (size_t axis = 0; axis < 3; ++axis) {
    accelSum_[axis] = 0.0;
    gyroSum_[axis] = 0.0;
    accelMin_[axis] = 0.0f;
    accelMax_[axis] = 0.0f;
    gyroMin_[axis] = 0.0f;
    gyroMax_[axis] = 0.0f;
  }
  bias_ = SensorBias();
}

void BiasCalibrator::abort() {
  state_ = State::kIdle;
  collected_ = 0;
}

bool BiasCalibrator::addSample(const telemetry::ImuSample& sample) {
  if (state_ != State::kRunning) {
    return false;
  }

  for (size_t axis = 0; axis < 3; ++axis) {
    const float a = sample.accelG[axis];
    const float g = sample.gyroDps[axis];
    accelSum_[axis] += a;
    gyroSum_[axis] += g;
    if (collected_ == 0) {
      accelMin_[axis] = accelMax_[axis] = a;
      gyroMin_[axis] = gyroMax_[axis] = g;
    } else {
      if (a < accelMin_[axis]) accelMin_[axis] = a;
      if (a > accelMax_[axis]) accelMax_[axis] = a;
      if (g < gyroMin_[axis]) gyroMin_[axis] = g;
      if (g > gyroMax_[axis]) gyroMax_[axis] = g;
    }
  }

  ++collected_;
  if (collected_ < required_) {
    return false;
  }
  finish();
  return true;
}

void BiasCalibrator::finish() {
  const float count = static_cast<float>(collected_);
  float accelMean[3];
  float gyroMean[3];
  for (size_t axis = 0; axis < 3; ++axis) {
    accelMean[axis] = static_cast<float>(accelSum_[axis] / count);
    gyroMean[axis] = static_cast<float>(gyroSum_[axis] / count);
  }

  for (size_t axis = 0; axis < 3; ++axis) {
    if ((accelMax_[axis] - accelMin_[axis]) > kMaxAccelSpreadG ||
        (gyroMax_[axis] - gyroMin_[axis]) > kMaxGyroSpreadDps) {
      state_ = State::kRejected;
      return;
    }
  }

  const float gravity = magnitude(accelMean);
  if (gravity < kMinGravityMagnitudeG || gravity > kMaxGravityMagnitudeG) {
    state_ = State::kRejected;
    return;
  }

  bias_ = SensorBias();
  bias_.valid = true;
  for (size_t axis = 0; axis < 3; ++axis) {
    bias_.gyroDps[axis] = gyroMean[axis];
    bias_.gravityG[axis] = accelMean[axis];
    // The whole stationary reading is gravity by definition, so the residual
    // accelerometer bias in the vehicle frame is what is left after levelling
    // -- handled by MotionProcessor, not folded in here.
    bias_.accelG[axis] = 0.0f;
  }
  state_ = State::kComplete;
}

// --- MotionProcessor --------------------------------------------------------

void MotionProcessor::setBias(const SensorBias& bias) {
  bias_ = bias;
  rebuildRotation();
}

void MotionProcessor::clearCalibration() {
  bias_ = SensorBias();
  rebuildRotation();
}

void MotionProcessor::setMountingYawDegrees(float degrees) {
  mountingYawDegrees_ = degrees;
  rebuildRotation();
}

void MotionProcessor::setGravityCompensation(bool enabled) {
  gravityCompensation_ = enabled;
}

void MotionProcessor::rebuildRotation() {
  if (!bias_.valid) {
    rotation_ = rotationAboutZ(mountingYawDegrees_);
    return;
  }
  // A stationary accelerometer measures specific force, which points up, so
  // levelling means rotating the captured vector onto +Z.
  const float up[3] = {0.0f, 0.0f, 1.0f};
  const Mat3 level = rotationAligningVector(bias_.gravityG, up);
  rotation_ = rotationAboutZ(mountingYawDegrees_).multiply(level);
}

telemetry::ImuSample MotionProcessor::process(const telemetry::ImuSample& sample) const {
  telemetry::ImuSample out;
  out.timestampMs = sample.timestampMs;
  out.temperatureC = sample.temperatureC;

  float gyroCorrected[3];
  float accelCorrected[3];
  for (size_t axis = 0; axis < 3; ++axis) {
    gyroCorrected[axis] = sample.gyroDps[axis] - (bias_.valid ? bias_.gyroDps[axis] : 0.0f);
    accelCorrected[axis] = sample.accelG[axis] - (bias_.valid ? bias_.accelG[axis] : 0.0f);
  }

  rotation_.apply(gyroCorrected, out.gyroDps);
  rotation_.apply(accelCorrected, out.accelG);

  if (gravityCompensation_ && bias_.valid) {
    // After levelling, gravity sits entirely on +Z with the magnitude that was
    // measured at rest, so removing it leaves a true vertical acceleration.
    out.accelG[telemetry::kAxisZ] -= magnitude(bias_.gravityG);
  }
  return out;
}

} // namespace imu
