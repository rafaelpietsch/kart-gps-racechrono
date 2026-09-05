// SPDX-License-Identifier: MIT
//
// MPU-6050 register map, unit conversion, zeroing and mounting compensation.
//
// Nothing here talks to I2C: the driver in src/hal feeds raw register bytes in
// and gets vehicle-frame accelerations out, which keeps the interesting maths
// testable on a desktop.
//
// Frame convention after processing:
//   X  longitudinal, positive forward   (braking is negative)
//   Y  lateral,      positive right
//   Z  vertical,     positive up
// Gravity is removed by default, so a stationary, calibrated device reads
// (0, 0, 0) on all three axes.

#ifndef KARTGPS_IMU_H
#define KARTGPS_IMU_H

#include <stddef.h>
#include <stdint.h>

#include "telemetry_types.h"

namespace imu {

// --- Device constants -------------------------------------------------------

constexpr uint8_t kI2cAddressLow = 0x68;  ///< AD0 tied low (GY-521 default)
constexpr uint8_t kI2cAddressHigh = 0x69; ///< AD0 tied high
constexpr uint8_t kWhoAmIValue = 0x68;    ///< What a genuine MPU-6050 reports

/// True for WHO_AM_I identities this driver can drive.
///
/// Boards sold as GY-521 modules do not all carry a genuine MPU-6050. The
/// parts that turn up in their place -- the MPU-6500 and MPU-9250 family, and
/// the unbranded clones that report neither -- share the register map this
/// firmware uses: the same PWR_MGMT_1, the same SMPLRT_DIV and CONFIG, the
/// same 14 byte burst at 0x3B, and crucially the same accelerometer and
/// gyroscope scale factors. Refusing to talk to them buys nothing.
///
/// The check is kept rather than dropped because it still catches a read that
/// landed on some other device entirely. A part that is accepted here but
/// scales differently is caught downstream anyway: BiasCalibrator rejects a
/// stationary capture that does not read close to 1 g.
bool isCompatibleWhoAmI(uint8_t value);

namespace reg {
constexpr uint8_t kSampleRateDivider = 0x19;
constexpr uint8_t kConfig = 0x1A;
constexpr uint8_t kGyroConfig = 0x1B;
constexpr uint8_t kAccelConfig = 0x1C;
constexpr uint8_t kIntPinConfig = 0x37;
constexpr uint8_t kIntEnable = 0x38;
constexpr uint8_t kIntStatus = 0x3A;
constexpr uint8_t kAccelXoutH = 0x3B; ///< First register of the 14 byte burst
constexpr uint8_t kSignalPathReset = 0x68;
constexpr uint8_t kPowerManagement1 = 0x6B;
constexpr uint8_t kWhoAmI = 0x75;
} // namespace reg

/// Accel, temperature and gyro come out of one 14 byte burst read from 0x3B.
constexpr size_t kBurstLength = 14;

enum class AccelRange : uint8_t { k2G = 0, k4G = 1, k8G = 2, k16G = 3 };
enum class GyroRange : uint8_t { k250Dps = 0, k500Dps = 1, k1000Dps = 2, k2000Dps = 3 };

/// Digital low pass filter settings. The bandwidth figure is the accelerometer
/// one; picking a filter also drops the internal gyro rate from 8 kHz to 1 kHz.
enum class DlpfBandwidth : uint8_t {
  k260Hz = 0, ///< Filter disabled, gyro output rate 8 kHz
  k184Hz = 1,
  k94Hz = 2,
  k44Hz = 3,
  k21Hz = 4,
  k10Hz = 5,
  k5Hz = 6,
};

float accelScaleLsbPerG(AccelRange range);
float gyroScaleLsbPerDps(GyroRange range);

/// True when the DLPF setting leaves the internal sample rate at 8 kHz.
bool dlpfBypassesFilter(DlpfBandwidth bandwidth);

/// Value for SMPRT_DIV that gets closest to `desiredHz` without exceeding it.
/// Returns 0xFF (the slowest setting) when the request is unreachably slow.
uint8_t sampleRateDivider(uint16_t desiredHz, DlpfBandwidth bandwidth);

/// Actual output rate produced by a given divider, for logging and tests.
float outputRateHz(uint8_t divider, DlpfBandwidth bandwidth);

/// Decodes the 14 byte burst (big-endian, two's complement).
bool decodeBurst(const uint8_t* raw, size_t length, telemetry::ImuRawSample& out);

/// Converts ADC counts to g, degrees per second and degrees Celsius.
telemetry::ImuSample toPhysicalUnits(const telemetry::ImuRawSample& raw, AccelRange accelRange,
                                     GyroRange gyroRange);

// --- Small 3x3 maths --------------------------------------------------------

struct Mat3 {
  float m[3][3] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

  static Mat3 identity();
  void apply(const float in[3], float out[3]) const;
  Mat3 multiply(const Mat3& rhs) const;
};

/// Rotation taking unit vector `from` onto unit vector `to` by the shortest
/// path (Rodrigues). Both inputs are normalised internally.
Mat3 rotationAligningVector(const float from[3], const float to[3]);

/// Rotation about the Z axis, in degrees, right handed.
Mat3 rotationAboutZ(float degrees);

// --- Zeroing ----------------------------------------------------------------

/// Everything the SET button captures while the kart stands still.
struct SensorBias {
  bool valid = false;
  float gyroDps[3] = {0.0f, 0.0f, 0.0f};  ///< Mean rate at rest, to be subtracted
  float gravityG[3] = {0.0f, 0.0f, 0.0f}; ///< Mean specific force at rest, points up
  float accelG[3] = {0.0f, 0.0f, 0.0f};   ///< Residual accel bias, gravity removed
};

/// Averages a window of stationary samples into a SensorBias.
///
/// The window is rejected if the kart moved while it was running: a zero taken
/// on a rolling kart is worse than no zero at all, because it silently biases
/// every lap that follows.
class BiasCalibrator {
public:
  static constexpr uint16_t kDefaultSampleCount = 256;
  /// Peak-to-peak tolerance over the window before the capture is rejected.
  static constexpr float kMaxAccelSpreadG = 0.20f;
  static constexpr float kMaxGyroSpreadDps = 10.0f;
  /// A stationary device must read close to 1 g in total.
  static constexpr float kMinGravityMagnitudeG = 0.85f;
  static constexpr float kMaxGravityMagnitudeG = 1.15f;

  enum class State : uint8_t { kIdle, kRunning, kComplete, kRejected };

  void begin(uint16_t sampleCount = kDefaultSampleCount);
  void abort();

  /// Feeds one converted sample. Returns true once the window finished, whether
  /// it was accepted or rejected -- check state() to tell those apart.
  bool addSample(const telemetry::ImuSample& sample);

  State state() const { return state_; }
  bool isRunning() const { return state_ == State::kRunning; }
  bool isComplete() const { return state_ == State::kComplete; }
  bool isRejected() const { return state_ == State::kRejected; }

  uint16_t collected() const { return collected_; }
  uint16_t required() const { return required_; }

  const SensorBias& bias() const { return bias_; }

private:
  void finish();

  State state_ = State::kIdle;
  uint16_t required_ = kDefaultSampleCount;
  uint16_t collected_ = 0;
  double accelSum_[3] = {0.0, 0.0, 0.0};
  double gyroSum_[3] = {0.0, 0.0, 0.0};
  float accelMin_[3] = {0.0f, 0.0f, 0.0f};
  float accelMax_[3] = {0.0f, 0.0f, 0.0f};
  float gyroMin_[3] = {0.0f, 0.0f, 0.0f};
  float gyroMax_[3] = {0.0f, 0.0f, 0.0f};
  SensorBias bias_;
};

/// Applies a captured bias and the mounting rotation to live samples.
class MotionProcessor {
public:
  /// Installs a calibration. A bias with valid == false clears it.
  void setBias(const SensorBias& bias);
  void clearCalibration();

  /// Rotation about the vertical axis between the device and the kart, applied
  /// after levelling. Set it when the box cannot be mounted facing forward.
  void setMountingYawDegrees(float degrees);

  /// When true (the default) the 1 g of gravity is removed from the vertical
  /// axis, so a stationary kart reads zero on all three channels.
  void setGravityCompensation(bool enabled);

  bool isCalibrated() const { return bias_.valid; }
  const SensorBias& bias() const { return bias_; }
  const Mat3& rotation() const { return rotation_; }

  /// Converts one sample from sensor frame to vehicle frame.
  telemetry::ImuSample process(const telemetry::ImuSample& sample) const;

private:
  void rebuildRotation();

  SensorBias bias_;
  float mountingYawDegrees_ = 0.0f;
  bool gravityCompensation_ = true;
  Mat3 rotation_;
};

} // namespace imu

#endif // KARTGPS_IMU_H
