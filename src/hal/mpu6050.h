// SPDX-License-Identifier: MIT
//
// I2C driver for the MPU-6050 on the GY-521 breakout.

#ifndef KARTGPS_HAL_MPU6050_H
#define KARTGPS_HAL_MPU6050_H

#include <Arduino.h>
#include <Wire.h>

#include "imu.h"
#include "telemetry_types.h"

namespace hal {

class Mpu6050 {
public:
  struct Config {
    uint8_t address = imu::kI2cAddressLow;
    imu::AccelRange accelRange = imu::AccelRange::k4G;
    imu::GyroRange gyroRange = imu::GyroRange::k500Dps;
    /// 44 Hz keeps kerb strikes visible while cutting the engine vibration
    /// that would otherwise alias into the sampled band.
    imu::DlpfBandwidth bandwidth = imu::DlpfBandwidth::k44Hz;
    uint16_t sampleRateHz = 100;
  };

  explicit Mpu6050(TwoWire& wire) : wire_(wire) {}

  /// Probes WHO_AM_I, wakes the device and applies the configuration.
  /// Returns false when the sensor does not answer or identifies as something
  /// else, which is almost always a wiring or address problem.
  bool begin(const Config& config);

  /// Reads one 14 byte burst. Returns false on an I2C error.
  bool read(telemetry::ImuRawSample& out, uint32_t nowMs);

  /// Re-applies the configuration after a signal path reset. Used by the full
  /// reset action so a wedged sensor recovers without a power cycle.
  bool resetSignalPath();

  const Config& config() const { return config_; }
  uint32_t errorCount() const { return errorCount_; }

private:
  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRegisters(uint8_t reg, uint8_t* buffer, size_t length);

  TwoWire& wire_;
  Config config_;
  uint32_t errorCount_ = 0;
};

} // namespace hal

#endif // KARTGPS_HAL_MPU6050_H
