// SPDX-License-Identifier: MIT

#include "mpu6050.h"

namespace hal {
namespace {

/// PWR_MGMT_1: clear SLEEP and select the X axis gyro PLL as the clock source,
/// which the datasheet recommends over the internal oscillator for stability.
constexpr uint8_t kPowerWakeGyroClock = 0x01;
constexpr uint8_t kPowerDeviceReset = 0x80;

/// SIGNAL_PATH_RESET: gyro, accelerometer and temperature paths.
constexpr uint8_t kSignalPathResetAll = 0x07;

constexpr uint32_t kResetSettleMs = 100;

} // namespace

bool Mpu6050::writeRegister(uint8_t reg, uint8_t value) {
  wire_.beginTransmission(config_.address);
  wire_.write(reg);
  wire_.write(value);
  if (wire_.endTransmission() != 0) {
    ++errorCount_;
    return false;
  }
  return true;
}

bool Mpu6050::readRegisters(uint8_t reg, uint8_t* buffer, size_t length) {
  wire_.beginTransmission(config_.address);
  wire_.write(reg);
  // A repeated start (endTransmission(false)) keeps the bus for the read, which
  // is what the MPU-6050 expects for a burst.
  if (wire_.endTransmission(false) != 0) {
    ++errorCount_;
    return false;
  }
  const size_t received = wire_.requestFrom(config_.address, static_cast<uint8_t>(length));
  if (received != length) {
    ++errorCount_;
    return false;
  }
  for (size_t i = 0; i < length; ++i) {
    buffer[i] = static_cast<uint8_t>(wire_.read());
  }
  return true;
}

bool Mpu6050::begin(const Config& config) {
  config_ = config;

  uint8_t whoAmI = 0;
  if (!readRegisters(imu::reg::kWhoAmI, &whoAmI, 1)) {
    return false;
  }
  if (whoAmI != imu::kWhoAmIValue) {
    return false;
  }

  if (!writeRegister(imu::reg::kPowerManagement1, kPowerDeviceReset)) {
    return false;
  }
  delay(kResetSettleMs);

  if (!writeRegister(imu::reg::kPowerManagement1, kPowerWakeGyroClock)) {
    return false;
  }
  if (!writeRegister(imu::reg::kConfig, static_cast<uint8_t>(config_.bandwidth))) {
    return false;
  }
  if (!writeRegister(imu::reg::kSampleRateDivider,
                     imu::sampleRateDivider(config_.sampleRateHz, config_.bandwidth))) {
    return false;
  }
  // The range selector lives in bits 4:3 of both configuration registers.
  if (!writeRegister(imu::reg::kGyroConfig,
                     static_cast<uint8_t>(static_cast<uint8_t>(config_.gyroRange) << 3))) {
    return false;
  }
  if (!writeRegister(imu::reg::kAccelConfig,
                     static_cast<uint8_t>(static_cast<uint8_t>(config_.accelRange) << 3))) {
    return false;
  }
  // No interrupt line is wired on this board; the loop polls instead.
  if (!writeRegister(imu::reg::kIntEnable, 0x00)) {
    return false;
  }
  return true;
}

bool Mpu6050::resetSignalPath() {
  if (!writeRegister(imu::reg::kSignalPathReset, kSignalPathResetAll)) {
    return false;
  }
  delay(kResetSettleMs);
  return begin(config_);
}

bool Mpu6050::read(telemetry::ImuRawSample& out, uint32_t nowMs) {
  uint8_t burst[imu::kBurstLength];
  if (!readRegisters(imu::reg::kAccelXoutH, burst, sizeof(burst))) {
    return false;
  }
  if (!imu::decodeBurst(burst, sizeof(burst), out)) {
    return false;
  }
  out.timestampMs = nowMs;
  return true;
}

} // namespace hal
