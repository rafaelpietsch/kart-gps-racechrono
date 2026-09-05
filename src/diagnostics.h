// SPDX-License-Identifier: MIT
//
// Bench diagnostics: what the I2C bus and the GPS UART are actually doing.
//
// None of this runs on the hot path. It exists because the two buses on this
// board are internal wires -- without a logic analyser the only way to see
// them is to have the firmware say what it sees.

#ifndef KARTGPS_DIAGNOSTICS_H
#define KARTGPS_DIAGNOSTICS_H

#include <Arduino.h>
#include <Wire.h>

namespace diag {

/// Result of one sweep of the 7-bit address space.
struct I2cScanResult {
  static constexpr size_t kMaxFound = 8;
  uint8_t found[kMaxFound] = {0};
  uint8_t foundCount = 0;
  /// Addresses that failed with something other than a plain address NACK.
  /// A bus with no pull-ups, or with SDA held low, produces these for every
  /// address rather than the silent NACK an empty address gives.
  uint16_t busErrors = 0;
};

/// Reports the idle level of SDA and SCL. Both must read high: a line stuck
/// low means a missing pull-up, a short, or a device holding the bus.
void reportBusLines(int sdaPin, int sclPin, Print& out);

/// Sweeps 0x08..0x77 and prints every device that acknowledges.
I2cScanResult scanI2c(TwoWire& wire, Print& out);

/// Reads a register block without going through the driver, so it still works
/// when Mpu6050::begin() has failed. `errorOut` carries the Wire error code.
bool readRegisters(TwoWire& wire, uint8_t address, uint8_t reg, uint8_t* buffer, size_t length,
                   uint8_t& errorOut);

/// Dumps the MPU-6050 configuration registers and one raw sample burst,
/// decoded into g and dps so the numbers can be sanity checked by eye.
void dumpMpuRegisters(TwoWire& wire, uint8_t address, Print& out);

/// Human readable name for a Wire::endTransmission() return code.
const char* wireErrorName(uint8_t code);

} // namespace diag

#endif // KARTGPS_DIAGNOSTICS_H
