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
#include <HardwareSerial.h>
#include <Wire.h>

#include "imu.h"

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
///
/// The burst is read straight off the bus, so it does not pass through the
/// driver and carries no trim correction. `trim` is therefore passed in and
/// reported alongside: without it the dump contradicts a zeroing that has just
/// succeeded, which is worse than saying nothing.
void dumpMpuRegisters(TwoWire& wire, uint8_t address, Print& out, const imu::RawAccelBias& trim);

/// Human readable name for a Wire::endTransmission() return code.
const char* wireErrorName(uint8_t code);

// --- GPS link ---------------------------------------------------------------

/// Tests both GPS pins as plain GPIOs, before any UART is involved.
///
/// Each pin is held down by an internal pull-down and then read. A powered UART
/// idles high and overpowers the pull-down, so a line that still reads low has
/// nothing driving it. Counting edges then separates "connected and idle" from
/// "connected and talking".
///
/// Both pins are checked rather than just the receive one, because the reading
/// that says "nothing is driving my RX" has a second cause besides no power: the
/// pair wired the wrong way round. In that case the module's TX lands on our
/// transmit pin, and the activity shows up there instead. Testing one pin
/// cannot tell those apart; testing both can.
///
/// Leaves the UART closed and both pins as inputs. The caller must reconfigure.
void probeGpsLines(HardwareSerial& uart, int rxPin, int txPin, Print& out);

/// Opens the port at each of the usual receiver baud rates in turn and reports
/// what arrives. Only worth running once the line test says something is
/// talking. The caller must reconfigure the receiver afterwards, since this
/// leaves the UART on whatever rate it tried last.
void probeGpsBaudRates(HardwareSerial& uart, int rxPin, int txPin, Print& out);

} // namespace diag

#endif // KARTGPS_DIAGNOSTICS_H
