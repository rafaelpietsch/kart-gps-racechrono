// SPDX-License-Identifier: MIT

#include "diagnostics.h"

#include <math.h>

#include "imu.h"

namespace diag {
namespace {

/// Registers worth seeing when the sensor is not behaving. Anything that is
/// written during begin() is here, so a mismatch between what we asked for and
/// what the device holds is visible at a glance.
struct NamedRegister {
  uint8_t address;
  const char* name;
};

constexpr NamedRegister kMpuRegisters[] = {
    {imu::reg::kWhoAmI, "WHO_AM_I"},
    {imu::reg::kPowerManagement1, "PWR_MGMT_1"},
    {imu::reg::kSampleRateDivider, "SMPRT_DIV"},
    {imu::reg::kConfig, "CONFIG"},
    {imu::reg::kGyroConfig, "GYRO_CONFIG"},
    {imu::reg::kAccelConfig, "ACCEL_CONFIG"},
    {imu::reg::kIntEnable, "INT_ENABLE"},
    {imu::reg::kIntStatus, "INT_STATUS"},
};

void printHexByte(uint8_t value, Print& out) {
  out.print("0x");
  if (value < 0x10) {
    out.print('0');
  }
  out.print(value, HEX);
}

} // namespace

const char* wireErrorName(uint8_t code) {
  switch (code) {
    case 0:
      return "ok";
    case 1:
      return "data too long";
    case 2:
      return "NACK on address (nothing there)";
    case 3:
      return "NACK on data";
    case 4:
      return "other error";
    case 5:
      return "timeout (bus held low?)";
    default:
      return "unknown";
  }
}

void reportBusLines(int sdaPin, int sclPin, Print& out) {
  // The I2C pads keep their input path enabled while the peripheral drives
  // them, so the GPIO input register still reflects the real pad level.
  const bool sdaHigh = digitalRead(sdaPin) == HIGH;
  const bool sclHigh = digitalRead(sclPin) == HIGH;

  out.printf("[i2c] idle levels: SDA(GPIO%d)=%s SCL(GPIO%d)=%s\n", sdaPin, sdaHigh ? "HIGH" : "LOW",
             sclPin, sclHigh ? "HIGH" : "LOW");
  if (!sdaHigh || !sclHigh) {
    out.println("[i2c] a line stuck LOW means no pull-up, a short to ground, or a "
                "device holding the bus. Nothing will be found until that clears.");
  }
}

I2cScanResult scanI2c(TwoWire& wire, Print& out) {
  I2cScanResult result;
  out.println("[i2c] scanning 0x08..0x77");

  for (uint8_t address = 0x08; address <= 0x77; ++address) {
    wire.beginTransmission(address);
    const uint8_t error = wire.endTransmission();
    if (error == 0) {
      out.print("[i2c]   found device at ");
      printHexByte(address, out);
      if (address == imu::kI2cAddressLow) {
        out.print("  (MPU-6050, AD0 low)");
      } else if (address == imu::kI2cAddressHigh) {
        out.print("  (MPU-6050, AD0 high)");
      }
      out.println();
      if (result.foundCount < I2cScanResult::kMaxFound) {
        result.found[result.foundCount++] = address;
      }
    } else if (error != 2) {
      ++result.busErrors;
    }
  }

  out.printf("[i2c] scan done: %u device(s), %u non-NACK bus errors\n",
             static_cast<unsigned>(result.foundCount), static_cast<unsigned>(result.busErrors));
  if (result.foundCount == 0 && result.busErrors == 0) {
    out.println("[i2c] every address NACKed cleanly: the bus works, but nothing "
                "is answering. Check VCC/GND on the sensor and the SDA/SCL pair.");
  } else if (result.busErrors > 0) {
    out.println("[i2c] bus errors on most addresses point at the wiring, not at "
                "the sensor: swapped SDA/SCL, or a line that never releases.");
  }
  return result;
}

bool readRegisters(TwoWire& wire, uint8_t address, uint8_t reg, uint8_t* buffer, size_t length,
                   uint8_t& errorOut) {
  wire.beginTransmission(address);
  wire.write(reg);
  errorOut = wire.endTransmission(false);
  if (errorOut != 0) {
    return false;
  }
  const size_t received = wire.requestFrom(address, static_cast<uint8_t>(length));
  if (received != length) {
    errorOut = 4;
    return false;
  }
  for (size_t i = 0; i < length; ++i) {
    buffer[i] = static_cast<uint8_t>(wire.read());
  }
  return true;
}

void dumpMpuRegisters(TwoWire& wire, uint8_t address, Print& out,
                      const imu::RawAccelBias& trim) {
  out.print("[i2c] MPU-6050 register dump at ");
  printHexByte(address, out);
  out.println();

  for (const NamedRegister& entry : kMpuRegisters) {
    uint8_t value = 0;
    uint8_t error = 0;
    if (!readRegisters(wire, address, entry.address, &value, 1, error)) {
      out.printf("[i2c]   %-13s read failed: %s\n", entry.name, wireErrorName(error));
      continue;
    }
    out.printf("[i2c]   %-13s (0x%02X) = 0x%02X\n", entry.name, entry.address, value);
    if (entry.address == imu::reg::kWhoAmI && value != imu::kWhoAmIValue) {
      out.printf("[i2c]   WHO_AM_I should be 0x%02X. A different value is usually a "
                 "clone chip or a read landing on the wrong device.\n",
                 imu::kWhoAmIValue);
    }
    if (entry.address == imu::reg::kPowerManagement1 && (value & 0x40) != 0) {
      out.println("[i2c]   SLEEP bit is set: the sensor is asleep and its output "
                  "registers will not update.");
    }
  }

  uint8_t burst[imu::kBurstLength];
  uint8_t error = 0;
  if (!readRegisters(wire, address, imu::reg::kAccelXoutH, burst, sizeof(burst), error)) {
    out.printf("[i2c]   burst read failed: %s\n", wireErrorName(error));
    return;
  }

  out.print("[i2c]   burst 0x3B..0x48 =");
  for (uint8_t byte : burst) {
    out.print(' ');
    if (byte < 0x10) {
      out.print('0');
    }
    out.print(byte, HEX);
  }
  out.println();

  telemetry::ImuRawSample raw;
  if (!imu::decodeBurst(burst, sizeof(burst), raw)) {
    out.println("[i2c]   burst did not decode");
    return;
  }
  // The dump is read straight off the wire, so it is deliberately reported at
  // the ranges the driver configures rather than at whatever it currently holds.
  const telemetry::ImuSample sample =
      imu::toPhysicalUnits(raw, imu::AccelRange::k4G, imu::GyroRange::k500Dps);
  float gravity =
      sqrtf(sample.accelG[0] * sample.accelG[0] + sample.accelG[1] * sample.accelG[1] +
            sample.accelG[2] * sample.accelG[2]);

  out.printf("[i2c]   accel %.3f %.3f %.3f g  |a|=%.3f g\n", sample.accelG[0], sample.accelG[1],
             sample.accelG[2], gravity);
  out.printf("[i2c]   gyro  %.2f %.2f %.2f dps   temp %.1f C\n", sample.gyroDps[0],
             sample.gyroDps[1], sample.gyroDps[2], sample.temperatureC);

  if (!trim.isZero()) {
    // The dump bypasses the driver, so the configured trim has to be applied
    // here or these numbers will disagree with a zeroing that just passed.
    telemetry::ImuRawSample corrected = raw;
    imu::applyRawAccelBias(corrected, trim);
    const telemetry::ImuSample fixed =
        imu::toPhysicalUnits(corrected, imu::AccelRange::k4G, imu::GyroRange::k500Dps);
    const float fixedGravity =
        sqrtf(fixed.accelG[0] * fixed.accelG[0] + fixed.accelG[1] * fixed.accelG[1] +
              fixed.accelG[2] * fixed.accelG[2]);
    out.printf("[i2c]   with the configured trim (%+d %+d %+d): %.3f %.3f %.3f g  |a|=%.3f g\n",
               trim.count[0], trim.count[1], trim.count[2], fixed.accelG[0], fixed.accelG[1],
               fixed.accelG[2], fixedGravity);
    gravity = fixedGravity;
  }

  if (gravity < 0.85f || gravity > 1.15f) {
    out.println("[i2c]   a device at rest should read about 1.000 g in total. "
                "This is outside the window the zeroing accepts.");
  }
  const bool allZero = raw.accel[0] == 0 && raw.accel[1] == 0 && raw.accel[2] == 0;
  if (allZero) {
    out.println("[i2c]   every accel axis reads exactly zero: the sensor is "
                "answering but not converting (asleep, or held in reset).");
  }
}

// --- GPS link ---------------------------------------------------------------

namespace {

/// Baud rates worth trying: the NEO-6M's factory default, the rate this
/// firmware moves it to, and the one in between that clone modules often ship
/// configured for.
constexpr uint32_t kCandidateBauds[] = {9600, 38400, 115200};

constexpr uint32_t kLineSampleMs = 300;
constexpr uint32_t kBaudListenMs = 1200;

} // namespace

namespace {

/// Samples one pin held down by an internal pull-down, reporting whether
/// anything external overpowers it and how often it moves.
struct LineState {
  bool drivenHigh = false;
  uint32_t transitions = 0;
};

LineState sampleLine(int pin, uint32_t forMs) {
  LineState state;
  state.drivenHigh = digitalRead(pin) == HIGH;
  int last = digitalRead(pin);
  const uint32_t deadline = millis() + forMs;
  while (millis() < deadline) {
    const int now = digitalRead(pin);
    if (now != last) {
      ++state.transitions;
      last = now;
    }
  }
  return state;
}

const char* describeLine(const LineState& state) {
  if (state.transitions > 0) {
    return "toggling, something is transmitting";
  }
  return state.drivenHigh ? "held HIGH, driven but idle" : "LOW, nothing is driving it";
}

} // namespace

void probeGpsLines(HardwareSerial& uart, int rxPin, int txPin, Print& out) {
  out.println("[gps ] testing both GPS pins as plain inputs");

  uart.end();
  pinMode(rxPin, INPUT_PULLDOWN);
  pinMode(txPin, INPUT_PULLDOWN);
  delay(20);

  const LineState rx = sampleLine(rxPin, kLineSampleMs);
  const LineState tx = sampleLine(txPin, kLineSampleMs);

  out.printf("[gps ]   GPIO%d (our RX, expects the module TX): %s, %lu edges\n", rxPin,
             describeLine(rx), static_cast<unsigned long>(rx.transitions));
  out.printf("[gps ]   GPIO%d (our TX, expects the module RX): %s, %lu edges\n", txPin,
             describeLine(tx), static_cast<unsigned long>(tx.transitions));

  const bool rxAlive = rx.drivenHigh || rx.transitions > 0;
  const bool txAlive = tx.drivenHigh || tx.transitions > 0;

  if (!rxAlive && txAlive) {
    out.println("[gps ] the module is driving the pin we transmit on, and nothing is "
                "driving the pin we listen on. TX and RX are swapped: the module TX "
                "belongs on our RX pin and its RX on our TX pin.");
    return;
  }
  if (!rxAlive && !txAlive) {
    out.println("[gps ] neither pin is driven by anything. A powered receiver holds its "
                "TX high and would overpower the pull-down on whichever pin it reached, "
                "so this is not a swap: the module has no 3V3, or its GND is not shared "
                "with the board, or neither wire is landing on these pins.");
    return;
  }
  if (rxAlive && rx.transitions == 0) {
    out.println("[gps ] our RX pin is held high but never moves: the module is powered "
                "and connected, yet silent. A NEO-6M sends sentences from power-up even "
                "with no satellites, so this is a module configured mute, or a dead one.");
    return;
  }
  out.println("[gps ] our RX pin is toggling, so the wiring and power are good. Whatever "
              "is wrong is the rate or the framing.");
}

void probeGpsBaudRates(HardwareSerial& uart, int rxPin, int txPin, Print& out) {
  out.println("[gps ] listening at each candidate baud rate");
  out.println("[gps ] this blocks for a few seconds and will drop a live BLE session");

  for (uint32_t baud : kCandidateBauds) {
    uart.end();
    delay(20);
    uart.begin(baud, SERIAL_8N1, rxPin, txPin);
    delay(20);
    while (uart.available() > 0) {
      uart.read();
    }

    uint32_t bytes = 0;
    uint32_t dollars = 0;
    uint32_t printable = 0;
    const uint32_t deadline = millis() + kBaudListenMs;
    while (millis() < deadline) {
      while (uart.available() > 0) {
        const int value = uart.read();
        ++bytes;
        if (value == '$') {
          ++dollars;
        }
        if (value == '\n' || value == '\r' || (value >= 0x20 && value < 0x7F)) {
          ++printable;
        }
      }
    }

    out.printf("[gps ]   %6lu baud: %lu bytes, %lu printable, %lu NMEA starts\n",
               static_cast<unsigned long>(baud), static_cast<unsigned long>(bytes),
               static_cast<unsigned long>(printable), static_cast<unsigned long>(dollars));

    if (bytes > 0 && dollars > 0) {
      out.println("[gps ]   ^ this is the rate the receiver is actually using");
    } else if (bytes > 0 && printable * 4 < bytes * 3) {
      out.println("[gps ]   ^ bytes arrive but most are not text: wrong rate, framing "
                  "noise from a neighbouring rate");
    }
  }

  out.println("[gps ] a row with bytes and NMEA starts is the receiver's real rate. All "
              "rows empty means nothing is being transmitted at all.");
}

} // namespace diag
