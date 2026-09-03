// SPDX-License-Identifier: MIT
//
// UART driver for the GY-NEO6MV2, plus the start-up configuration sequence.

#ifndef KARTGPS_HAL_GPS_RECEIVER_H
#define KARTGPS_HAL_GPS_RECEIVER_H

#include <Arduino.h>
#include <HardwareSerial.h>

#include "nmea.h"
#include "telemetry_types.h"

namespace hal {

/// Owns the GPS UART, configures the receiver and turns bytes into fixes.
class GpsReceiver {
public:
  struct Config {
    int rxPin;
    int txPin;
    uint32_t bootBaud;
    uint32_t runBaud;
    uint16_t rateHz;
  };

  explicit GpsReceiver(HardwareSerial& uart) : uart_(uart) {}

  /// Opens the port, walks the receiver up to the run baud rate and applies
  /// the message, rate and dynamic model configuration.
  void begin(const Config& config);

  /// Drains the UART. Returns true when a complete fix is ready in fix().
  /// Call it every loop iteration: at 115200 baud the RX FIFO fills in a few
  /// milliseconds.
  bool poll(uint32_t nowMs);

  const telemetry::GnssFix& fix() const { return fix_; }

  /// Asks the receiver for a hot restart, keeping the stored ephemeris so it
  /// reacquires in seconds rather than minutes.
  void requestHotStart();

  /// Re-applies the full configuration. Used when the receiver has been power
  /// cycled behind our back, or after a hot start.
  void applyConfiguration();

  const nmea::SentenceAssembler::Stats& sentenceStats() const { return assembler_.stats(); }
  uint32_t fixCount() const { return fixCount_; }

private:
  void sendUbx(const uint8_t* frame, size_t length);
  void setMessageRate(uint8_t messageClass, uint8_t messageId, uint8_t rate);

  HardwareSerial& uart_;
  Config config_ = {};
  nmea::SentenceAssembler assembler_;
  nmea::FixAssembler fixes_;
  telemetry::GnssFix fix_;
  uint32_t fixCount_ = 0;
};

} // namespace hal

#endif // KARTGPS_HAL_GPS_RECEIVER_H
