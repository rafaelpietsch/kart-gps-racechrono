// SPDX-License-Identifier: MIT

#include "gps_receiver.h"

#include "ublox.h"

namespace hal {
namespace {

/// A UBX frame is at most a CFG-NAV5 payload plus the 8 byte envelope.
constexpr size_t kUbxBufferSize = 64;

/// The receiver needs a moment to act on a configuration frame before the next
/// one arrives, and a baud change needs the UART to drain first.
constexpr uint32_t kConfigSettleMs = 60;
constexpr uint32_t kBaudChangeSettleMs = 150;

/// A generous RX buffer: at 115200 baud a 5 Hz burst of RMC+GGA+GSA is around
/// 200 bytes, and the loop can be busy with BLE work when it lands.
constexpr size_t kRxBufferSize = 1024;

} // namespace

void GpsReceiver::sendUbx(const uint8_t* frame, size_t length) {
  if (length == 0) {
    return;
  }
  uart_.write(frame, length);
  uart_.flush();
  delay(kConfigSettleMs);
}

void GpsReceiver::setMessageRate(uint8_t messageClass, uint8_t messageId, uint8_t rate) {
  uint8_t frame[kUbxBufferSize];
  const size_t length = ublox::buildCfgMsgForPort(messageClass, messageId, ublox::kPortUart1, rate,
                                                  frame, sizeof(frame));
  sendUbx(frame, length);
}

void GpsReceiver::begin(const Config& config) {
  config_ = config;

  uart_.setRxBufferSize(kRxBufferSize);
  uart_.begin(config.bootBaud, SERIAL_8N1, config.rxPin, config.txPin);
  delay(kBaudChangeSettleMs);

  // Raise the port speed first: the rest of the configuration is small, but the
  // NMEA stream that follows it will not fit in 9600 baud at 5 Hz.
  uint8_t frame[kUbxBufferSize];
  size_t length = ublox::buildCfgPrtUart1(config.runBaud, frame, sizeof(frame));
  sendUbx(frame, length);

  uart_.flush();
  uart_.end();
  uart_.setRxBufferSize(kRxBufferSize);
  uart_.begin(config.runBaud, SERIAL_8N1, config.rxPin, config.txPin);
  delay(kBaudChangeSettleMs);

  // A receiver that ignored the baud change (already reconfigured, or a clone
  // with different defaults) would leave us reading noise. Sending the port
  // command again at the new rate is harmless and makes the state definite.
  length = ublox::buildCfgPrtUart1(config.runBaud, frame, sizeof(frame));
  sendUbx(frame, length);

  applyConfiguration();
}

void GpsReceiver::applyConfiguration() {
  // Only the three sentences the parser consumes stay on. GSV in particular is
  // several lines per epoch and would eat the whole link budget.
  setMessageRate(ublox::kClassNmea, ublox::kIdNmeaGga, 1);
  setMessageRate(ublox::kClassNmea, ublox::kIdNmeaRmc, 1);
  setMessageRate(ublox::kClassNmea, ublox::kIdNmeaGsa, 1);
  setMessageRate(ublox::kClassNmea, ublox::kIdNmeaGll, 0);
  setMessageRate(ublox::kClassNmea, ublox::kIdNmeaVtg, 0);
  setMessageRate(ublox::kClassNmea, ublox::kIdNmeaGsv, 0);
  setMessageRate(ublox::kClassNmea, ublox::kIdNmeaGbs, 0);

  uint8_t frame[kUbxBufferSize];
  size_t length =
      ublox::buildCfgNav5DynamicModel(ublox::DynamicModel::kAutomotive, frame, sizeof(frame));
  sendUbx(frame, length);

  length = ublox::buildCfgRateHz(config_.rateHz, frame, sizeof(frame));
  sendUbx(frame, length);

  // Anything the receiver emitted while it was being reconfigured is a mix of
  // old and new settings, so start the parser from a clean slate.
  while (uart_.available() > 0) {
    uart_.read();
  }
  assembler_.reset();
  fixes_.reset();
}

void GpsReceiver::requestHotStart() {
  uint8_t frame[kUbxBufferSize];
  const size_t length = ublox::buildCfgRstHotStart(frame, sizeof(frame));
  uart_.write(frame, length);
  uart_.flush();
}

bool GpsReceiver::poll(uint32_t nowMs) {
  bool completed = false;
  // Bound the work per call so a backlog cannot starve the IMU and BLE.
  int budget = 512;
  while (uart_.available() > 0 && budget-- > 0) {
    if (!assembler_.push(static_cast<char>(uart_.read()))) {
      continue;
    }
    fixes_.consume(assembler_.sentence(), assembler_.length());
    telemetry::GnssFix fix;
    if (fixes_.takeCompletedFix(fix)) {
      fix.receivedAtMs = nowMs;
      fix_ = fix;
      ++fixCount_;
      completed = true;
    }
  }
  return completed;
}

} // namespace hal
