// SPDX-License-Identifier: MIT
//
// The telemetry pipeline: everything that happens between "a sensor produced a
// reading" and "a BLE notification went out", with no BLE stack in sight.
//
// The transport is behind the PacketSink interface, so the integration tests
// drive the real pipeline with a recording sink and assert on the exact bytes
// RaceChrono would have received.

#ifndef KARTGPS_PIPELINE_H
#define KARTGPS_PIPELINE_H

#include <stddef.h>
#include <stdint.h>

#include "button.h"
#include "channels.h"
#include "rc_protocol.h"
#include "session_timer.h"
#include "telemetry_types.h"

namespace kart {

/// Whatever carries the packets. Implemented by the NimBLE server on the
/// device and by a recorder in the tests.
class PacketSink {
public:
  virtual ~PacketSink() = default;

  virtual bool isConnected() const = 0;
  virtual bool notifyGpsMain(const uint8_t* data, size_t length) = 0;
  virtual bool notifyGpsTime(const uint8_t* data, size_t length) = 0;
  virtual bool notifyCan(const uint8_t* data, size_t length) = 0;
};

class TelemetryPipeline {
public:
  struct Config {
    /// Upper bound on the motion channel rate, independent of how fast the IMU
    /// is sampled and of what interval RaceChrono asks for.
    uint16_t motionPublishHz = 25;
    uint16_t statusPublishMs = 1000;
    /// A fix older than this stops counting as a fix in the status flags.
    uint32_t gnssStaleAfterMs = 3000;
  };

  struct Stats {
    uint32_t gpsMainSent = 0;
    uint32_t gpsTimeSent = 0;
    uint32_t motionSent = 0;
    uint32_t statusSent = 0;
    uint32_t droppedDisconnected = 0;
    uint32_t droppedByFilter = 0;
    uint32_t droppedByRate = 0;
    uint32_t transportErrors = 0;
  };

  explicit TelemetryPipeline(PacketSink& sink);
  TelemetryPipeline(PacketSink& sink, const Config& config);

  // --- Inputs ---------------------------------------------------------------

  /// Publishes one GNSS fix on characteristics 0x0003 and, when the hour
  /// rolled over, 0x0004.
  void onGnssFix(const telemetry::GnssFix& fix, uint32_t nowMs);

  /// Publishes one processed IMU sample on the motion channel, subject to the
  /// configured rate cap and to whatever RaceChrono asked for.
  void onMotionSample(const telemetry::ImuSample& sample, uint32_t nowMs);

  /// Periodic housekeeping: emits the status channel.
  void tick(uint32_t nowMs);

  /// Forwards a write on the CAN filter characteristic (0x0002).
  bool onCanFilterWrite(const uint8_t* data, size_t length);

  // --- Connection lifecycle -------------------------------------------------

  void onConnect();
  void onDisconnect();

  // --- SET button actions ---------------------------------------------------

  /// Zeroes the session clock and the GPS sync state.
  void resetSession(uint32_t nowMs);

  // --- Status inputs --------------------------------------------------------

  void setImuCalibrated(bool calibrated);
  void setCalibrationRunning(bool running);
  void setCalibrationRejected(bool rejected);
  void setDeviceTemperature(float celsius);

  // --- Accessors ------------------------------------------------------------

  const Stats& stats() const { return stats_; }
  SessionTimer& session() { return session_; }
  const SessionTimer& session() const { return session_; }
  rc::CanFilter& canFilter() { return filter_; }
  rc::GpsEncoder& gpsEncoder() { return gpsEncoder_; }

  /// True when no fix has arrived within Config::gnssStaleAfterMs.
  bool isGnssStale(uint32_t nowMs) const;

  uint8_t statusFlags(uint32_t nowMs) const;

private:
  bool publishCan(uint32_t pid, const uint8_t* payload, size_t payloadLength, uint32_t nowMs);

  PacketSink& sink_;
  Config config_;
  Stats stats_;

  rc::GpsEncoder gpsEncoder_;
  rc::CanFilter filter_;
  SessionTimer session_;
  RateLimiter motionRate_;
  RateLimiter statusRate_;

  telemetry::GnssFix lastFix_;
  bool hasFix_ = false;
  uint32_t lastFixMs_ = 0;

  bool imuCalibrated_ = false;
  bool calibrationRunning_ = false;
  bool calibrationRejected_ = false;
  float deviceTemperatureC_ = 0.0f;
};

} // namespace kart

#endif // KARTGPS_PIPELINE_H
