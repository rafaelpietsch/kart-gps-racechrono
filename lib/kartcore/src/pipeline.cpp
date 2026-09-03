// SPDX-License-Identifier: MIT

#include "pipeline.h"

namespace kart {
namespace {

uint32_t intervalForHz(uint16_t hertz) {
  if (hertz == 0) {
    return 0;
  }
  return 1000u / hertz;
}

} // namespace

TelemetryPipeline::TelemetryPipeline(PacketSink& sink) : TelemetryPipeline(sink, Config()) {}

TelemetryPipeline::TelemetryPipeline(PacketSink& sink, const Config& config)
    : sink_(sink), config_(config), motionRate_(intervalForHz(config.motionPublishHz)),
      statusRate_(config.statusPublishMs) {}

void TelemetryPipeline::onConnect() {
  // RaceChrono re-sends its filter after every connect, so start from the
  // default "deny all" rather than trusting state from the previous session.
  filter_.reset();
  motionRate_.reset();
  statusRate_.reset();
}

void TelemetryPipeline::onDisconnect() {
  filter_.reset();
}

void TelemetryPipeline::resetSession(uint32_t nowMs) {
  session_.reset(nowMs);
  // Forcing the sync counter to restart makes the app discard whatever it had
  // half-assembled from the previous session.
  gpsEncoder_.reset();
  motionRate_.reset();
  statusRate_.reset();
}

void TelemetryPipeline::setImuCalibrated(bool calibrated) {
  imuCalibrated_ = calibrated;
  if (calibrated) {
    calibrationRejected_ = false;
  }
}

void TelemetryPipeline::setCalibrationRunning(bool running) {
  calibrationRunning_ = running;
  if (running) {
    calibrationRejected_ = false;
  }
}

void TelemetryPipeline::setCalibrationRejected(bool rejected) {
  calibrationRejected_ = rejected;
}

void TelemetryPipeline::setDeviceTemperature(float celsius) {
  deviceTemperatureC_ = celsius;
}

bool TelemetryPipeline::isGnssStale(uint32_t nowMs) const {
  if (!hasFix_) {
    return true;
  }
  return static_cast<uint32_t>(nowMs - lastFixMs_) > config_.gnssStaleAfterMs;
}

uint8_t TelemetryPipeline::statusFlags(uint32_t nowMs) const {
  uint8_t flags = 0;
  if (imuCalibrated_) {
    flags |= channels::kFlagImuCalibrated;
  }
  if (hasFix_ && lastFix_.navigationValid && !isGnssStale(nowMs)) {
    flags |= channels::kFlagGnssFixValid;
  }
  if (calibrationRunning_) {
    flags |= channels::kFlagCalibrationRunning;
  }
  if (calibrationRejected_) {
    flags |= channels::kFlagCalibrationRejected;
  }
  if (isGnssStale(nowMs)) {
    flags |= channels::kFlagGnssStale;
  }
  return flags;
}

void TelemetryPipeline::onGnssFix(const telemetry::GnssFix& fix, uint32_t nowMs) {
  lastFix_ = fix;
  hasFix_ = true;
  lastFixMs_ = nowMs;

  if (!sink_.isConnected()) {
    ++stats_.droppedDisconnected;
    return;
  }

  const rc::GpsPackets packets = gpsEncoder_.encode(fix);

  if (!sink_.notifyGpsMain(packets.main, rc::kGpsMainPacketSize)) {
    ++stats_.transportErrors;
  } else {
    ++stats_.gpsMainSent;
  }

  // 0x0004 only has to change when the hour does; sending it every epoch would
  // waste a notification slot that the motion channel needs.
  if (packets.timeChanged) {
    if (!sink_.notifyGpsTime(packets.time, rc::kGpsTimePacketSize)) {
      ++stats_.transportErrors;
    } else {
      ++stats_.gpsTimeSent;
    }
  }
}

bool TelemetryPipeline::publishCan(uint32_t pid, const uint8_t* payload, size_t payloadLength,
                                   uint32_t nowMs) {
  if (!filter_.shouldNotify(pid, nowMs)) {
    ++stats_.droppedByFilter;
    return false;
  }
  uint8_t packet[rc::kCanMaxPacketSize];
  const size_t written = rc::encodeCanPacket(pid, payload, payloadLength, packet);
  if (written == 0) {
    ++stats_.transportErrors;
    return false;
  }
  if (!sink_.notifyCan(packet, written)) {
    ++stats_.transportErrors;
    return false;
  }
  return true;
}

void TelemetryPipeline::onMotionSample(const telemetry::ImuSample& sample, uint32_t nowMs) {
  if (!sink_.isConnected()) {
    ++stats_.droppedDisconnected;
    return;
  }
  if (!motionRate_.tryAcquire(nowMs)) {
    ++stats_.droppedByRate;
    return;
  }

  uint8_t payload[channels::kMotionPayloadSize];
  const size_t length = channels::encodeMotionPayload(sample, payload, sizeof(payload));
  if (length == 0) {
    ++stats_.transportErrors;
    return;
  }
  if (publishCan(channels::kPidMotion, payload, length, nowMs)) {
    ++stats_.motionSent;
  }
}

void TelemetryPipeline::tick(uint32_t nowMs) {
  if (!sink_.isConnected()) {
    return;
  }
  if (!statusRate_.tryAcquire(nowMs)) {
    return;
  }

  channels::DeviceStatus status;
  status.sessionTimeMs = session_.elapsedMs(nowMs);
  status.satellites = lastFix_.satellites;
  status.satellitesValid = hasFix_ && lastFix_.satellitesValid && !isGnssStale(nowMs);
  status.fixQuality = (hasFix_ && !isGnssStale(nowMs)) ? lastFix_.fixQuality : 0;
  status.flags = statusFlags(nowMs);
  status.temperatureC = deviceTemperatureC_;
  status.sessionResets = session_.resetCount();

  uint8_t payload[channels::kStatusPayloadSize];
  const size_t length = channels::encodeStatusPayload(status, payload, sizeof(payload));
  if (length == 0) {
    ++stats_.transportErrors;
    return;
  }
  if (publishCan(channels::kPidStatus, payload, length, nowMs)) {
    ++stats_.statusSent;
  }
}

bool TelemetryPipeline::onCanFilterWrite(const uint8_t* data, size_t length) {
  return filter_.handleWrite(data, length);
}

} // namespace kart
