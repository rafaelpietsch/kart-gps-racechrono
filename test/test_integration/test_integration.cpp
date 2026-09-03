// SPDX-License-Identifier: MIT
//
// Integration tests: raw sensor bytes in, RaceChrono BLE packets out.
//
// These run the real parsers, the real encoders and the real pipeline. Only the
// BLE transport is substituted, by a sink that records exactly what would have
// gone over the air, so the assertions are on wire bytes rather than on
// internal state.

#include <unity.h>

#include <stdio.h>
#include <string.h>

#include <vector>

#include "channels.h"
#include "imu.h"
#include "nmea.h"
#include "pipeline.h"
#include "rc_protocol.h"

namespace {

// --- Test doubles -----------------------------------------------------------

struct Packet {
  std::vector<uint8_t> bytes;
  uint32_t atMs = 0;
};

/// Stands in for the NimBLE server and remembers every notification.
class RecordingSink : public kart::PacketSink {
public:
  bool isConnected() const override { return connected_; }

  bool notifyGpsMain(const uint8_t* data, size_t length) override {
    return record(gpsMain_, data, length);
  }
  bool notifyGpsTime(const uint8_t* data, size_t length) override {
    return record(gpsTime_, data, length);
  }
  bool notifyCan(const uint8_t* data, size_t length) override {
    return record(can_, data, length);
  }

  void setConnected(bool connected) { connected_ = connected; }
  void setFailing(bool failing) { failing_ = failing; }
  void setClock(uint32_t nowMs) { clockMs_ = nowMs; }

  const std::vector<Packet>& gpsMain() const { return gpsMain_; }
  const std::vector<Packet>& gpsTime() const { return gpsTime_; }
  const std::vector<Packet>& can() const { return can_; }

  /// CAN notifications whose little-endian packet ID matches `pid`.
  std::vector<Packet> canFor(uint32_t pid) const {
    std::vector<Packet> matches;
    for (const Packet& packet : can_) {
      if (packet.bytes.size() < 4) {
        continue;
      }
      const uint32_t packetId = static_cast<uint32_t>(packet.bytes[0]) |
                                (static_cast<uint32_t>(packet.bytes[1]) << 8) |
                                (static_cast<uint32_t>(packet.bytes[2]) << 16) |
                                (static_cast<uint32_t>(packet.bytes[3]) << 24);
      if (packetId == pid) {
        matches.push_back(packet);
      }
    }
    return matches;
  }

  size_t totalBytes() const {
    size_t total = 0;
    for (const Packet& p : gpsMain_) total += p.bytes.size();
    for (const Packet& p : gpsTime_) total += p.bytes.size();
    for (const Packet& p : can_) total += p.bytes.size();
    return total;
  }

  size_t totalNotifications() const { return gpsMain_.size() + gpsTime_.size() + can_.size(); }

  void clear() {
    gpsMain_.clear();
    gpsTime_.clear();
    can_.clear();
  }

private:
  bool record(std::vector<Packet>& into, const uint8_t* data, size_t length) {
    if (failing_) {
      return false;
    }
    Packet packet;
    packet.bytes.assign(data, data + length);
    packet.atMs = clockMs_;
    into.push_back(packet);
    return true;
  }

  bool connected_ = true;
  bool failing_ = false;
  uint32_t clockMs_ = 0;
  std::vector<Packet> gpsMain_;
  std::vector<Packet> gpsTime_;
  std::vector<Packet> can_;
};

// --- Decoders, so assertions read as values rather than as byte offsets -----

int32_t decodeInt32(const uint8_t* data) {
  return static_cast<int32_t>((static_cast<uint32_t>(data[0]) << 24) |
                              (static_cast<uint32_t>(data[1]) << 16) |
                              (static_cast<uint32_t>(data[2]) << 8) | data[3]);
}

uint16_t decodeUint16(const uint8_t* data) {
  return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}

int16_t decodeInt16(const uint8_t* data) {
  return static_cast<int16_t>(decodeUint16(data));
}

uint32_t decodeUint32(const uint8_t* data) {
  return (static_cast<uint32_t>(data[0]) << 24) | (static_cast<uint32_t>(data[1]) << 16) |
         (static_cast<uint32_t>(data[2]) << 8) | data[3];
}

// --- Recorded NMEA, one 5 Hz epoch per group --------------------------------

const char kEpoch1[] =
    "$GPGGA,143512.50,2330.0000,S,04636.0000,W,1,09,0.9,760.5,M,-6.5,M,,*79\r\n"
    "$GPGSA,A,3,04,05,,09,12,,,24,,,,,1.7,0.9,1.4*35\r\n"
    "$GPRMC,143512.50,A,2330.0000,S,04636.0000,W,49.86,271.25,150326,,,A*62\r\n";
const char kEpoch2[] =
    "$GPGGA,143513.00,2330.0000,S,04636.0000,W,1,09,0.9,760.5,M,-6.5,M,,*7D\r\n"
    "$GPRMC,143513.00,A,2330.0000,S,04636.0000,W,49.86,271.25,150326,,,A*66\r\n";
// Same day, one hour later, which is what advances the sync counter.
const char kEpochNextHour[] =
    "$GPGGA,153512.50,2330.0000,S,04636.0000,W,1,09,0.9,760.5,M,-6.5,M,,*78\r\n"
    "$GPRMC,153512.50,A,2330.0000,S,04636.0000,W,49.86,271.25,150326,,,A*63\r\n";

/// Feeds NMEA text through the assemblers and pushes every completed fix into
/// the pipeline. Returns how many fixes were published.
size_t feedNmea(nmea::SentenceAssembler& assembler, nmea::FixAssembler& fixes,
                kart::TelemetryPipeline& pipeline, const char* text, uint32_t nowMs) {
  size_t published = 0;
  for (const char* p = text; *p != '\0'; ++p) {
    if (!assembler.push(*p)) {
      continue;
    }
    fixes.consume(assembler.sentence(), assembler.length());
    telemetry::GnssFix fix;
    if (fixes.takeCompletedFix(fix)) {
      pipeline.onGnssFix(fix, nowMs);
      ++published;
    }
  }
  return published;
}

/// The filter write RaceChrono sends to subscribe to one packet ID.
void allowPid(kart::TelemetryPipeline& pipeline, uint32_t pid, uint16_t intervalMs) {
  const uint8_t write[7] = {
      2,
      static_cast<uint8_t>(intervalMs >> 8),
      static_cast<uint8_t>(intervalMs),
      static_cast<uint8_t>(pid >> 24),
      static_cast<uint8_t>(pid >> 16),
      static_cast<uint8_t>(pid >> 8),
      static_cast<uint8_t>(pid),
  };
  TEST_ASSERT_TRUE(pipeline.onCanFilterWrite(write, sizeof(write)));
}

/// A raw MPU-6050 burst for the given physical values, at the ranges the
/// firmware configures.
std::vector<uint8_t> makeBurst(float axG, float ayG, float azG, float gxDps, float gyDps,
                               float gzDps) {
  const float accelScale = imu::accelScaleLsbPerG(imu::AccelRange::k4G);
  const float gyroScale = imu::gyroScaleLsbPerDps(imu::GyroRange::k500Dps);
  const int16_t values[7] = {
      static_cast<int16_t>(axG * accelScale),   static_cast<int16_t>(ayG * accelScale),
      static_cast<int16_t>(azG * accelScale),   static_cast<int16_t>(0),
      static_cast<int16_t>(gxDps * gyroScale),  static_cast<int16_t>(gyDps * gyroScale),
      static_cast<int16_t>(gzDps * gyroScale),
  };
  std::vector<uint8_t> burst(imu::kBurstLength);
  for (size_t i = 0; i < 7; ++i) {
    const uint16_t raw = static_cast<uint16_t>(values[i]);
    burst[i * 2] = static_cast<uint8_t>(raw >> 8);
    burst[i * 2 + 1] = static_cast<uint8_t>(raw);
  }
  return burst;
}

/// Raw bytes to a processed, vehicle-frame sample.
telemetry::ImuSample processBurst(const imu::MotionProcessor& processor,
                                  const std::vector<uint8_t>& burst, uint32_t nowMs) {
  telemetry::ImuRawSample raw;
  imu::decodeBurst(burst.data(), burst.size(), raw);
  raw.timestampMs = nowMs;
  return processor.process(
      imu::toPhysicalUnits(raw, imu::AccelRange::k4G, imu::GyroRange::k500Dps));
}

} // namespace

void setUp(void) {}
void tearDown(void) {}

// --- GNSS path --------------------------------------------------------------

void test_recorded_nmea_becomes_a_correct_gps_notification(void) {
  RecordingSink sink;
  kart::TelemetryPipeline pipeline(sink);
  nmea::SentenceAssembler assembler;
  nmea::FixAssembler fixes;

  TEST_ASSERT_EQUAL_size_t(1, feedNmea(assembler, fixes, pipeline, kEpoch1, 1000));
  TEST_ASSERT_EQUAL_size_t(1, sink.gpsMain().size());

  const std::vector<uint8_t>& packet = sink.gpsMain()[0].bytes;
  TEST_ASSERT_EQUAL_size_t(rc::kGpsMainPacketSize, packet.size());

  // Fix quality 1, nine satellites.
  TEST_ASSERT_EQUAL_UINT8(1, (packet[3] >> 6) & 0x03);
  TEST_ASSERT_EQUAL_UINT8(9, packet[3] & 0x3F);
  // 23 deg 30.0000 min south, 46 deg 36.0000 min west.
  TEST_ASSERT_EQUAL_INT32(-235000000, decodeInt32(&packet[4]));
  TEST_ASSERT_EQUAL_INT32(-466000000, decodeInt32(&packet[8]));
  // 760.5 m in the fine altitude encoding.
  TEST_ASSERT_EQUAL_UINT16(12605, decodeUint16(&packet[12]));
  // 49.86 knots is 92.34 km/h, encoded in hundredths.
  TEST_ASSERT_UINT16_WITHIN(1, 9234, decodeUint16(&packet[14]));
  TEST_ASSERT_EQUAL_UINT16(27125, decodeUint16(&packet[16]));
  TEST_ASSERT_EQUAL_UINT8(9, packet[18]);  // HDOP 0.9
  TEST_ASSERT_EQUAL_UINT8(14, packet[19]); // VDOP 1.4 from GSA
}

void test_gps_time_characteristic_only_fires_when_the_hour_changes(void) {
  RecordingSink sink;
  kart::TelemetryPipeline pipeline(sink);
  nmea::SentenceAssembler assembler;
  nmea::FixAssembler fixes;

  feedNmea(assembler, fixes, pipeline, kEpoch1, 1000);
  TEST_ASSERT_EQUAL_size_t(1, sink.gpsTime().size());

  // Half a second later, same hour: main updates, time stays quiet.
  feedNmea(assembler, fixes, pipeline, kEpoch2, 1200);
  TEST_ASSERT_EQUAL_size_t(2, sink.gpsMain().size());
  TEST_ASSERT_EQUAL_size_t(1, sink.gpsTime().size());

  // An hour on, both characteristics move and carry the same sync bits.
  feedNmea(assembler, fixes, pipeline, kEpochNextHour, 2000);
  TEST_ASSERT_EQUAL_size_t(2, sink.gpsTime().size());
  const uint8_t mainSync = sink.gpsMain().back().bytes[0] & 0xE0;
  const uint8_t timeSync = sink.gpsTime().back().bytes[0] & 0xE0;
  TEST_ASSERT_EQUAL_HEX8(mainSync, timeSync);
}

void test_fixes_are_dropped_while_no_phone_is_connected(void) {
  RecordingSink sink;
  sink.setConnected(false);
  kart::TelemetryPipeline pipeline(sink);
  nmea::SentenceAssembler assembler;
  nmea::FixAssembler fixes;

  feedNmea(assembler, fixes, pipeline, kEpoch1, 1000);
  TEST_ASSERT_EQUAL_size_t(0, sink.gpsMain().size());
  TEST_ASSERT_EQUAL_UINT32(1, pipeline.stats().droppedDisconnected);
}

void test_a_failing_transport_is_counted_not_ignored(void) {
  RecordingSink sink;
  sink.setFailing(true);
  kart::TelemetryPipeline pipeline(sink);
  nmea::SentenceAssembler assembler;
  nmea::FixAssembler fixes;

  feedNmea(assembler, fixes, pipeline, kEpoch1, 1000);
  TEST_ASSERT_EQUAL_UINT32(0, pipeline.stats().gpsMainSent);
  TEST_ASSERT_TRUE(pipeline.stats().transportErrors > 0);
}

// --- Motion path ------------------------------------------------------------

void test_motion_is_silent_until_racechrono_subscribes(void) {
  RecordingSink sink;
  kart::TelemetryPipeline pipeline(sink);
  imu::MotionProcessor processor;

  const std::vector<uint8_t> burst = makeBurst(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
  pipeline.onMotionSample(processBurst(processor, burst, 0), 0);

  TEST_ASSERT_EQUAL_size_t(0, sink.can().size());
  TEST_ASSERT_EQUAL_UINT32(1, pipeline.stats().droppedByFilter);
}

void test_motion_flows_once_the_pid_is_allowed(void) {
  RecordingSink sink;
  kart::TelemetryPipeline pipeline(sink);
  imu::MotionProcessor processor;
  allowPid(pipeline, kart::channels::kPidMotion, 0);

  // Calibrate flat, then brake at 0.8 g while turning right at 30 deg/s.
  imu::BiasCalibrator calibrator;
  calibrator.begin(16);
  for (int i = 0; i < 16; ++i) {
    telemetry::ImuRawSample raw;
    imu::decodeBurst(makeBurst(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f).data(), imu::kBurstLength, raw);
    calibrator.addSample(imu::toPhysicalUnits(raw, imu::AccelRange::k4G, imu::GyroRange::k500Dps));
  }
  TEST_ASSERT_TRUE(calibrator.isComplete());
  processor.setBias(calibrator.bias());

  const std::vector<uint8_t> burst = makeBurst(-0.8f, 0.0f, 1.0f, 0.0f, 0.0f, 30.0f);
  pipeline.onMotionSample(processBurst(processor, burst, 100), 100);

  const std::vector<Packet> motion = sink.canFor(kart::channels::kPidMotion);
  TEST_ASSERT_EQUAL_size_t(1, motion.size());
  TEST_ASSERT_EQUAL_size_t(4 + kart::channels::kMotionPayloadSize, motion[0].bytes.size());

  const uint8_t* payload = motion[0].bytes.data() + 4;
  TEST_ASSERT_INT16_WITHIN(5, -800, decodeInt16(&payload[0])); // -0.8 g longitudinal
  TEST_ASSERT_INT16_WITHIN(5, 0, decodeInt16(&payload[2]));    // no lateral g
  TEST_ASSERT_INT16_WITHIN(5, 0, decodeInt16(&payload[4]));    // gravity removed
  TEST_ASSERT_INT16_WITHIN(5, 300, decodeInt16(&payload[10])); // 30 deg/s yaw
}

void test_motion_is_capped_at_the_configured_rate(void) {
  RecordingSink sink;
  kart::TelemetryPipeline::Config config;
  config.motionPublishHz = 25;
  kart::TelemetryPipeline pipeline(sink, config);
  imu::MotionProcessor processor;
  // The app asks for everything as fast as it comes; our own cap has to hold.
  allowPid(pipeline, kart::channels::kPidMotion, 0);

  const std::vector<uint8_t> burst = makeBurst(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
  for (uint32_t t = 0; t < 1000; t += 10) { // IMU sampled at 100 Hz
    sink.setClock(t);
    pipeline.onMotionSample(processBurst(processor, burst, t), t);
  }

  // One second at 25 Hz starting from t = 0: 0, 40, ... 960.
  TEST_ASSERT_EQUAL_size_t(25, sink.canFor(kart::channels::kPidMotion).size());
  TEST_ASSERT_EQUAL_UINT32(25, pipeline.stats().motionSent);
  TEST_ASSERT_EQUAL_UINT32(75, pipeline.stats().droppedByRate);
}

void test_racechrono_can_ask_for_a_slower_motion_rate(void) {
  RecordingSink sink;
  kart::TelemetryPipeline pipeline(sink);
  imu::MotionProcessor processor;
  allowPid(pipeline, kart::channels::kPidMotion, 200); // 5 Hz

  const std::vector<uint8_t> burst = makeBurst(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
  for (uint32_t t = 0; t < 1000; t += 10) {
    pipeline.onMotionSample(processBurst(processor, burst, t), t);
  }
  TEST_ASSERT_EQUAL_size_t(5, sink.canFor(kart::channels::kPidMotion).size());
}

// --- Status channel and the SET button --------------------------------------

void test_status_channel_reports_the_session_clock(void) {
  RecordingSink sink;
  kart::TelemetryPipeline pipeline(sink);
  allowPid(pipeline, kart::channels::kPidStatus, 0);

  pipeline.resetSession(10000);
  pipeline.tick(10000);
  pipeline.tick(12500); // 2.5 s later, past the 1 s status interval

  const std::vector<Packet> status = sink.canFor(kart::channels::kPidStatus);
  TEST_ASSERT_EQUAL_size_t(2, status.size());
  TEST_ASSERT_EQUAL_UINT32(0, decodeUint32(status[0].bytes.data() + 4));
  TEST_ASSERT_EQUAL_UINT32(2500, decodeUint32(status[1].bytes.data() + 4));
}

void test_set_button_zeroes_the_session_mid_run(void) {
  RecordingSink sink;
  kart::TelemetryPipeline pipeline(sink);
  allowPid(pipeline, kart::channels::kPidStatus, 0);

  pipeline.resetSession(0);
  pipeline.tick(0);
  pipeline.tick(60000); // a minute into the run

  // The driver rolls onto the grid and presses SET.
  pipeline.resetSession(60000);
  pipeline.tick(61000);

  const std::vector<Packet> status = sink.canFor(kart::channels::kPidStatus);
  TEST_ASSERT_EQUAL_size_t(3, status.size());
  TEST_ASSERT_EQUAL_UINT32(60000, decodeUint32(status[1].bytes.data() + 4));
  TEST_ASSERT_EQUAL_UINT32(1000, decodeUint32(status[2].bytes.data() + 4));
  // The reset counter confirms the press actually landed.
  TEST_ASSERT_EQUAL_UINT8(2, status[2].bytes[4 + 9]);
}

void test_status_flags_track_calibration_and_fix_state(void) {
  RecordingSink sink;
  kart::TelemetryPipeline pipeline(sink);
  nmea::SentenceAssembler assembler;
  nmea::FixAssembler fixes;
  allowPid(pipeline, kart::channels::kPidStatus, 0);

  // Nothing has happened yet: no calibration, no fix.
  pipeline.tick(0);
  uint8_t flags = sink.canFor(kart::channels::kPidStatus)[0].bytes[4 + 6];
  TEST_ASSERT_EQUAL_HEX8(kart::channels::kFlagGnssStale, flags);

  // A fix arrives and the driver zeroes the IMU.
  feedNmea(assembler, fixes, pipeline, kEpoch1, 1000);
  pipeline.setImuCalibrated(true);
  pipeline.tick(1500);
  flags = sink.canFor(kart::channels::kPidStatus)[1].bytes[4 + 6];
  TEST_ASSERT_EQUAL_HEX8(kart::channels::kFlagImuCalibrated | kart::channels::kFlagGnssFixValid,
                         flags);

  // The antenna is knocked loose: no fix for four seconds.
  pipeline.tick(6000);
  flags = sink.canFor(kart::channels::kPidStatus)[2].bytes[4 + 6];
  TEST_ASSERT_TRUE((flags & kart::channels::kFlagGnssStale) != 0);
  TEST_ASSERT_TRUE((flags & kart::channels::kFlagGnssFixValid) == 0);
  TEST_ASSERT_TRUE((flags & kart::channels::kFlagImuCalibrated) != 0);
}

void test_a_rejected_calibration_is_reported_to_the_app(void) {
  RecordingSink sink;
  kart::TelemetryPipeline pipeline(sink);
  allowPid(pipeline, kart::channels::kPidStatus, 0);

  pipeline.setCalibrationRejected(true);
  pipeline.tick(0);
  const uint8_t flags = sink.canFor(kart::channels::kPidStatus)[0].bytes[4 + 6];
  TEST_ASSERT_TRUE((flags & kart::channels::kFlagCalibrationRejected) != 0);
}

// --- Connection lifecycle ---------------------------------------------------

void test_reconnecting_starts_from_deny_all(void) {
  RecordingSink sink;
  kart::TelemetryPipeline pipeline(sink);
  imu::MotionProcessor processor;
  allowPid(pipeline, kart::channels::kPidMotion, 0);

  const std::vector<uint8_t> burst = makeBurst(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f);
  pipeline.onMotionSample(processBurst(processor, burst, 0), 0);
  TEST_ASSERT_EQUAL_size_t(1, sink.canFor(kart::channels::kPidMotion).size());

  // The phone goes away and comes back; its old subscription must not persist.
  pipeline.onDisconnect();
  pipeline.onConnect();
  sink.clear();
  pipeline.onMotionSample(processBurst(processor, burst, 1000), 1000);
  TEST_ASSERT_EQUAL_size_t(0, sink.canFor(kart::channels::kPidMotion).size());
}

// --- Bandwidth --------------------------------------------------------------

void test_one_second_of_telemetry_fits_the_ble_budget(void) {
  // A 5 Hz GPS stream plus a 25 Hz motion stream plus a 1 Hz status packet.
  // At a 15 ms connection interval, BLE 4.2 carries at least 4 notifications
  // per interval, so roughly 260 per second; this checks the design stays far
  // inside that, with room for retries.
  RecordingSink sink;
  kart::TelemetryPipeline pipeline(sink);
  nmea::SentenceAssembler assembler;
  nmea::FixAssembler fixes;
  imu::MotionProcessor processor;
  allowPid(pipeline, kart::channels::kPidMotion, 0);
  allowPid(pipeline, kart::channels::kPidStatus, 0);

  const std::vector<uint8_t> burst = makeBurst(0.2f, 0.4f, 1.0f, 1.0f, 2.0f, 15.0f);
  for (uint32_t t = 0; t < 1000; t += 10) {
    sink.setClock(t);
    pipeline.onMotionSample(processBurst(processor, burst, t), t);
    pipeline.tick(t);
    if (t % 200 == 0) {
      feedNmea(assembler, fixes, pipeline, (t == 0) ? kEpoch1 : kEpoch2, t);
    }
  }

  TEST_ASSERT_EQUAL_size_t(25, sink.canFor(kart::channels::kPidMotion).size());
  TEST_ASSERT_EQUAL_size_t(1, sink.canFor(kart::channels::kPidStatus).size());
  TEST_ASSERT_TRUE(sink.totalNotifications() < 60);
  // Every packet has to fit the 23 byte default ATT payload, otherwise the
  // link would have to negotiate a larger MTU before any of this works.
  for (const Packet& packet : sink.can()) {
    TEST_ASSERT_TRUE(packet.bytes.size() <= 20);
  }
  TEST_ASSERT_TRUE(sink.totalBytes() < 900);
}

int main(int, char**) {
  UNITY_BEGIN();

  RUN_TEST(test_recorded_nmea_becomes_a_correct_gps_notification);
  RUN_TEST(test_gps_time_characteristic_only_fires_when_the_hour_changes);
  RUN_TEST(test_fixes_are_dropped_while_no_phone_is_connected);
  RUN_TEST(test_a_failing_transport_is_counted_not_ignored);

  RUN_TEST(test_motion_is_silent_until_racechrono_subscribes);
  RUN_TEST(test_motion_flows_once_the_pid_is_allowed);
  RUN_TEST(test_motion_is_capped_at_the_configured_rate);
  RUN_TEST(test_racechrono_can_ask_for_a_slower_motion_rate);

  RUN_TEST(test_status_channel_reports_the_session_clock);
  RUN_TEST(test_set_button_zeroes_the_session_mid_run);
  RUN_TEST(test_status_flags_track_calibration_and_fix_state);
  RUN_TEST(test_a_rejected_calibration_is_reported_to_the_app);

  RUN_TEST(test_reconnecting_starts_from_deny_all);
  RUN_TEST(test_one_second_of_telemetry_fits_the_ble_budget);

  return UNITY_END();
}
