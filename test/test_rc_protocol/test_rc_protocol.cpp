// SPDX-License-Identifier: MIT
//
// Unit tests for the RaceChrono DIY BLE wire format.
//
// The golden packet in test_gps_main_packet_golden_vector was worked out by
// hand from the specification in reference/racechrono-ble-diy-device/README.md
// rather than by running this encoder, so it catches a layout or endianness
// mistake instead of blessing one.

#include <unity.h>

#include <string.h>

#include "rc_protocol.h"

namespace {

telemetry::GnssFix makeReferenceFix() {
  telemetry::GnssFix fix;
  fix.timeValid = true;
  fix.hour = 14;
  fix.minute = 35;
  fix.second = 12;
  fix.millisecond = 500;

  fix.dateValid = true;
  fix.year = 2026;
  fix.month = 3;
  fix.day = 15;

  fix.navigationValid = true;
  fix.positionValid = true;
  fix.latitudeE7 = -235000000;  // -23.5 degrees
  fix.longitudeE7 = -466000000; // -46.6 degrees

  fix.altitudeValid = true;
  fix.altitudeMeters = 760.5f;
  fix.speedValid = true;
  fix.speedKph = 92.34f;
  fix.bearingValid = true;
  fix.bearingDeg = 271.25f;

  fix.fixQuality = 1;
  fix.satellitesValid = true;
  fix.satellites = 9;
  fix.hdopValid = true;
  fix.hdop = 0.9f;
  fix.vdopValid = true;
  fix.vdop = 1.4f;
  return fix;
}

} // namespace

void setUp(void) {}
void tearDown(void) {}

// --- Time and date ----------------------------------------------------------

void test_time_since_hour_start(void) {
  // 35 * 30000 + 12 * 500 + 500 / 2
  TEST_ASSERT_EQUAL_UINT32(1056250u, rc::encodeTimeSinceHourStart(35, 12, 500));
  TEST_ASSERT_EQUAL_UINT32(0u, rc::encodeTimeSinceHourStart(0, 0, 0));
  // The field is 21 bits; the largest legal input must still fit.
  const uint32_t maximum = rc::encodeTimeSinceHourStart(59, 59, 999);
  TEST_ASSERT_EQUAL_UINT32(1799999u, maximum);
  TEST_ASSERT_TRUE(maximum < (1u << 21));
}

void test_time_since_hour_start_clamps_out_of_range_input(void) {
  TEST_ASSERT_EQUAL_UINT32(rc::encodeTimeSinceHourStart(59, 59, 999),
                           rc::encodeTimeSinceHourStart(200, 200, 5000));
}

void test_date_and_hour(void) {
  // (2026-2000)*8928 + (3-1)*744 + (15-1)*24 + 14
  TEST_ASSERT_EQUAL_INT32(233966, rc::encodeDateAndHour(2026, 3, 15, 14));
  TEST_ASSERT_EQUAL_INT32(0, rc::encodeDateAndHour(2000, 1, 1, 0));
}

void test_date_and_hour_rejects_invalid_dates(void) {
  TEST_ASSERT_EQUAL_INT32(-1, rc::encodeDateAndHour(1999, 1, 1, 0));
  TEST_ASSERT_EQUAL_INT32(-1, rc::encodeDateAndHour(2026, 0, 1, 0));
  TEST_ASSERT_EQUAL_INT32(-1, rc::encodeDateAndHour(2026, 13, 1, 0));
  TEST_ASSERT_EQUAL_INT32(-1, rc::encodeDateAndHour(2026, 1, 0, 0));
  TEST_ASSERT_EQUAL_INT32(-1, rc::encodeDateAndHour(2026, 1, 32, 0));
  TEST_ASSERT_EQUAL_INT32(-1, rc::encodeDateAndHour(2026, 1, 1, 24));
}

// --- Altitude ---------------------------------------------------------------

void test_altitude_fine_encoding(void) {
  TEST_ASSERT_EQUAL_HEX16(5000, rc::encodeAltitude(0.0f, true));      // (0+500)*10
  TEST_ASSERT_EQUAL_HEX16(0, rc::encodeAltitude(-500.0f, true));      // floor of the range
  TEST_ASSERT_EQUAL_HEX16(12605, rc::encodeAltitude(760.5f, true));   // (760.5+500)*10
  TEST_ASSERT_EQUAL_HEX16(0xFFFF, rc::encodeAltitude(0.0f, false));
}

void test_altitude_switches_to_coarse_above_the_fine_ceiling(void) {
  // The fine encoding is masked to 15 bits, so it tops out at
  // (0x7FFF / 10) - 500 = 2776.7 m. One metre above that must flip to coarse.
  const uint16_t justBelow = rc::encodeAltitude(2776.0f, true);
  TEST_ASSERT_EQUAL_HEX16(0, justBelow & 0x8000);
  TEST_ASSERT_EQUAL_HEX16(32760, justBelow);

  const uint16_t justAbove = rc::encodeAltitude(2800.0f, true);
  TEST_ASSERT_EQUAL_HEX16(0x8000, justAbove & 0x8000);
  TEST_ASSERT_EQUAL_HEX16(3300, justAbove & 0x7FFF); // 2800 + 500
}

void test_altitude_below_the_floor_is_clamped_not_wrapped(void) {
  // A negative intermediate would wrap to a huge positive altitude, so the
  // encoder has to clamp instead.
  TEST_ASSERT_EQUAL_HEX16(0, rc::encodeAltitude(-1000.0f, true));
}

// --- Speed ------------------------------------------------------------------

void test_speed_fine_encoding(void) {
  TEST_ASSERT_EQUAL_HEX16(0, rc::encodeSpeed(0.0f, true));
  TEST_ASSERT_EQUAL_HEX16(9234, rc::encodeSpeed(92.34f, true));
  TEST_ASSERT_EQUAL_HEX16(0xFFFF, rc::encodeSpeed(0.0f, false));
}

void test_speed_switches_to_coarse_above_the_fine_ceiling(void) {
  const uint16_t justBelow = rc::encodeSpeed(327.0f, true);
  TEST_ASSERT_EQUAL_HEX16(0, justBelow & 0x8000);
  TEST_ASSERT_EQUAL_HEX16(32700, justBelow);

  const uint16_t justAbove = rc::encodeSpeed(400.0f, true);
  TEST_ASSERT_EQUAL_HEX16(0x8000, justAbove & 0x8000);
  TEST_ASSERT_EQUAL_HEX16(4000, justAbove & 0x7FFF);
}

void test_negative_speed_is_clamped_to_zero(void) {
  TEST_ASSERT_EQUAL_HEX16(0, rc::encodeSpeed(-5.0f, true));
}

// --- Bearing and DOP --------------------------------------------------------

void test_bearing_encoding_and_wrapping(void) {
  TEST_ASSERT_EQUAL_HEX16(27125, rc::encodeBearing(271.25f, true));
  TEST_ASSERT_EQUAL_HEX16(0, rc::encodeBearing(360.0f, true));
  TEST_ASSERT_EQUAL_HEX16(35900, rc::encodeBearing(-1.0f, true));
  TEST_ASSERT_EQUAL_HEX16(1000, rc::encodeBearing(370.0f, true));
  TEST_ASSERT_EQUAL_HEX16(0xFFFF, rc::encodeBearing(0.0f, false));
}

void test_dop_encoding(void) {
  TEST_ASSERT_EQUAL_HEX8(9, rc::encodeDop(0.9f, true));
  TEST_ASSERT_EQUAL_HEX8(14, rc::encodeDop(1.4f, true));
  TEST_ASSERT_EQUAL_HEX8(0xFF, rc::encodeDop(1.0f, false));
  // 0xFF is the reserved "unknown" value, so a huge DOP has to stop at 0xFE.
  TEST_ASSERT_EQUAL_HEX8(0xFE, rc::encodeDop(99.0f, true));
}

// --- Full packets -----------------------------------------------------------

void test_gps_main_packet_golden_vector(void) {
  rc::GpsEncoder encoder;
  const rc::GpsPackets packets = encoder.encode(makeReferenceFix());

  const uint8_t expectedMain[rc::kGpsMainPacketSize] = {
      0x30,                   // sync 1, time bits 20..16
      0x1D, 0xFA,             // time 1056250
      0x49,                   // fix quality 1, 9 satellites
      0xF1, 0xFE, 0x2F, 0x40, // latitude -235000000
      0xE4, 0x39, 0x67, 0x80, // longitude -466000000
      0x31, 0x3D,             // altitude 12605
      0x24, 0x12,             // speed 9234
      0x69, 0xF5,             // bearing 27125
      0x09,                   // hdop 0.9
      0x0E,                   // vdop 1.4
  };
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expectedMain, packets.main, rc::kGpsMainPacketSize);

  const uint8_t expectedTime[rc::kGpsTimePacketSize] = {0x23, 0x91, 0xEE};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expectedTime, packets.time, rc::kGpsTimePacketSize);
  TEST_ASSERT_TRUE(packets.timeChanged);
}

void test_invalid_fix_uses_the_specified_sentinels(void) {
  telemetry::GnssFix fix;
  fix.timeValid = false;
  rc::GpsEncoder encoder;
  const rc::GpsPackets packets = encoder.encode(fix);

  // Latitude and longitude both 0x7FFFFFFF.
  const uint8_t expectedCoordinate[4] = {0x7F, 0xFF, 0xFF, 0xFF};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expectedCoordinate, &packets.main[4], 4);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expectedCoordinate, &packets.main[8], 4);
  // Altitude, speed and bearing all 0xFFFF, both DOPs 0xFF.
  TEST_ASSERT_EQUAL_HEX8(0xFF, packets.main[12]);
  TEST_ASSERT_EQUAL_HEX8(0xFF, packets.main[13]);
  TEST_ASSERT_EQUAL_HEX8(0xFF, packets.main[14]);
  TEST_ASSERT_EQUAL_HEX8(0xFF, packets.main[15]);
  TEST_ASSERT_EQUAL_HEX8(0xFF, packets.main[16]);
  TEST_ASSERT_EQUAL_HEX8(0xFF, packets.main[17]);
  TEST_ASSERT_EQUAL_HEX8(0xFF, packets.main[18]);
  TEST_ASSERT_EQUAL_HEX8(0xFF, packets.main[19]);
  // Unknown satellite count is 0x3F with fix quality 0.
  TEST_ASSERT_EQUAL_HEX8(0x3F, packets.main[3]);
  TEST_ASSERT_FALSE(packets.timeChanged);
}

void test_satellite_count_never_collides_with_the_unknown_marker(void) {
  telemetry::GnssFix fix = makeReferenceFix();
  fix.satellites = 63; // 0x3F, which means "unknown" on the wire
  rc::GpsEncoder encoder;
  const rc::GpsPackets packets = encoder.encode(fix);
  TEST_ASSERT_EQUAL_HEX8(0x3E, packets.main[3] & 0x3F);
}

// --- Sync bits --------------------------------------------------------------

void test_sync_bits_advance_only_when_the_hour_changes(void) {
  rc::GpsEncoder encoder;
  telemetry::GnssFix fix = makeReferenceFix();

  const rc::GpsPackets first = encoder.encode(fix);
  TEST_ASSERT_TRUE(first.timeChanged);
  TEST_ASSERT_EQUAL_UINT8(1, encoder.syncBits());

  // Same hour, later in the minute: the counter must hold still.
  fix.second = 40;
  const rc::GpsPackets second = encoder.encode(fix);
  TEST_ASSERT_FALSE(second.timeChanged);
  TEST_ASSERT_EQUAL_UINT8(1, encoder.syncBits());
  TEST_ASSERT_EQUAL_HEX8(first.main[0] & 0xE0, second.main[0] & 0xE0);

  // New hour: the counter advances and both characteristics carry it.
  fix.hour = 15;
  const rc::GpsPackets third = encoder.encode(fix);
  TEST_ASSERT_TRUE(third.timeChanged);
  TEST_ASSERT_EQUAL_UINT8(2, encoder.syncBits());
  TEST_ASSERT_EQUAL_HEX8(third.main[0] & 0xE0, third.time[0] & 0xE0);
}

void test_sync_bits_wrap_at_three_bits(void) {
  rc::GpsEncoder encoder;
  telemetry::GnssFix fix = makeReferenceFix();
  for (int i = 0; i < 8; ++i) {
    fix.hour = static_cast<uint8_t>(i);
    encoder.encode(fix);
  }
  TEST_ASSERT_EQUAL_UINT8(0, encoder.syncBits());
  fix.hour = 9;
  encoder.encode(fix);
  TEST_ASSERT_EQUAL_UINT8(1, encoder.syncBits());
}

// --- CAN packets ------------------------------------------------------------

void test_can_packet_id_is_little_endian(void) {
  const uint8_t payload[3] = {0xAA, 0xBB, 0xCC};
  uint8_t out[rc::kCanMaxPacketSize];
  const size_t written = rc::encodeCanPacket(0x12345678u, payload, sizeof(payload), out);

  TEST_ASSERT_EQUAL_size_t(7, written);
  const uint8_t expected[7] = {0x78, 0x56, 0x34, 0x12, 0xAA, 0xBB, 0xCC};
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, out, written);
}

void test_can_packet_rejects_illegal_payload_lengths(void) {
  uint8_t payload[20] = {0};
  uint8_t out[rc::kCanMaxPacketSize];
  TEST_ASSERT_EQUAL_size_t(0, rc::encodeCanPacket(1, payload, 0, out));
  TEST_ASSERT_EQUAL_size_t(0, rc::encodeCanPacket(1, payload, 17, out));
  TEST_ASSERT_EQUAL_size_t(20, rc::encodeCanPacket(1, payload, 16, out));
}

// --- CAN filter -------------------------------------------------------------

void test_filter_denies_everything_before_the_app_says_otherwise(void) {
  rc::CanFilter filter;
  TEST_ASSERT_FALSE(filter.isAllowed(0x100));
  TEST_ASSERT_FALSE(filter.shouldNotify(0x100, 0));
}

void test_filter_allow_one_pid_reads_a_big_endian_pid(void) {
  rc::CanFilter filter;
  // Command 2, interval 0x0064 (100 ms), PID 0x00000100, all big-endian.
  const uint8_t write[7] = {2, 0x00, 0x64, 0x00, 0x00, 0x01, 0x00};
  TEST_ASSERT_TRUE(filter.handleWrite(write, sizeof(write)));

  TEST_ASSERT_TRUE(filter.isAllowed(0x100));
  TEST_ASSERT_FALSE(filter.isAllowed(0x101));
  TEST_ASSERT_EQUAL_UINT16(100, filter.notifyIntervalMs(0x100));
  TEST_ASSERT_EQUAL_size_t(1, filter.allowedPidCount());
}

void test_filter_rate_limits_per_pid(void) {
  rc::CanFilter filter;
  const uint8_t allow[7] = {2, 0x00, 0x64, 0x00, 0x00, 0x01, 0x00}; // 100 ms
  TEST_ASSERT_TRUE(filter.handleWrite(allow, sizeof(allow)));

  TEST_ASSERT_TRUE(filter.shouldNotify(0x100, 1000));  // first one always passes
  TEST_ASSERT_FALSE(filter.shouldNotify(0x100, 1050)); // too soon
  TEST_ASSERT_FALSE(filter.shouldNotify(0x100, 1099));
  TEST_ASSERT_TRUE(filter.shouldNotify(0x100, 1100)); // exactly one interval
  TEST_ASSERT_TRUE(filter.shouldNotify(0x100, 1300));
}

void test_filter_rate_limit_survives_the_millisecond_rollover(void) {
  rc::CanFilter filter;
  const uint8_t allow[7] = {2, 0x00, 0x64, 0x00, 0x00, 0x01, 0x00};
  filter.handleWrite(allow, sizeof(allow));

  const uint32_t beforeWrap = 0xFFFFFFF0u;
  TEST_ASSERT_TRUE(filter.shouldNotify(0x100, beforeWrap));
  // 0x40 milliseconds later, having wrapped through zero: still inside the
  // 100 ms interval, so it must not fire.
  TEST_ASSERT_FALSE(filter.shouldNotify(0x100, 0x30u));
  // 0x74 past the wrap is 116 ms after the last notification.
  TEST_ASSERT_TRUE(filter.shouldNotify(0x100, 0x64u));
}

void test_filter_allow_all_shares_one_interval(void) {
  rc::CanFilter filter;
  const uint8_t allowAll[3] = {1, 0x00, 0x32}; // 50 ms
  TEST_ASSERT_TRUE(filter.handleWrite(allowAll, sizeof(allowAll)));

  TEST_ASSERT_TRUE(filter.allowAll());
  TEST_ASSERT_TRUE(filter.isAllowed(0xDEAD));
  TEST_ASSERT_TRUE(filter.shouldNotify(0xDEAD, 0));
  TEST_ASSERT_FALSE(filter.shouldNotify(0xBEEF, 20));
  TEST_ASSERT_TRUE(filter.shouldNotify(0xBEEF, 50));
}

void test_filter_deny_all_clears_previous_permissions(void) {
  rc::CanFilter filter;
  const uint8_t allow[7] = {2, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00};
  filter.handleWrite(allow, sizeof(allow));
  TEST_ASSERT_TRUE(filter.isAllowed(0x100));

  const uint8_t denyAll[1] = {0};
  TEST_ASSERT_TRUE(filter.handleWrite(denyAll, sizeof(denyAll)));
  TEST_ASSERT_FALSE(filter.isAllowed(0x100));
  TEST_ASSERT_EQUAL_size_t(0, filter.allowedPidCount());
}

void test_filter_updating_a_known_pid_does_not_consume_a_slot(void) {
  rc::CanFilter filter;
  const uint8_t first[7] = {2, 0x00, 0x64, 0x00, 0x00, 0x01, 0x00};
  const uint8_t second[7] = {2, 0x00, 0x0A, 0x00, 0x00, 0x01, 0x00};
  filter.handleWrite(first, sizeof(first));
  filter.handleWrite(second, sizeof(second));

  TEST_ASSERT_EQUAL_size_t(1, filter.allowedPidCount());
  TEST_ASSERT_EQUAL_UINT16(10, filter.notifyIntervalMs(0x100));
}

void test_filter_rejects_malformed_writes(void) {
  rc::CanFilter filter;
  const uint8_t truncatedAllowAll[2] = {1, 0x00};
  const uint8_t truncatedAllowPid[6] = {2, 0x00, 0x64, 0x00, 0x00, 0x01};
  const uint8_t unknownCommand[1] = {9};

  TEST_ASSERT_FALSE(filter.handleWrite(nullptr, 0));
  TEST_ASSERT_FALSE(filter.handleWrite(truncatedAllowAll, sizeof(truncatedAllowAll)));
  TEST_ASSERT_FALSE(filter.handleWrite(truncatedAllowPid, sizeof(truncatedAllowPid)));
  TEST_ASSERT_FALSE(filter.handleWrite(unknownCommand, sizeof(unknownCommand)));
  TEST_ASSERT_FALSE(filter.isAllowed(0x100));
}

void test_filter_reports_a_full_pid_table(void) {
  rc::CanFilter filter;
  uint8_t write[7] = {2, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  for (size_t i = 0; i < rc::CanFilter::kMaxAllowedPids; ++i) {
    write[6] = static_cast<uint8_t>(i);
    TEST_ASSERT_TRUE(filter.handleWrite(write, sizeof(write)));
  }
  write[6] = 0xFF;
  TEST_ASSERT_FALSE(filter.handleWrite(write, sizeof(write)));
  TEST_ASSERT_EQUAL_size_t(rc::CanFilter::kMaxAllowedPids, filter.allowedPidCount());
}

int main(int, char**) {
  UNITY_BEGIN();

  RUN_TEST(test_time_since_hour_start);
  RUN_TEST(test_time_since_hour_start_clamps_out_of_range_input);
  RUN_TEST(test_date_and_hour);
  RUN_TEST(test_date_and_hour_rejects_invalid_dates);

  RUN_TEST(test_altitude_fine_encoding);
  RUN_TEST(test_altitude_switches_to_coarse_above_the_fine_ceiling);
  RUN_TEST(test_altitude_below_the_floor_is_clamped_not_wrapped);

  RUN_TEST(test_speed_fine_encoding);
  RUN_TEST(test_speed_switches_to_coarse_above_the_fine_ceiling);
  RUN_TEST(test_negative_speed_is_clamped_to_zero);

  RUN_TEST(test_bearing_encoding_and_wrapping);
  RUN_TEST(test_dop_encoding);

  RUN_TEST(test_gps_main_packet_golden_vector);
  RUN_TEST(test_invalid_fix_uses_the_specified_sentinels);
  RUN_TEST(test_satellite_count_never_collides_with_the_unknown_marker);

  RUN_TEST(test_sync_bits_advance_only_when_the_hour_changes);
  RUN_TEST(test_sync_bits_wrap_at_three_bits);

  RUN_TEST(test_can_packet_id_is_little_endian);
  RUN_TEST(test_can_packet_rejects_illegal_payload_lengths);

  RUN_TEST(test_filter_denies_everything_before_the_app_says_otherwise);
  RUN_TEST(test_filter_allow_one_pid_reads_a_big_endian_pid);
  RUN_TEST(test_filter_rate_limits_per_pid);
  RUN_TEST(test_filter_rate_limit_survives_the_millisecond_rollover);
  RUN_TEST(test_filter_allow_all_shares_one_interval);
  RUN_TEST(test_filter_deny_all_clears_previous_permissions);
  RUN_TEST(test_filter_updating_a_known_pid_does_not_consume_a_slot);
  RUN_TEST(test_filter_rejects_malformed_writes);
  RUN_TEST(test_filter_reports_a_full_pid_table);

  return UNITY_END();
}
