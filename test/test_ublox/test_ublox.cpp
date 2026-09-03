// SPDX-License-Identifier: MIT
//
// Unit tests for the UBX frame builder.
//
// The expected byte sequences are the hardcoded commands published by the
// bonogps project (reference/bonogps/src/bonogps.cpp, git submodule). Matching
// an independent implementation byte for byte is a much stronger check than
// comparing this builder against itself.

#include <unity.h>

#include "ublox.h"

void setUp(void) {}
void tearDown(void) {}

// --- Checksum and framing ---------------------------------------------------

void test_checksum_is_fletcher_8(void) {
  // CFG-RATE at 5 Hz: class, id, length, payload.
  const uint8_t data[] = {0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00};
  const ublox::Checksum checksum = ublox::computeChecksum(data, sizeof(data));
  TEST_ASSERT_EQUAL_HEX8(0xDE, checksum.a);
  TEST_ASSERT_EQUAL_HEX8(0x6A, checksum.b);
}

void test_build_frame_lays_out_the_header_correctly(void) {
  const uint8_t payload[] = {0xAA, 0xBB};
  uint8_t frame[16];
  const size_t written = ublox::buildFrame(0x06, 0x08, payload, sizeof(payload), frame,
                                           sizeof(frame));
  TEST_ASSERT_EQUAL_size_t(ublox::kFrameOverhead + 2, written);
  TEST_ASSERT_EQUAL_HEX8(0xB5, frame[0]);
  TEST_ASSERT_EQUAL_HEX8(0x62, frame[1]);
  TEST_ASSERT_EQUAL_HEX8(0x06, frame[2]);
  TEST_ASSERT_EQUAL_HEX8(0x08, frame[3]);
  // Length is little-endian.
  TEST_ASSERT_EQUAL_HEX8(0x02, frame[4]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[5]);
  TEST_ASSERT_TRUE(ublox::validateFrame(frame, written));
}

void test_build_frame_refuses_a_buffer_that_is_too_small(void) {
  const uint8_t payload[] = {0xAA, 0xBB};
  uint8_t frame[9]; // needs 10
  TEST_ASSERT_EQUAL_size_t(0, ublox::buildFrame(0x06, 0x08, payload, sizeof(payload), frame,
                                                sizeof(frame)));
}

void test_validate_frame_rejects_corruption(void) {
  uint8_t frame[16];
  const size_t written = ublox::buildCfgRateHz(5, frame, sizeof(frame));
  TEST_ASSERT_TRUE(ublox::validateFrame(frame, written));

  frame[7] ^= 0x01; // flip a payload bit
  TEST_ASSERT_FALSE(ublox::validateFrame(frame, written));

  const size_t rewritten = ublox::buildCfgRateHz(5, frame, sizeof(frame));
  frame[0] = 0x00; // break the sync word
  TEST_ASSERT_FALSE(ublox::validateFrame(frame, rewritten));
}

// --- CFG-RATE ---------------------------------------------------------------

void test_cfg_rate_matches_the_published_5hz_command(void) {
  const uint8_t expected[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8,
                              0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};
  uint8_t frame[sizeof(expected)];
  const size_t written = ublox::buildCfgRateHz(5, frame, sizeof(frame));
  TEST_ASSERT_EQUAL_size_t(sizeof(expected), written);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, frame, written);
}

void test_cfg_rate_matches_the_published_1hz_command(void) {
  const uint8_t expected[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8,
                              0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39};
  uint8_t frame[sizeof(expected)];
  const size_t written = ublox::buildCfgRateHz(1, frame, sizeof(frame));
  TEST_ASSERT_EQUAL_size_t(sizeof(expected), written);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, frame, written);
}

void test_cfg_rate_matches_the_published_10hz_command(void) {
  const uint8_t expected[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64,
                              0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
  uint8_t frame[sizeof(expected)];
  const size_t written = ublox::buildCfgRateHz(10, frame, sizeof(frame));
  TEST_ASSERT_EQUAL_size_t(sizeof(expected), written);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, frame, written);
}

void test_cfg_rate_rejects_zero_hertz(void) {
  uint8_t frame[16];
  TEST_ASSERT_EQUAL_size_t(0, ublox::buildCfgRateHz(0, frame, sizeof(frame)));
}

// --- CFG-PRT ----------------------------------------------------------------

void test_cfg_prt_matches_the_published_115200_command(void) {
  const uint8_t expected[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
                              0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x03, 0x00,
                              0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBC, 0x5E};
  uint8_t frame[sizeof(expected)];
  const size_t written = ublox::buildCfgPrtUart1(115200, frame, sizeof(frame));
  TEST_ASSERT_EQUAL_size_t(sizeof(expected), written);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, frame, written);
}

void test_cfg_prt_matches_the_published_38400_command(void) {
  const uint8_t expected[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
                              0xD0, 0x08, 0x00, 0x00, 0x00, 0x96, 0x00, 0x00, 0x03, 0x00,
                              0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8F, 0x70};
  uint8_t frame[sizeof(expected)];
  const size_t written = ublox::buildCfgPrtUart1(38400, frame, sizeof(frame));
  TEST_ASSERT_EQUAL_size_t(sizeof(expected), written);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, frame, written);
}

// --- CFG-MSG ----------------------------------------------------------------

void test_cfg_msg_matches_the_published_gll_off_command(void) {
  const uint8_t expected[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A};
  const uint8_t rates[ublox::kPortCount] = {0, 0, 0, 0, 0, 0};
  uint8_t frame[sizeof(expected)];
  const size_t written =
      ublox::buildCfgMsg(ublox::kClassNmea, ublox::kIdNmeaGll, rates, frame, sizeof(frame));
  TEST_ASSERT_EQUAL_size_t(sizeof(expected), written);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, frame, written);
}

void test_cfg_msg_matches_the_published_gsa_on_command(void) {
  const uint8_t expected[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02,
                              0x01, 0x01, 0x00, 0x01, 0x01, 0x00, 0x05, 0x41};
  const uint8_t rates[ublox::kPortCount] = {1, 1, 0, 1, 1, 0};
  uint8_t frame[sizeof(expected)];
  const size_t written =
      ublox::buildCfgMsg(ublox::kClassNmea, ublox::kIdNmeaGsa, rates, frame, sizeof(frame));
  TEST_ASSERT_EQUAL_size_t(sizeof(expected), written);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, frame, written);
}

void test_cfg_msg_for_port_silences_the_other_ports(void) {
  uint8_t frame[16];
  const size_t written = ublox::buildCfgMsgForPort(ublox::kClassNmea, ublox::kIdNmeaRmc,
                                                   ublox::kPortUart1, 1, frame, sizeof(frame));
  TEST_ASSERT_EQUAL_size_t(16, written);
  TEST_ASSERT_EQUAL_HEX8(0xF0, frame[6]);
  TEST_ASSERT_EQUAL_HEX8(0x04, frame[7]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[8]);  // port 0
  TEST_ASSERT_EQUAL_HEX8(0x01, frame[9]);  // port 1, UART1
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[10]); // port 2
  TEST_ASSERT_TRUE(ublox::validateFrame(frame, written));
}

void test_cfg_msg_rejects_an_out_of_range_port(void) {
  uint8_t frame[16];
  TEST_ASSERT_EQUAL_size_t(
      0, ublox::buildCfgMsgForPort(ublox::kClassNmea, ublox::kIdNmeaRmc, 9, 1, frame,
                                   sizeof(frame)));
}

// --- CFG-NAV5 and CFG-RST ---------------------------------------------------

void test_cfg_nav5_selects_the_automotive_dynamic_model(void) {
  uint8_t frame[64];
  const size_t written =
      ublox::buildCfgNav5DynamicModel(ublox::DynamicModel::kAutomotive, frame, sizeof(frame));
  TEST_ASSERT_EQUAL_size_t(ublox::kFrameOverhead + 36, written);
  TEST_ASSERT_EQUAL_HEX8(0x06, frame[2]);
  TEST_ASSERT_EQUAL_HEX8(0x24, frame[3]);
  // Only bit 0 of the mask is set, so nothing but the dynamic model changes.
  TEST_ASSERT_EQUAL_HEX8(0x01, frame[6]);
  TEST_ASSERT_EQUAL_HEX8(0x00, frame[7]);
  TEST_ASSERT_EQUAL_HEX8(0x04, frame[8]);
  TEST_ASSERT_TRUE(ublox::validateFrame(frame, written));
}

void test_cfg_rst_matches_the_published_warm_start_command(void) {
  const uint8_t expected[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00,
                              0x00, 0x00, 0x02, 0x00, 0x10, 0x68};
  uint8_t frame[sizeof(expected)];
  const size_t written = ublox::buildCfgRstHotStart(frame, sizeof(frame));
  TEST_ASSERT_EQUAL_size_t(sizeof(expected), written);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, frame, written);
}

int main(int, char**) {
  UNITY_BEGIN();

  RUN_TEST(test_checksum_is_fletcher_8);
  RUN_TEST(test_build_frame_lays_out_the_header_correctly);
  RUN_TEST(test_build_frame_refuses_a_buffer_that_is_too_small);
  RUN_TEST(test_validate_frame_rejects_corruption);

  RUN_TEST(test_cfg_rate_matches_the_published_5hz_command);
  RUN_TEST(test_cfg_rate_matches_the_published_1hz_command);
  RUN_TEST(test_cfg_rate_matches_the_published_10hz_command);
  RUN_TEST(test_cfg_rate_rejects_zero_hertz);

  RUN_TEST(test_cfg_prt_matches_the_published_115200_command);
  RUN_TEST(test_cfg_prt_matches_the_published_38400_command);

  RUN_TEST(test_cfg_msg_matches_the_published_gll_off_command);
  RUN_TEST(test_cfg_msg_matches_the_published_gsa_on_command);
  RUN_TEST(test_cfg_msg_for_port_silences_the_other_ports);
  RUN_TEST(test_cfg_msg_rejects_an_out_of_range_port);

  RUN_TEST(test_cfg_nav5_selects_the_automotive_dynamic_model);
  RUN_TEST(test_cfg_rst_matches_the_published_warm_start_command);

  return UNITY_END();
}
