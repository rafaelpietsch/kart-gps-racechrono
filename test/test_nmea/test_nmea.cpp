// SPDX-License-Identifier: MIT
//
// Unit tests for NMEA sentence assembly and fix parsing.
//
// The sentences below are real u-blox output shapes with checksums computed
// independently of this code.

#include <unity.h>

#include <string.h>

#include "nmea.h"

namespace {

// One complete 5 Hz epoch at 14:35:12.50 UTC, Interlagos-ish coordinates.
const char kRmc[] = "$GPRMC,143512.50,A,2330.0000,S,04636.0000,W,49.86,271.25,150326,,,A*62\r\n";
const char kGga[] = "$GPGGA,143512.50,2330.0000,S,04636.0000,W,1,09,0.9,760.5,M,-6.5,M,,*79\r\n";
const char kGsa[] = "$GPGSA,A,3,04,05,,09,12,,,24,,,,,1.7,0.9,1.4*35\r\n";
// The following epoch, half a second later.
const char kRmcNext[] =
    "$GPRMC,143513.00,A,2330.0000,S,04636.0000,W,49.86,271.25,150326,,,A*66\r\n";
const char kGgaNext[] = "$GPGGA,143513.00,2330.0000,S,04636.0000,W,1,09,0.9,760.5,M,-6.5,M,,*7D\r\n";
// No fix yet: RMC status V, GGA quality 0.
const char kRmcNoFix[] = "$GPRMC,143512.50,V,,,,,,,150326,,,N*7B\r\n";
const char kGgaNoFix[] = "$GPGGA,143512.50,,,,,0,00,99.99,,,,,,*63\r\n";
// A sentence type the parser has no use for.
const char kVtg[] = "$GPVTG,271.25,T,,M,49.86,N,92.34,K,A*31\r\n";

/// Feeds a whole C string through the assembler and returns how many complete
/// sentences came out, handing each one to `sink`.
template <typename Sink>
size_t feed(nmea::SentenceAssembler& assembler, const char* text, Sink sink) {
  size_t count = 0;
  for (const char* p = text; *p != '\0'; ++p) {
    if (assembler.push(*p)) {
      ++count;
      sink(assembler.sentence(), assembler.length());
    }
  }
  return count;
}

size_t feedAndDiscard(nmea::SentenceAssembler& assembler, const char* text) {
  return feed(assembler, text, [](const char*, size_t) {});
}

/// Drives the whole chain: raw bytes in, assembled fix out.
bool feedIntoAssembler(nmea::SentenceAssembler& assembler, nmea::FixAssembler& fixes,
                       const char* text) {
  bool completed = false;
  feed(assembler, text, [&](const char* body, size_t length) {
    fixes.consume(body, length);
    if (fixes.hasCompleteFix()) {
      completed = true;
    }
  });
  return completed;
}

} // namespace

void setUp(void) {}
void tearDown(void) {}

// --- Checksum ---------------------------------------------------------------

void test_checksum_matches_the_published_value(void) {
  const char body[] = "GPGSA,A,3,04,05,,09,12,,,24,,,,,1.7,0.9,1.4";
  TEST_ASSERT_EQUAL_HEX8(0x35, nmea::computeChecksum(body, strlen(body)));
}

// --- SentenceAssembler ------------------------------------------------------

void test_assembler_delivers_the_body_without_delimiters(void) {
  nmea::SentenceAssembler assembler;
  const char* delivered = nullptr;
  size_t deliveredLength = 0;
  const size_t count = feed(assembler, kGsa, [&](const char* body, size_t length) {
    delivered = body;
    deliveredLength = length;
  });

  TEST_ASSERT_EQUAL_size_t(1, count);
  TEST_ASSERT_EQUAL_STRING("GPGSA,A,3,04,05,,09,12,,,24,,,,,1.7,0.9,1.4", delivered);
  TEST_ASSERT_EQUAL_size_t(strlen("GPGSA,A,3,04,05,,09,12,,,24,,,,,1.7,0.9,1.4"), deliveredLength);
  TEST_ASSERT_EQUAL_UINT32(1, assembler.stats().accepted);
  TEST_ASSERT_EQUAL_UINT32(0, assembler.stats().checksumErrors);
}

void test_assembler_rejects_a_corrupted_sentence(void) {
  nmea::SentenceAssembler assembler;
  // Same sentence with one digit flipped, so the checksum no longer matches.
  const char corrupted[] = "$GPGSA,A,3,04,05,,09,12,,,24,,,,,1.7,0.9,9.4*35\r\n";
  TEST_ASSERT_EQUAL_size_t(0, feedAndDiscard(assembler, corrupted));
  TEST_ASSERT_EQUAL_UINT32(0, assembler.stats().accepted);
  TEST_ASSERT_EQUAL_UINT32(1, assembler.stats().checksumErrors);
}

void test_assembler_rejects_a_sentence_without_a_checksum(void) {
  nmea::SentenceAssembler assembler;
  const char noChecksum[] = "$GPGSA,A,3,04,05\r\n";
  TEST_ASSERT_EQUAL_size_t(0, feedAndDiscard(assembler, noChecksum));
  TEST_ASSERT_EQUAL_UINT32(1, assembler.stats().checksumErrors);
}

void test_assembler_recovers_when_a_sentence_is_cut_short(void) {
  nmea::SentenceAssembler assembler;
  // A truncated sentence followed by a good one: only the good one survives,
  // and the '$' is what resynchronises the stream.
  const size_t count = feedAndDiscard(assembler, "$GPRMC,1435");
  TEST_ASSERT_EQUAL_size_t(0, count);
  TEST_ASSERT_EQUAL_size_t(1, feedAndDiscard(assembler, kGsa));
  TEST_ASSERT_EQUAL_UINT32(1, assembler.stats().accepted);
}

void test_assembler_survives_leading_noise(void) {
  nmea::SentenceAssembler assembler;
  TEST_ASSERT_EQUAL_size_t(0, feedAndDiscard(assembler, "\x00\xFF garbage 12345"));
  TEST_ASSERT_EQUAL_size_t(1, feedAndDiscard(assembler, kGga));
}

void test_assembler_counts_an_overlong_sentence_as_an_overflow(void) {
  nmea::SentenceAssembler assembler;
  char oversized[nmea::SentenceAssembler::kMaxSentenceLength + 40];
  oversized[0] = '$';
  memset(oversized + 1, 'A', sizeof(oversized) - 2);
  oversized[sizeof(oversized) - 1] = '\0';

  TEST_ASSERT_EQUAL_size_t(0, feedAndDiscard(assembler, oversized));
  TEST_ASSERT_EQUAL_UINT32(1, assembler.stats().overflows);
  // And the next real sentence still gets through.
  TEST_ASSERT_EQUAL_size_t(1, feedAndDiscard(assembler, kGga));
}

// --- Field parsers ----------------------------------------------------------

void test_parse_coordinate_keeps_full_resolution(void) {
  int32_t value = 0;
  const char field[] = "2330.0000";
  TEST_ASSERT_TRUE(nmea::parseCoordinate(field, strlen(field), 'S', value));
  TEST_ASSERT_EQUAL_INT32(-235000000, value);

  const char east[] = "04636.0000";
  TEST_ASSERT_TRUE(nmea::parseCoordinate(east, strlen(east), 'W', value));
  TEST_ASSERT_EQUAL_INT32(-466000000, value);
}

void test_parse_coordinate_rounds_the_last_digit(void) {
  int32_t value = 0;
  // 23 deg 30.00001 min: the seventh decimal of a degree is about 1.1 cm, so
  // truncating here would visibly bend a racing line.
  const char field[] = "2330.00001";
  TEST_ASSERT_TRUE(nmea::parseCoordinate(field, strlen(field), 'N', value));
  TEST_ASSERT_EQUAL_INT32(235000002, value);
}

void test_parse_coordinate_rejects_malformed_input(void) {
  int32_t value = 0;
  const char badMinutes[] = "2360.0000"; // 60 minutes is not a valid value
  const char badHemisphere[] = "2330.0000";
  const char notANumber[] = "23x0.0000";
  TEST_ASSERT_FALSE(nmea::parseCoordinate(badMinutes, strlen(badMinutes), 'N', value));
  TEST_ASSERT_FALSE(nmea::parseCoordinate(badHemisphere, strlen(badHemisphere), 'X', value));
  TEST_ASSERT_FALSE(nmea::parseCoordinate(notANumber, strlen(notANumber), 'N', value));
  TEST_ASSERT_FALSE(nmea::parseCoordinate("", 0, 'N', value));
}

void test_parse_time_of_day(void) {
  uint8_t hour = 0;
  uint8_t minute = 0;
  uint8_t second = 0;
  uint16_t millisecond = 0;

  const char withFraction[] = "143512.50";
  TEST_ASSERT_TRUE(
      nmea::parseTimeOfDay(withFraction, strlen(withFraction), hour, minute, second, millisecond));
  TEST_ASSERT_EQUAL_UINT8(14, hour);
  TEST_ASSERT_EQUAL_UINT8(35, minute);
  TEST_ASSERT_EQUAL_UINT8(12, second);
  TEST_ASSERT_EQUAL_UINT16(500, millisecond);

  const char whole[] = "000000";
  TEST_ASSERT_TRUE(nmea::parseTimeOfDay(whole, strlen(whole), hour, minute, second, millisecond));
  TEST_ASSERT_EQUAL_UINT8(0, hour);
  TEST_ASSERT_EQUAL_UINT16(0, millisecond);
}

void test_parse_time_of_day_clamps_a_leap_second(void) {
  uint8_t hour = 0;
  uint8_t minute = 0;
  uint8_t second = 0;
  uint16_t millisecond = 0;
  const char leap[] = "235960.00";
  TEST_ASSERT_TRUE(nmea::parseTimeOfDay(leap, strlen(leap), hour, minute, second, millisecond));
  TEST_ASSERT_EQUAL_UINT8(59, second);
}

void test_parse_time_of_day_rejects_short_or_invalid_fields(void) {
  uint8_t hour = 0;
  uint8_t minute = 0;
  uint8_t second = 0;
  uint16_t millisecond = 0;
  TEST_ASSERT_FALSE(nmea::parseTimeOfDay("1435", 4, hour, minute, second, millisecond));
  TEST_ASSERT_FALSE(nmea::parseTimeOfDay("246000", 6, hour, minute, second, millisecond));
  TEST_ASSERT_FALSE(nmea::parseTimeOfDay("", 0, hour, minute, second, millisecond));
}

void test_parse_date(void) {
  uint16_t year = 0;
  uint8_t month = 0;
  uint8_t day = 0;
  TEST_ASSERT_TRUE(nmea::parseDate("150326", 6, year, month, day));
  TEST_ASSERT_EQUAL_UINT16(2026, year);
  TEST_ASSERT_EQUAL_UINT8(3, month);
  TEST_ASSERT_EQUAL_UINT8(15, day);

  TEST_ASSERT_FALSE(nmea::parseDate("321326", 6, year, month, day));
  TEST_ASSERT_FALSE(nmea::parseDate("15032", 5, year, month, day));
}

void test_parse_number(void) {
  float value = 0.0f;
  TEST_ASSERT_TRUE(nmea::parseNumber("760.5", 5, value));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 760.5f, value);
  TEST_ASSERT_TRUE(nmea::parseNumber("-6.5", 4, value));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -6.5f, value);
  TEST_ASSERT_TRUE(nmea::parseNumber("0", 1, value));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, value);
  TEST_ASSERT_FALSE(nmea::parseNumber("", 0, value));
  TEST_ASSERT_FALSE(nmea::parseNumber("1.2.3", 5, value));
}

// --- Sentence parsing -------------------------------------------------------

void test_rmc_populates_position_speed_and_date(void) {
  nmea::SentenceAssembler assembler;
  nmea::FixAssembler fixes;
  feedIntoAssembler(assembler, fixes, kRmc);

  const telemetry::GnssFix& fix = fixes.partialFix();
  TEST_ASSERT_TRUE(fix.navigationValid);
  TEST_ASSERT_TRUE(fix.positionValid);
  TEST_ASSERT_EQUAL_INT32(-235000000, fix.latitudeE7);
  TEST_ASSERT_EQUAL_INT32(-466000000, fix.longitudeE7);
  TEST_ASSERT_TRUE(fix.speedValid);
  // 49.86 knots * 1.852
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 92.34f, fix.speedKph);
  TEST_ASSERT_TRUE(fix.bearingValid);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 271.25f, fix.bearingDeg);
  TEST_ASSERT_TRUE(fix.timeValid);
  TEST_ASSERT_EQUAL_UINT8(14, fix.hour);
  TEST_ASSERT_EQUAL_UINT16(500, fix.millisecond);
  TEST_ASSERT_TRUE(fix.dateValid);
  TEST_ASSERT_EQUAL_UINT16(2026, fix.year);
}

void test_gga_populates_altitude_satellites_and_quality(void) {
  nmea::SentenceAssembler assembler;
  nmea::FixAssembler fixes;
  feedIntoAssembler(assembler, fixes, kGga);

  const telemetry::GnssFix& fix = fixes.partialFix();
  TEST_ASSERT_EQUAL_UINT8(1, fix.fixQuality);
  TEST_ASSERT_TRUE(fix.satellitesValid);
  TEST_ASSERT_EQUAL_UINT8(9, fix.satellites);
  TEST_ASSERT_TRUE(fix.hdopValid);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.9f, fix.hdop);
  TEST_ASSERT_TRUE(fix.altitudeValid);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 760.5f, fix.altitudeMeters);
}

void test_gsa_adds_vertical_dilution_of_precision(void) {
  nmea::SentenceAssembler assembler;
  nmea::FixAssembler fixes;
  feedIntoAssembler(assembler, fixes, kGga);
  feedIntoAssembler(assembler, fixes, kGsa);

  const telemetry::GnssFix& fix = fixes.partialFix();
  TEST_ASSERT_TRUE(fix.vdopValid);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.4f, fix.vdop);
  TEST_ASSERT_TRUE(fix.pdopValid);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.7f, fix.pdop);
}

void test_unknown_sentences_are_ignored(void) {
  nmea::FixAssembler fixes;
  const char body[] = "GPVTG,271.25,T,,M,49.86,N,92.34,K,A";
  TEST_ASSERT_TRUE(fixes.consume(body, strlen(body)) == nmea::FixAssembler::Update::kIgnored);

  nmea::SentenceAssembler assembler;
  feedIntoAssembler(assembler, fixes, kVtg);
  TEST_ASSERT_FALSE(fixes.hasCompleteFix());
}

void test_talker_id_does_not_matter(void) {
  nmea::FixAssembler fixes;
  // A multi-constellation receiver uses GN instead of GP.
  const char body[] = "GNGGA,143512.50,2330.0000,S,04636.0000,W,1,09,0.9,760.5,M,-6.5,M,,";
  TEST_ASSERT_TRUE(fixes.consume(body, strlen(body)) == nmea::FixAssembler::Update::kGga);
  TEST_ASSERT_EQUAL_UINT8(9, fixes.partialFix().satellites);
}

// --- Epoch assembly ---------------------------------------------------------

void test_an_epoch_completes_once_rmc_and_gga_have_both_arrived(void) {
  nmea::SentenceAssembler assembler;
  nmea::FixAssembler fixes;

  feedIntoAssembler(assembler, fixes, kGga);
  TEST_ASSERT_FALSE(fixes.hasCompleteFix());
  feedIntoAssembler(assembler, fixes, kGsa);
  TEST_ASSERT_FALSE(fixes.hasCompleteFix());
  feedIntoAssembler(assembler, fixes, kRmc);
  TEST_ASSERT_TRUE(fixes.hasCompleteFix());

  telemetry::GnssFix fix;
  TEST_ASSERT_TRUE(fixes.takeCompletedFix(fix));
  // The merged fix carries fields from all three sentences.
  TEST_ASSERT_EQUAL_INT32(-235000000, fix.latitudeE7);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 760.5f, fix.altitudeMeters);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.4f, fix.vdop);
}

void test_a_completed_fix_is_only_handed_out_once(void) {
  nmea::SentenceAssembler assembler;
  nmea::FixAssembler fixes;
  feedIntoAssembler(assembler, fixes, kGga);
  feedIntoAssembler(assembler, fixes, kRmc);

  telemetry::GnssFix fix;
  TEST_ASSERT_TRUE(fixes.takeCompletedFix(fix));
  TEST_ASSERT_FALSE(fixes.takeCompletedFix(fix));
  TEST_ASSERT_FALSE(fixes.hasCompleteFix());
}

void test_a_new_epoch_starts_cleanly_but_keeps_the_date(void) {
  nmea::SentenceAssembler assembler;
  nmea::FixAssembler fixes;
  feedIntoAssembler(assembler, fixes, kGga);
  feedIntoAssembler(assembler, fixes, kRmc);
  telemetry::GnssFix first;
  TEST_ASSERT_TRUE(fixes.takeCompletedFix(first));

  // The next GGA opens a new epoch: the previous altitude must not leak into
  // it, but the date, which only RMC carries, has to survive.
  feedIntoAssembler(assembler, fixes, kGgaNext);
  TEST_ASSERT_TRUE(fixes.partialFix().dateValid);
  TEST_ASSERT_EQUAL_UINT16(2026, fixes.partialFix().year);
  TEST_ASSERT_FALSE(fixes.hasCompleteFix());

  feedIntoAssembler(assembler, fixes, kRmcNext);
  telemetry::GnssFix second;
  TEST_ASSERT_TRUE(fixes.takeCompletedFix(second));
  TEST_ASSERT_EQUAL_UINT8(13, second.second);
  TEST_ASSERT_EQUAL_UINT16(0, second.millisecond);
}

void test_a_receiver_without_a_fix_reports_no_position(void) {
  nmea::SentenceAssembler assembler;
  nmea::FixAssembler fixes;
  feedIntoAssembler(assembler, fixes, kGgaNoFix);
  feedIntoAssembler(assembler, fixes, kRmcNoFix);

  telemetry::GnssFix fix;
  TEST_ASSERT_TRUE(fixes.takeCompletedFix(fix));
  TEST_ASSERT_FALSE(fix.navigationValid);
  TEST_ASSERT_FALSE(fix.positionValid);
  TEST_ASSERT_FALSE(fix.altitudeValid);
  TEST_ASSERT_EQUAL_UINT8(0, fix.fixQuality);
  // The date is still usable while the receiver is still hunting for a fix.
  TEST_ASSERT_TRUE(fix.dateValid);
}

void test_reset_clears_everything(void) {
  nmea::SentenceAssembler assembler;
  nmea::FixAssembler fixes;
  feedIntoAssembler(assembler, fixes, kGga);
  feedIntoAssembler(assembler, fixes, kRmc);
  fixes.reset();

  TEST_ASSERT_FALSE(fixes.hasCompleteFix());
  TEST_ASSERT_FALSE(fixes.partialFix().positionValid);
  TEST_ASSERT_FALSE(fixes.partialFix().dateValid);
}

int main(int, char**) {
  UNITY_BEGIN();

  RUN_TEST(test_checksum_matches_the_published_value);

  RUN_TEST(test_assembler_delivers_the_body_without_delimiters);
  RUN_TEST(test_assembler_rejects_a_corrupted_sentence);
  RUN_TEST(test_assembler_rejects_a_sentence_without_a_checksum);
  RUN_TEST(test_assembler_recovers_when_a_sentence_is_cut_short);
  RUN_TEST(test_assembler_survives_leading_noise);
  RUN_TEST(test_assembler_counts_an_overlong_sentence_as_an_overflow);

  RUN_TEST(test_parse_coordinate_keeps_full_resolution);
  RUN_TEST(test_parse_coordinate_rounds_the_last_digit);
  RUN_TEST(test_parse_coordinate_rejects_malformed_input);
  RUN_TEST(test_parse_time_of_day);
  RUN_TEST(test_parse_time_of_day_clamps_a_leap_second);
  RUN_TEST(test_parse_time_of_day_rejects_short_or_invalid_fields);
  RUN_TEST(test_parse_date);
  RUN_TEST(test_parse_number);

  RUN_TEST(test_rmc_populates_position_speed_and_date);
  RUN_TEST(test_gga_populates_altitude_satellites_and_quality);
  RUN_TEST(test_gsa_adds_vertical_dilution_of_precision);
  RUN_TEST(test_unknown_sentences_are_ignored);
  RUN_TEST(test_talker_id_does_not_matter);

  RUN_TEST(test_an_epoch_completes_once_rmc_and_gga_have_both_arrived);
  RUN_TEST(test_a_completed_fix_is_only_handed_out_once);
  RUN_TEST(test_a_new_epoch_starts_cleanly_but_keeps_the_date);
  RUN_TEST(test_a_receiver_without_a_fix_reports_no_position);
  RUN_TEST(test_reset_clears_everything);

  return UNITY_END();
}
