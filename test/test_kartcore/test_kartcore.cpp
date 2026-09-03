// SPDX-License-Identifier: MIT
//
// Unit tests for the SET button state machine, the session clock and the
// custom CAN channel payloads.

#include <unity.h>

#include "button.h"
#include "channels.h"
#include "session_timer.h"

using kart::ButtonDebouncer;

namespace {

/// Holds a level for `durationMs` in 1 ms steps, returning the last event that
/// was not kNone so a test can assert on it.
ButtonDebouncer::Event hold(ButtonDebouncer& button, bool pressed, uint32_t& clockMs,
                            uint32_t durationMs) {
  ButtonDebouncer::Event seen = ButtonDebouncer::Event::kNone;
  for (uint32_t i = 0; i < durationMs; ++i) {
    const ButtonDebouncer::Event event = button.update(pressed, clockMs);
    if (event != ButtonDebouncer::Event::kNone) {
      seen = event;
    }
    ++clockMs;
  }
  return seen;
}

} // namespace

void setUp(void) {}
void tearDown(void) {}

// --- Button debouncing ------------------------------------------------------

void test_button_ignores_contact_bounce(void) {
  ButtonDebouncer button(ButtonDebouncer::Config{25, 1500});
  uint32_t clock = 0;

  // 10 ms of chatter, well inside the 25 ms debounce window.
  for (int i = 0; i < 10; ++i) {
    const ButtonDebouncer::Event event = button.update(i % 2 == 0, clock++);
    TEST_ASSERT_TRUE(event == ButtonDebouncer::Event::kNone);
  }
  TEST_ASSERT_FALSE(button.isPressed());
}

void test_button_reports_a_press_after_the_debounce_window(void) {
  ButtonDebouncer button(ButtonDebouncer::Config{25, 1500});
  uint32_t clock = 0;

  TEST_ASSERT_TRUE(hold(button, true, clock, 24) == ButtonDebouncer::Event::kNone);
  TEST_ASSERT_TRUE(hold(button, true, clock, 2) == ButtonDebouncer::Event::kPressed);
  TEST_ASSERT_TRUE(button.isPressed());
}

void test_button_reports_a_short_press_on_release(void) {
  ButtonDebouncer button(ButtonDebouncer::Config{25, 1500});
  uint32_t clock = 0;

  hold(button, true, clock, 200);
  TEST_ASSERT_TRUE(button.isPressed());
  TEST_ASSERT_TRUE(hold(button, false, clock, 40) == ButtonDebouncer::Event::kShortPress);
  TEST_ASSERT_FALSE(button.isPressed());
}

void test_button_reports_a_long_press_without_waiting_for_release(void) {
  ButtonDebouncer button(ButtonDebouncer::Config{25, 1500});
  uint32_t clock = 0;

  // The press event lands first, then the long press once the hold matures.
  TEST_ASSERT_TRUE(hold(button, true, clock, 30) == ButtonDebouncer::Event::kPressed);
  TEST_ASSERT_TRUE(hold(button, true, clock, 1400) == ButtonDebouncer::Event::kNone);
  TEST_ASSERT_TRUE(hold(button, true, clock, 200) == ButtonDebouncer::Event::kLongPress);
}

void test_a_long_press_does_not_also_fire_a_short_press(void) {
  // A driver holding the button for a full reset must not trigger the
  // short-press action as well when they let go.
  ButtonDebouncer button(ButtonDebouncer::Config{25, 1500});
  uint32_t clock = 0;

  hold(button, true, clock, 2000);
  const ButtonDebouncer::Event onRelease = hold(button, false, clock, 40);
  TEST_ASSERT_TRUE(onRelease == ButtonDebouncer::Event::kReleased);
}

void test_long_press_fires_only_once_while_held(void) {
  ButtonDebouncer button(ButtonDebouncer::Config{25, 1500});
  uint32_t clock = 0;
  hold(button, true, clock, 1600);
  TEST_ASSERT_TRUE(hold(button, true, clock, 3000) == ButtonDebouncer::Event::kNone);
}

void test_button_reports_how_long_it_has_been_held(void) {
  ButtonDebouncer button(ButtonDebouncer::Config{25, 1500});
  uint32_t clock = 0;
  hold(button, true, clock, 526);
  // The press was registered 25 ms in, so the hold is 500 ms at clock 525.
  TEST_ASSERT_UINT32_WITHIN(2, 500, button.heldForMs(clock - 1));
  TEST_ASSERT_EQUAL_UINT32(0, ButtonDebouncer().heldForMs(clock));
}

void test_button_reset_returns_to_the_idle_state(void) {
  ButtonDebouncer button(ButtonDebouncer::Config{25, 1500});
  uint32_t clock = 0;
  hold(button, true, clock, 100);
  TEST_ASSERT_TRUE(button.isPressed());
  button.reset();
  TEST_ASSERT_FALSE(button.isPressed());
}

// --- Rate limiter -----------------------------------------------------------

void test_rate_limiter_allows_the_first_call_then_throttles(void) {
  kart::RateLimiter limiter(40); // 25 Hz
  TEST_ASSERT_TRUE(limiter.tryAcquire(1000));
  TEST_ASSERT_FALSE(limiter.tryAcquire(1020));
  TEST_ASSERT_FALSE(limiter.tryAcquire(1039));
  TEST_ASSERT_TRUE(limiter.tryAcquire(1040));
}

void test_rate_limiter_with_no_interval_never_blocks(void) {
  kart::RateLimiter limiter(0);
  TEST_ASSERT_TRUE(limiter.tryAcquire(0));
  TEST_ASSERT_TRUE(limiter.tryAcquire(0));
}

void test_rate_limiter_survives_the_millisecond_rollover(void) {
  kart::RateLimiter limiter(40);
  TEST_ASSERT_TRUE(limiter.tryAcquire(0xFFFFFFF0u));
  TEST_ASSERT_FALSE(limiter.tryAcquire(0x00000005u)); // 21 ms later
  TEST_ASSERT_TRUE(limiter.tryAcquire(0x00000020u));  // 48 ms later
}

// --- Session timer ----------------------------------------------------------

void test_session_timer_measures_from_the_last_reset(void) {
  kart::SessionTimer timer;
  timer.reset(10000);
  TEST_ASSERT_TRUE(timer.isRunning());
  TEST_ASSERT_EQUAL_UINT32(0, timer.elapsedMs(10000));
  TEST_ASSERT_EQUAL_UINT32(2500, timer.elapsedMs(12500));

  timer.reset(12500);
  TEST_ASSERT_EQUAL_UINT32(0, timer.elapsedMs(12500));
  TEST_ASSERT_EQUAL_UINT32(2, timer.resetCount());
}

void test_session_timer_survives_the_millisecond_rollover(void) {
  kart::SessionTimer timer;
  timer.reset(0xFFFFF000u);
  // 0x2000 ms later, having wrapped through zero.
  TEST_ASSERT_EQUAL_UINT32(0x2000u, timer.elapsedMs(0x00001000u));
}

void test_session_timer_freezes_when_stopped(void) {
  kart::SessionTimer timer;
  timer.reset(1000);
  timer.stop(4000);
  TEST_ASSERT_FALSE(timer.isRunning());
  TEST_ASSERT_EQUAL_UINT32(3000, timer.elapsedMs(9000));
}

void test_session_timer_clear_resets_the_counter(void) {
  kart::SessionTimer timer;
  timer.reset(1000);
  timer.clear();
  TEST_ASSERT_FALSE(timer.isRunning());
  TEST_ASSERT_EQUAL_UINT32(0, timer.resetCount());
  TEST_ASSERT_EQUAL_UINT32(0, timer.elapsedMs(5000));
}

// --- Channel payloads -------------------------------------------------------

void test_motion_payload_layout(void) {
  telemetry::ImuSample sample;
  sample.accelG[0] = 1.234f;   // 1234 milli-g
  sample.accelG[1] = -0.500f;  // -500 milli-g
  sample.accelG[2] = 0.0f;
  sample.gyroDps[0] = 12.3f;   // 123 units of 0.1 deg/s
  sample.gyroDps[1] = -45.6f;  // -456
  sample.gyroDps[2] = 200.0f;  // 2000

  uint8_t payload[kart::channels::kMotionPayloadSize];
  const size_t written =
      kart::channels::encodeMotionPayload(sample, payload, sizeof(payload));
  TEST_ASSERT_EQUAL_size_t(12, written);

  const uint8_t expected[12] = {
      0x04, 0xD2, // 1234
      0xFE, 0x0C, // -500
      0x00, 0x00, // 0
      0x00, 0x7B, // 123
      0xFE, 0x38, // -456
      0x07, 0xD0, // 2000
  };
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, payload, written);
}

void test_motion_payload_saturates_instead_of_wrapping(void) {
  // A 40 g kerb strike is beyond int16 milli-g. Wrapping would flip the sign
  // and paint a braking spike as acceleration.
  telemetry::ImuSample sample;
  sample.accelG[0] = 40.0f;
  sample.accelG[1] = -40.0f;

  uint8_t payload[kart::channels::kMotionPayloadSize];
  kart::channels::encodeMotionPayload(sample, payload, sizeof(payload));
  TEST_ASSERT_EQUAL_HEX8(0x7F, payload[0]);
  TEST_ASSERT_EQUAL_HEX8(0xFF, payload[1]);
  TEST_ASSERT_EQUAL_HEX8(0x80, payload[2]);
  TEST_ASSERT_EQUAL_HEX8(0x00, payload[3]);
}

void test_motion_payload_rejects_a_short_buffer(void) {
  telemetry::ImuSample sample;
  uint8_t payload[4];
  TEST_ASSERT_EQUAL_size_t(0, kart::channels::encodeMotionPayload(sample, payload, sizeof(payload)));
}

void test_status_payload_layout(void) {
  kart::channels::DeviceStatus status;
  status.sessionTimeMs = 0x0102030A;
  status.satellites = 11;
  status.satellitesValid = true;
  status.fixQuality = 1;
  status.flags = kart::channels::kFlagImuCalibrated | kart::channels::kFlagGnssFixValid;
  status.temperatureC = 31.5f;
  status.sessionResets = 3;

  uint8_t payload[kart::channels::kStatusPayloadSize];
  const size_t written = kart::channels::encodeStatusPayload(status, payload, sizeof(payload));
  TEST_ASSERT_EQUAL_size_t(10, written);

  const uint8_t expected[10] = {
      0x01, 0x02, 0x03, 0x0A, // session time, big-endian
      0x0B,                   // satellites
      0x01,                   // fix quality
      0x03,                   // flags
      0x01, 0x3B,             // 315 (31.5 C in tenths)
      0x03,                   // session resets
  };
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected, payload, written);
}

void test_status_payload_marks_an_unknown_satellite_count(void) {
  kart::channels::DeviceStatus status;
  status.satellites = 7;
  status.satellitesValid = false;

  uint8_t payload[kart::channels::kStatusPayloadSize];
  kart::channels::encodeStatusPayload(status, payload, sizeof(payload));
  TEST_ASSERT_EQUAL_HEX8(0xFF, payload[4]);
}

int main(int, char**) {
  UNITY_BEGIN();

  RUN_TEST(test_button_ignores_contact_bounce);
  RUN_TEST(test_button_reports_a_press_after_the_debounce_window);
  RUN_TEST(test_button_reports_a_short_press_on_release);
  RUN_TEST(test_button_reports_a_long_press_without_waiting_for_release);
  RUN_TEST(test_a_long_press_does_not_also_fire_a_short_press);
  RUN_TEST(test_long_press_fires_only_once_while_held);
  RUN_TEST(test_button_reports_how_long_it_has_been_held);
  RUN_TEST(test_button_reset_returns_to_the_idle_state);

  RUN_TEST(test_rate_limiter_allows_the_first_call_then_throttles);
  RUN_TEST(test_rate_limiter_with_no_interval_never_blocks);
  RUN_TEST(test_rate_limiter_survives_the_millisecond_rollover);

  RUN_TEST(test_session_timer_measures_from_the_last_reset);
  RUN_TEST(test_session_timer_survives_the_millisecond_rollover);
  RUN_TEST(test_session_timer_freezes_when_stopped);
  RUN_TEST(test_session_timer_clear_resets_the_counter);

  RUN_TEST(test_motion_payload_layout);
  RUN_TEST(test_motion_payload_saturates_instead_of_wrapping);
  RUN_TEST(test_motion_payload_rejects_a_short_buffer);
  RUN_TEST(test_status_payload_layout);
  RUN_TEST(test_status_payload_marks_an_unknown_satellite_count);

  return UNITY_END();
}
