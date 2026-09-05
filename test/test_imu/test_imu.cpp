// SPDX-License-Identifier: MIT
//
// Unit tests for MPU-6050 conversion, zeroing and mounting compensation.

#include <unity.h>

#include <math.h>

#include "imu.h"

namespace {

constexpr float kGravity = 1.0f;

telemetry::ImuSample makeSample(float ax, float ay, float az, float gx = 0.0f, float gy = 0.0f,
                                float gz = 0.0f) {
  telemetry::ImuSample sample;
  sample.accelG[0] = ax;
  sample.accelG[1] = ay;
  sample.accelG[2] = az;
  sample.gyroDps[0] = gx;
  sample.gyroDps[1] = gy;
  sample.gyroDps[2] = gz;
  return sample;
}

/// Runs a full calibration window with the device held in one attitude.
imu::SensorBias calibrateAt(float ax, float ay, float az, float gx = 0.0f, float gy = 0.0f,
                            float gz = 0.0f) {
  imu::BiasCalibrator calibrator;
  calibrator.begin(64);
  for (int i = 0; i < 64; ++i) {
    calibrator.addSample(makeSample(ax, ay, az, gx, gy, gz));
  }
  return calibrator.bias();
}

} // namespace

void setUp(void) {}
void tearDown(void) {}

// --- Scaling ----------------------------------------------------------------

void test_accelerometer_scale_factors(void) {
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 16384.0f, imu::accelScaleLsbPerG(imu::AccelRange::k2G));
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 8192.0f, imu::accelScaleLsbPerG(imu::AccelRange::k4G));
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 4096.0f, imu::accelScaleLsbPerG(imu::AccelRange::k8G));
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 2048.0f, imu::accelScaleLsbPerG(imu::AccelRange::k16G));
}

void test_gyroscope_scale_factors(void) {
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 131.0f, imu::gyroScaleLsbPerDps(imu::GyroRange::k250Dps));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 65.5f, imu::gyroScaleLsbPerDps(imu::GyroRange::k500Dps));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 32.8f, imu::gyroScaleLsbPerDps(imu::GyroRange::k1000Dps));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 16.4f, imu::gyroScaleLsbPerDps(imu::GyroRange::k2000Dps));
}

// --- Factory trim correction ------------------------------------------------

void test_raw_bias_is_subtracted_from_every_axis(void) {
  telemetry::ImuRawSample sample;
  sample.accel[0] = 1754;
  sample.accel[1] = 4416;
  sample.accel[2] = -2084;

  imu::RawAccelBias bias;
  bias.count[0] = -6344;
  bias.count[1] = 5123;
  bias.count[2] = -963;
  imu::applyRawAccelBias(sample, bias);

  TEST_ASSERT_EQUAL_INT16(8098, sample.accel[0]);
  TEST_ASSERT_EQUAL_INT16(-707, sample.accel[1]);
  TEST_ASSERT_EQUAL_INT16(-1121, sample.accel[2]);
}

void test_raw_bias_correction_restores_one_g(void) {
  // The reading the bench part gives at rest comes to 0.63 g uncorrected,
  // which the zeroing rejects. Removing the measured trim brings it back to
  // the 1 g that a stationary device has to read.
  telemetry::ImuRawSample sample;
  sample.accel[0] = 1754;
  sample.accel[1] = 4416;
  sample.accel[2] = -2084;

  imu::RawAccelBias bias;
  bias.count[0] = -6344;
  bias.count[1] = 5123;
  bias.count[2] = -963;
  imu::applyRawAccelBias(sample, bias);

  const telemetry::ImuSample converted =
      imu::toPhysicalUnits(sample, imu::AccelRange::k4G, imu::GyroRange::k500Dps);
  const float magnitude =
      sqrtf(converted.accelG[0] * converted.accelG[0] + converted.accelG[1] * converted.accelG[1] +
            converted.accelG[2] * converted.accelG[2]);
  TEST_ASSERT_FLOAT_WITHIN(0.05f, 1.0f, magnitude);
}

void test_raw_bias_saturates_instead_of_wrapping(void) {
  // A large offset near full scale must not turn a pegged positive reading
  // into a large negative one.
  telemetry::ImuRawSample sample;
  sample.accel[0] = 32000;
  sample.accel[1] = -32000;
  sample.accel[2] = 0;

  imu::RawAccelBias bias;
  bias.count[0] = -30000;
  bias.count[1] = 30000;
  imu::applyRawAccelBias(sample, bias);

  TEST_ASSERT_EQUAL_INT16(32767, sample.accel[0]);
  TEST_ASSERT_EQUAL_INT16(-32768, sample.accel[1]);
  TEST_ASSERT_EQUAL_INT16(0, sample.accel[2]);
}

void test_a_zero_bias_changes_nothing(void) {
  telemetry::ImuRawSample sample;
  sample.accel[0] = 123;
  sample.accel[1] = -456;
  sample.accel[2] = 789;

  const imu::RawAccelBias bias;
  TEST_ASSERT_TRUE(bias.isZero());
  imu::applyRawAccelBias(sample, bias);

  TEST_ASSERT_EQUAL_INT16(123, sample.accel[0]);
  TEST_ASSERT_EQUAL_INT16(-456, sample.accel[1]);
  TEST_ASSERT_EQUAL_INT16(789, sample.accel[2]);
}

// --- Accelerometer characterisation -----------------------------------------

/// Feeds one attitude to the characteriser long enough to clear the settle
/// window, the way resting the board on a face does.
void holdAt(imu::AccelCharacterizer& characterizer, int16_t x, int16_t y, int16_t z) {
  telemetry::ImuRawSample raw;
  raw.accel[0] = x;
  raw.accel[1] = y;
  raw.accel[2] = z;
  for (int i = 0; i < imu::AccelCharacterizer::kSettleSamples + 5; ++i) {
    characterizer.addSample(raw);
  }
}

/// Walks the six faces of a part whose axes carry `offset` counts of bias and
/// respond with `sensitivity` counts per g.
void tumbleThroughSixFaces(imu::AccelCharacterizer& characterizer, int16_t sensitivity,
                           int16_t offset) {
  holdAt(characterizer, static_cast<int16_t>(sensitivity + offset), offset, offset);
  holdAt(characterizer, static_cast<int16_t>(-sensitivity + offset), offset, offset);
  holdAt(characterizer, offset, static_cast<int16_t>(sensitivity + offset), offset);
  holdAt(characterizer, offset, static_cast<int16_t>(-sensitivity + offset), offset);
  holdAt(characterizer, offset, offset, static_cast<int16_t>(sensitivity + offset));
  holdAt(characterizer, offset, offset, static_cast<int16_t>(-sensitivity + offset));
}

void test_characteriser_recovers_sensitivity_and_offset(void) {
  imu::AccelCharacterizer characterizer;
  characterizer.reset();
  tumbleThroughSixFaces(characterizer, 8192, 0);

  TEST_ASSERT_TRUE(characterizer.isComplete());
  for (size_t axis = 0; axis < 3; ++axis) {
    const imu::AccelAxisFit fit = characterizer.axis(axis);
    TEST_ASSERT_TRUE(fit.complete);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 8192.0f, fit.sensitivityLsbPerG);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, fit.offsetCount);
  }
}

void test_characteriser_separates_a_bias_from_a_scale_error(void) {
  // The part on the bench reads low and its magnitude changes with attitude,
  // which is what a bias does and a pure scale error cannot.
  imu::AccelCharacterizer characterizer;
  characterizer.reset();
  tumbleThroughSixFaces(characterizer, 5200, 900);

  TEST_ASSERT_TRUE(characterizer.isComplete());
  for (size_t axis = 0; axis < 3; ++axis) {
    const imu::AccelAxisFit fit = characterizer.axis(axis);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 5200.0f, fit.sensitivityLsbPerG);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 900.0f, fit.offsetCount);
  }
}

void test_characteriser_is_incomplete_from_a_single_attitude(void) {
  imu::AccelCharacterizer characterizer;
  characterizer.reset();
  holdAt(characterizer, 1754, 4416, -2084);

  TEST_ASSERT_FALSE(characterizer.isComplete());
  TEST_ASSERT_FALSE(characterizer.axis(0).complete);
}

void test_characteriser_ignores_samples_taken_while_moving(void) {
  imu::AccelCharacterizer characterizer;
  characterizer.reset();

  // A swing between two faces must not be mistaken for either of them.
  const int step = imu::AccelCharacterizer::kStillDeltaCounts + 50;
  telemetry::ImuRawSample raw;
  for (int i = 0; i < 100; ++i) {
    raw.accel[0] = static_cast<int16_t>(i * step);
    raw.accel[1] = 0;
    raw.accel[2] = 0;
    TEST_ASSERT_FALSE(characterizer.addSample(raw));
  }
  TEST_ASSERT_EQUAL_UINT32(0, characterizer.acceptedSamples());
}

// --- Device identity --------------------------------------------------------

void test_who_am_i_accepts_the_genuine_part(void) {
  TEST_ASSERT_TRUE(imu::isCompatibleWhoAmI(imu::kWhoAmIValue));
}

void test_who_am_i_accepts_the_substituted_parts(void) {
  // Modules sold as GY-521 boards routinely carry one of these instead. They
  // share the register map and the scale factors, so the driver must not
  // refuse them.
  TEST_ASSERT_TRUE(imu::isCompatibleWhoAmI(0x70));
  TEST_ASSERT_TRUE(imu::isCompatibleWhoAmI(0x71));
  TEST_ASSERT_TRUE(imu::isCompatibleWhoAmI(0x72));
  TEST_ASSERT_TRUE(imu::isCompatibleWhoAmI(0x73));
}

void test_who_am_i_still_rejects_an_unrelated_device(void) {
  // The check exists to catch a read that landed somewhere else entirely, so
  // it has to keep saying no to values outside the family.
  TEST_ASSERT_FALSE(imu::isCompatibleWhoAmI(0x00));
  TEST_ASSERT_FALSE(imu::isCompatibleWhoAmI(0xFF));
  TEST_ASSERT_FALSE(imu::isCompatibleWhoAmI(0x1A));
  TEST_ASSERT_FALSE(imu::isCompatibleWhoAmI(0xD0));
}

// --- Sample rate ------------------------------------------------------------

void test_sample_rate_divider_with_the_filter_enabled(void) {
  // With any DLPF setting the internal rate is 1 kHz, so 100 Hz needs a
  // divider of 9.
  TEST_ASSERT_EQUAL_UINT8(9, imu::sampleRateDivider(100, imu::DlpfBandwidth::k44Hz));
  TEST_ASSERT_EQUAL_UINT8(3, imu::sampleRateDivider(250, imu::DlpfBandwidth::k44Hz));
  TEST_ASSERT_EQUAL_UINT8(0, imu::sampleRateDivider(1000, imu::DlpfBandwidth::k44Hz));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, imu::outputRateHz(9, imu::DlpfBandwidth::k44Hz));
}

void test_sample_rate_divider_with_the_filter_bypassed(void) {
  // Bypassing the DLPF raises the internal rate to 8 kHz.
  TEST_ASSERT_EQUAL_UINT8(79, imu::sampleRateDivider(100, imu::DlpfBandwidth::k260Hz));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 100.0f, imu::outputRateHz(79, imu::DlpfBandwidth::k260Hz));
}

void test_sample_rate_divider_never_overshoots_the_request(void) {
  // 300 Hz does not divide 1000 exactly. Rounding up the divider gives 250 Hz,
  // which is under budget; rounding down would give 333 Hz and blow the BLE
  // bandwidth the link was sized for.
  const uint8_t divider = imu::sampleRateDivider(300, imu::DlpfBandwidth::k44Hz);
  TEST_ASSERT_TRUE(imu::outputRateHz(divider, imu::DlpfBandwidth::k44Hz) <= 300.0f);
}

// --- Burst decoding ---------------------------------------------------------

void test_decode_burst_reads_big_endian_two_complement(void) {
  const uint8_t raw[imu::kBurstLength] = {
      0x12, 0x34,             // accel X = 4660
      0xFF, 0xFF,             // accel Y = -1
      0x20, 0x00,             // accel Z = 8192
      0x00, 0x00,             // temperature
      0x80, 0x00,             // gyro X = -32768
      0x00, 0x64,             // gyro Y = 100
      0xFE, 0x0C,             // gyro Z = -500
  };
  telemetry::ImuRawSample sample;
  TEST_ASSERT_TRUE(imu::decodeBurst(raw, sizeof(raw), sample));
  TEST_ASSERT_EQUAL_INT16(4660, sample.accel[0]);
  TEST_ASSERT_EQUAL_INT16(-1, sample.accel[1]);
  TEST_ASSERT_EQUAL_INT16(8192, sample.accel[2]);
  TEST_ASSERT_EQUAL_INT16(-32768, sample.gyro[0]);
  TEST_ASSERT_EQUAL_INT16(100, sample.gyro[1]);
  TEST_ASSERT_EQUAL_INT16(-500, sample.gyro[2]);
}

void test_decode_burst_rejects_a_short_read(void) {
  const uint8_t raw[8] = {0};
  telemetry::ImuRawSample sample;
  TEST_ASSERT_FALSE(imu::decodeBurst(raw, sizeof(raw), sample));
}

void test_conversion_to_physical_units(void) {
  telemetry::ImuRawSample raw;
  raw.accel[2] = 8192;  // exactly 1 g at the 4 g range
  raw.gyro[2] = 655;    // 10 deg/s at the 500 dps range
  raw.temperature = 0;  // 36.53 C by the datasheet formula

  const telemetry::ImuSample sample =
      imu::toPhysicalUnits(raw, imu::AccelRange::k4G, imu::GyroRange::k500Dps);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.0f, sample.accelG[2]);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, sample.gyroDps[2]);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 36.53f, sample.temperatureC);
}

// --- Rotation maths ---------------------------------------------------------

void test_rotation_aligning_identical_vectors_is_the_identity(void) {
  const float up[3] = {0.0f, 0.0f, 1.0f};
  const imu::Mat3 rotation = imu::rotationAligningVector(up, up);
  float out[3];
  const float sample[3] = {1.0f, 2.0f, 3.0f};
  rotation.apply(sample, out);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, out[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 2.0f, out[1]);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 3.0f, out[2]);
}

void test_rotation_aligns_a_tilted_vector_onto_up(void) {
  // A device lying on its side reads gravity along +X.
  const float measured[3] = {1.0f, 0.0f, 0.0f};
  const float up[3] = {0.0f, 0.0f, 1.0f};
  const imu::Mat3 rotation = imu::rotationAligningVector(measured, up);

  float out[3];
  rotation.apply(measured, out);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, out[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, out[1]);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, out[2]);
}

void test_rotation_handles_an_upside_down_device(void) {
  // The antiparallel case is the one that makes a naive Rodrigues formula
  // divide by zero.
  const float measured[3] = {0.0f, 0.0f, -1.0f};
  const float up[3] = {0.0f, 0.0f, 1.0f};
  const imu::Mat3 rotation = imu::rotationAligningVector(measured, up);

  float out[3];
  rotation.apply(measured, out);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, out[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, out[1]);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, out[2]);
}

void test_rotation_about_z(void) {
  const imu::Mat3 rotation = imu::rotationAboutZ(90.0f);
  const float forward[3] = {1.0f, 0.0f, 0.0f};
  float out[3];
  rotation.apply(forward, out);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, out[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 1.0f, out[1]);
  TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, out[2]);
}

// --- Bias calibration -------------------------------------------------------

void test_calibration_captures_gyro_bias_and_gravity(void) {
  imu::BiasCalibrator calibrator;
  calibrator.begin(32);
  TEST_ASSERT_TRUE(calibrator.isRunning());

  for (int i = 0; i < 31; ++i) {
    TEST_ASSERT_FALSE(calibrator.addSample(makeSample(0.01f, -0.02f, kGravity, 1.5f, -0.5f, 0.25f)));
  }
  TEST_ASSERT_TRUE(calibrator.addSample(makeSample(0.01f, -0.02f, kGravity, 1.5f, -0.5f, 0.25f)));

  TEST_ASSERT_TRUE(calibrator.isComplete());
  const imu::SensorBias& bias = calibrator.bias();
  TEST_ASSERT_TRUE(bias.valid);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 1.5f, bias.gyroDps[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, -0.5f, bias.gyroDps[1]);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.25f, bias.gyroDps[2]);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, kGravity, bias.gravityG[2]);
}

void test_calibration_is_rejected_when_the_kart_is_moving(void) {
  imu::BiasCalibrator calibrator;
  calibrator.begin(32);
  for (int i = 0; i < 32; ++i) {
    // Alternating half-g swings: far beyond the stationary tolerance.
    const float wobble = (i % 2 == 0) ? 0.5f : -0.5f;
    calibrator.addSample(makeSample(wobble, 0.0f, kGravity, 0.0f, 0.0f, 0.0f));
  }
  TEST_ASSERT_TRUE(calibrator.isRejected());
  TEST_ASSERT_FALSE(calibrator.bias().valid);
}

void test_calibration_is_rejected_when_the_gyro_will_not_settle(void) {
  imu::BiasCalibrator calibrator;
  calibrator.begin(32);
  for (int i = 0; i < 32; ++i) {
    const float wobble = (i % 2 == 0) ? 40.0f : -40.0f;
    calibrator.addSample(makeSample(0.0f, 0.0f, kGravity, 0.0f, 0.0f, wobble));
  }
  TEST_ASSERT_TRUE(calibrator.isRejected());
}

void test_calibration_is_rejected_when_gravity_is_implausible(void) {
  // A steady reading that is nowhere near 1 g means the sensor range is
  // misconfigured or the part is faulty; zeroing on it would be worse than
  // refusing.
  imu::BiasCalibrator calibrator;
  calibrator.begin(32);
  for (int i = 0; i < 32; ++i) {
    calibrator.addSample(makeSample(0.0f, 0.0f, 0.5f, 0.0f, 0.0f, 0.0f));
  }
  TEST_ASSERT_TRUE(calibrator.isRejected());
}

void test_calibration_ignores_samples_when_not_running(void) {
  imu::BiasCalibrator calibrator;
  TEST_ASSERT_FALSE(calibrator.addSample(makeSample(0.0f, 0.0f, kGravity, 0.0f, 0.0f, 0.0f)));
  TEST_ASSERT_EQUAL_UINT16(0, calibrator.collected());
}

// --- Motion processing ------------------------------------------------------

void test_processor_subtracts_gyro_bias(void) {
  imu::MotionProcessor processor;
  processor.setBias(calibrateAt(0.0f, 0.0f, kGravity, 2.0f, -3.0f, 1.0f));

  const telemetry::ImuSample out =
      processor.process(makeSample(0.0f, 0.0f, kGravity, 12.0f, -3.0f, 1.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 10.0f, out.gyroDps[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, out.gyroDps[1]);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, out.gyroDps[2]);
}

void test_a_calibrated_stationary_device_reads_zero_on_every_axis(void) {
  imu::MotionProcessor processor;
  processor.setBias(calibrateAt(0.0f, 0.0f, kGravity, 0.0f, 0.0f, 0.0f));

  const telemetry::ImuSample out =
      processor.process(makeSample(0.0f, 0.0f, kGravity, 0.0f, 0.0f, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, out.accelG[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, out.accelG[1]);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, out.accelG[2]);
}

void test_processor_levels_a_device_mounted_on_its_side(void) {
  // Mounted so that gravity lands on +X. After levelling, a real forward
  // acceleration along the sensor's +Z has to come out on the vehicle's +X.
  imu::MotionProcessor processor;
  processor.setBias(calibrateAt(kGravity, 0.0f, 0.0f));

  const telemetry::ImuSample out = processor.process(makeSample(kGravity, 0.0f, 0.0f));
  // Gravity alone: every vehicle axis reads zero once it is compensated.
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, out.accelG[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, out.accelG[1]);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, out.accelG[2]);

  // A 0.5 g bump straight up in the world frame is measured along sensor +X.
  const telemetry::ImuSample bump = processor.process(makeSample(kGravity + 0.5f, 0.0f, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.5f, bump.accelG[2]);
}

void test_processor_can_keep_gravity_when_asked(void) {
  imu::MotionProcessor processor;
  processor.setBias(calibrateAt(0.0f, 0.0f, kGravity));
  processor.setGravityCompensation(false);

  const telemetry::ImuSample out = processor.process(makeSample(0.0f, 0.0f, kGravity));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, out.accelG[2]);
}

void test_mounting_yaw_rotates_the_horizontal_axes(void) {
  imu::MotionProcessor processor;
  processor.setBias(calibrateAt(0.0f, 0.0f, kGravity));
  // The box points 90 degrees off the direction of travel.
  processor.setMountingYawDegrees(90.0f);

  // 1 g along the sensor's +X should appear on the vehicle's +Y.
  const telemetry::ImuSample out = processor.process(makeSample(1.0f, 0.0f, kGravity));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, out.accelG[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, out.accelG[1]);
}

void test_an_uncalibrated_processor_passes_samples_through(void) {
  imu::MotionProcessor processor;
  TEST_ASSERT_FALSE(processor.isCalibrated());

  const telemetry::ImuSample out = processor.process(makeSample(0.1f, 0.2f, 0.9f, 1.0f, 2.0f, 3.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.1f, out.accelG[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.9f, out.accelG[2]);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.0f, out.gyroDps[2]);
}

void test_clearing_the_calibration_restores_pass_through(void) {
  imu::MotionProcessor processor;
  processor.setBias(calibrateAt(0.0f, 0.0f, kGravity, 5.0f, 0.0f, 0.0f));
  TEST_ASSERT_TRUE(processor.isCalibrated());

  processor.clearCalibration();
  TEST_ASSERT_FALSE(processor.isCalibrated());
  const telemetry::ImuSample out = processor.process(makeSample(0.0f, 0.0f, kGravity, 5.0f, 0.0f, 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 5.0f, out.gyroDps[0]);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f, out.accelG[2]);
}

int main(int, char**) {
  UNITY_BEGIN();

  RUN_TEST(test_accelerometer_scale_factors);
  RUN_TEST(test_gyroscope_scale_factors);

  RUN_TEST(test_raw_bias_is_subtracted_from_every_axis);
  RUN_TEST(test_raw_bias_correction_restores_one_g);
  RUN_TEST(test_raw_bias_saturates_instead_of_wrapping);
  RUN_TEST(test_a_zero_bias_changes_nothing);

  RUN_TEST(test_characteriser_recovers_sensitivity_and_offset);
  RUN_TEST(test_characteriser_separates_a_bias_from_a_scale_error);
  RUN_TEST(test_characteriser_is_incomplete_from_a_single_attitude);
  RUN_TEST(test_characteriser_ignores_samples_taken_while_moving);

  RUN_TEST(test_who_am_i_accepts_the_genuine_part);
  RUN_TEST(test_who_am_i_accepts_the_substituted_parts);
  RUN_TEST(test_who_am_i_still_rejects_an_unrelated_device);

  RUN_TEST(test_sample_rate_divider_with_the_filter_enabled);
  RUN_TEST(test_sample_rate_divider_with_the_filter_bypassed);
  RUN_TEST(test_sample_rate_divider_never_overshoots_the_request);

  RUN_TEST(test_decode_burst_reads_big_endian_two_complement);
  RUN_TEST(test_decode_burst_rejects_a_short_read);
  RUN_TEST(test_conversion_to_physical_units);

  RUN_TEST(test_rotation_aligning_identical_vectors_is_the_identity);
  RUN_TEST(test_rotation_aligns_a_tilted_vector_onto_up);
  RUN_TEST(test_rotation_handles_an_upside_down_device);
  RUN_TEST(test_rotation_about_z);

  RUN_TEST(test_calibration_captures_gyro_bias_and_gravity);
  RUN_TEST(test_calibration_is_rejected_when_the_kart_is_moving);
  RUN_TEST(test_calibration_is_rejected_when_the_gyro_will_not_settle);
  RUN_TEST(test_calibration_is_rejected_when_gravity_is_implausible);
  RUN_TEST(test_calibration_ignores_samples_when_not_running);

  RUN_TEST(test_processor_subtracts_gyro_bias);
  RUN_TEST(test_a_calibrated_stationary_device_reads_zero_on_every_axis);
  RUN_TEST(test_processor_levels_a_device_mounted_on_its_side);
  RUN_TEST(test_processor_can_keep_gravity_when_asked);
  RUN_TEST(test_mounting_yaw_rotates_the_horizontal_axes);
  RUN_TEST(test_an_uncalibrated_processor_passes_samples_through);
  RUN_TEST(test_clearing_the_calibration_restores_pass_through);

  return UNITY_END();
}
