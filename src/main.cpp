// SPDX-License-Identifier: MIT
//
// kart-gps: GNSS and inertial telemetry for RaceChrono, on an ESP32-C3.
//
// This file is deliberately thin. It owns the hardware objects and the loop
// schedule; every decision worth testing lives in lib/, which is compiled and
// tested on the host by `pio test -e native`.

#include <Arduino.h>
#include <Wire.h>

#include "button.h"
#include "channels.h"
#include "config.h"
#include "hal/ble_server.h"
#include "hal/gps_receiver.h"
#include "hal/mpu6050.h"
#include "imu.h"
#include "pipeline.h"

#if KARTGPS_LOG_LEVEL > 0
#define KARTGPS_LOG(...)                                                                           \
  do {                                                                                             \
    Serial.printf(__VA_ARGS__);                                                                    \
    Serial.println();                                                                              \
  } while (0)
#else
#define KARTGPS_LOG(...)                                                                           \
  do {                                                                                             \
  } while (0)
#endif

namespace {

// --- Hardware ---------------------------------------------------------------

HardwareSerial gpsUart(1);
hal::GpsReceiver gpsReceiver(gpsUart);
hal::Mpu6050 mpu(Wire);
hal::BleTelemetryServer bleServer;

// --- Processing -------------------------------------------------------------

kart::TelemetryPipeline::Config makePipelineConfig() {
  kart::TelemetryPipeline::Config config;
  config.motionPublishHz = KARTGPS_MOTION_PUBLISH_HZ;
  return config;
}

kart::TelemetryPipeline pipeline(bleServer, makePipelineConfig());
imu::MotionProcessor motionProcessor;
imu::BiasCalibrator calibrator;
kart::ButtonDebouncer setButton;

bool imuPresent = false;
uint32_t lastImuSampleMs = 0;
constexpr uint32_t kImuPeriodMs = 1000 / KARTGPS_IMU_SAMPLE_HZ;

// --- Status LED -------------------------------------------------------------

enum class LedPattern : uint8_t {
  kSearching,   ///< advertising, nobody connected
  kConnected,   ///< connected but no usable fix yet
  kReady,       ///< connected and logging
  kCalibrating, ///< the SET capture is running
  kFault,       ///< the last calibration was rejected, or the IMU is missing
};

void writeLed(bool on) {
#if KARTGPS_LED_ACTIVE_LOW
  digitalWrite(KARTGPS_PIN_STATUS_LED, on ? LOW : HIGH);
#else
  digitalWrite(KARTGPS_PIN_STATUS_LED, on ? HIGH : LOW);
#endif
}

void updateStatusLed(LedPattern pattern, uint32_t nowMs) {
  switch (pattern) {
    case LedPattern::kReady:
      writeLed(true);
      return;
    case LedPattern::kSearching:
      writeLed((nowMs % 1000) < 100); // a short wink once a second
      return;
    case LedPattern::kConnected:
      writeLed((nowMs % 500) < 250);
      return;
    case LedPattern::kCalibrating:
      writeLed((nowMs % 100) < 50);
      return;
    case LedPattern::kFault:
      // Two quick flashes, then a pause: distinguishable at a glance on a grid.
      writeLed((nowMs % 1200) < 100 || ((nowMs % 1200) >= 200 && (nowMs % 1200) < 300));
      return;
  }
}

LedPattern currentPattern(uint32_t nowMs) {
  if (calibrator.isRunning()) {
    return LedPattern::kCalibrating;
  }
  if (!imuPresent || calibrator.isRejected()) {
    return LedPattern::kFault;
  }
  if (!bleServer.isConnected()) {
    return LedPattern::kSearching;
  }
  return pipeline.isGnssStale(nowMs) ? LedPattern::kConnected : LedPattern::kReady;
}

// --- Actions ----------------------------------------------------------------

void startCalibration() {
  if (!imuPresent) {
    return;
  }
  calibrator.begin(KARTGPS_CALIBRATION_SAMPLES);
  pipeline.setCalibrationRunning(true);
  KARTGPS_LOG("[set] zeroing sensors, hold still");
}

/// Short press: zero the sensors and restart the session clock. This is what
/// the driver does once the kart is on the grid.
void handleShortPress(uint32_t nowMs) {
  pipeline.resetSession(nowMs);
  startCalibration();
  KARTGPS_LOG("[set] session reset at %lu ms", static_cast<unsigned long>(nowMs));
}

/// Long press: everything the short press does, plus a GNSS hot restart and a
/// sensor signal path reset. For when something is genuinely wrong.
void handleLongPress(uint32_t nowMs) {
  KARTGPS_LOG("[set] full reset");
  motionProcessor.clearCalibration();
  pipeline.setImuCalibrated(false);

  if (imuPresent && !mpu.resetSignalPath()) {
    imuPresent = false;
    KARTGPS_LOG("[imu] lost the sensor during reset");
  }

  gpsReceiver.requestHotStart();
  delay(200); // let the receiver restart before it is reconfigured
  gpsReceiver.applyConfiguration();

  pipeline.gpsEncoder().reset();
  pipeline.resetSession(nowMs);
  startCalibration();
}

void serviceButton(uint32_t nowMs) {
  const bool pressed = digitalRead(KARTGPS_PIN_SET_BUTTON) == LOW;
  switch (setButton.update(pressed, nowMs)) {
    case kart::ButtonDebouncer::Event::kShortPress:
      handleShortPress(nowMs);
      break;
    case kart::ButtonDebouncer::Event::kLongPress:
      handleLongPress(nowMs);
      break;
    default:
      break;
  }
}

void serviceImu(uint32_t nowMs) {
  if (!imuPresent || static_cast<uint32_t>(nowMs - lastImuSampleMs) < kImuPeriodMs) {
    return;
  }
  lastImuSampleMs = nowMs;

  telemetry::ImuRawSample raw;
  if (!mpu.read(raw, nowMs)) {
    return;
  }
  const telemetry::ImuSample sample =
      imu::toPhysicalUnits(raw, mpu.config().accelRange, mpu.config().gyroRange);
  pipeline.setDeviceTemperature(sample.temperatureC);

  if (calibrator.isRunning()) {
    // While the window is open the samples belong to the calibrator, not to
    // the app: publishing them would log the kart accelerating out of the pits
    // with a stale zero.
    if (calibrator.addSample(sample)) {
      pipeline.setCalibrationRunning(false);
      if (calibrator.isComplete()) {
        motionProcessor.setBias(calibrator.bias());
        pipeline.setImuCalibrated(true);
        KARTGPS_LOG("[imu] zeroed, gyro bias %.2f %.2f %.2f dps", calibrator.bias().gyroDps[0],
                    calibrator.bias().gyroDps[1], calibrator.bias().gyroDps[2]);
      } else {
        pipeline.setCalibrationRejected(true);
        KARTGPS_LOG("[imu] zeroing rejected, the kart was not still");
      }
    }
    return;
  }

  pipeline.onMotionSample(motionProcessor.process(sample), nowMs);
}

void serviceGps(uint32_t nowMs) {
  if (gpsReceiver.poll(nowMs)) {
    pipeline.onGnssFix(gpsReceiver.fix(), nowMs);
  }
}

#if KARTGPS_LOG_LEVEL > 1
void logHeartbeat(uint32_t nowMs) {
  static uint32_t lastLogMs = 0;
  if (static_cast<uint32_t>(nowMs - lastLogMs) < 5000) {
    return;
  }
  lastLogMs = nowMs;
  const kart::TelemetryPipeline::Stats& stats = pipeline.stats();
  KARTGPS_LOG("[stat] fixes=%lu gps=%lu motion=%lu status=%lu nmeaErr=%lu heap=%lu",
              static_cast<unsigned long>(gpsReceiver.fixCount()),
              static_cast<unsigned long>(stats.gpsMainSent),
              static_cast<unsigned long>(stats.motionSent),
              static_cast<unsigned long>(stats.statusSent),
              static_cast<unsigned long>(gpsReceiver.sentenceStats().checksumErrors),
              static_cast<unsigned long>(ESP.getFreeHeap()));
}
#endif

} // namespace

void setup() {
#if KARTGPS_LOG_LEVEL > 0
  Serial.begin(115200);
  // Do not block forever waiting for a console: the kart has no keyboard.
  const uint32_t serialDeadline = millis() + 1500;
  while (!Serial && millis() < serialDeadline) {
    delay(10);
  }
#endif
  KARTGPS_LOG("\n[boot] kart-gps starting");

  pinMode(KARTGPS_PIN_STATUS_LED, OUTPUT);
  writeLed(true);
  pinMode(KARTGPS_PIN_SET_BUTTON, INPUT_PULLUP);
  setButton.configure({KARTGPS_BUTTON_DEBOUNCE_MS, KARTGPS_BUTTON_LONG_PRESS_MS});

  Wire.begin(KARTGPS_PIN_I2C_SDA, KARTGPS_PIN_I2C_SCL, 400000);

  hal::Mpu6050::Config imuConfig;
  imuConfig.sampleRateHz = KARTGPS_IMU_SAMPLE_HZ;
  imuPresent = mpu.begin(imuConfig);
  KARTGPS_LOG("[imu] %s", imuPresent ? "MPU-6050 ready" : "MPU-6050 NOT FOUND, check wiring");

  hal::GpsReceiver::Config gpsConfig;
  gpsConfig.rxPin = KARTGPS_PIN_GPS_RX;
  gpsConfig.txPin = KARTGPS_PIN_GPS_TX;
  gpsConfig.bootBaud = KARTGPS_GPS_BOOT_BAUD;
  gpsConfig.runBaud = KARTGPS_GPS_RUN_BAUD;
  gpsConfig.rateHz = KARTGPS_GPS_RATE_HZ;
  gpsReceiver.begin(gpsConfig);
  KARTGPS_LOG("[gps] configured for %d Hz at %lu baud", KARTGPS_GPS_RATE_HZ,
              static_cast<unsigned long>(KARTGPS_GPS_RUN_BAUD));

  motionProcessor.setMountingYawDegrees(KARTGPS_MOUNTING_YAW_DEGREES);

  hal::BleTelemetryServer::Config bleConfig;
  bleConfig.deviceName = KARTGPS_BLE_DEVICE_NAME;
  bleConfig.minConnectionInterval = KARTGPS_BLE_MIN_INTERVAL;
  bleConfig.maxConnectionInterval = KARTGPS_BLE_MAX_INTERVAL;
  bleConfig.supervisionTimeout = KARTGPS_BLE_TIMEOUT;
  bleConfig.txPowerDbm = KARTGPS_BLE_TX_POWER_DBM;
  bleServer.begin(bleConfig);
  KARTGPS_LOG("[ble] advertising as %s", KARTGPS_BLE_DEVICE_NAME);

  const uint32_t now = millis();
  pipeline.resetSession(now);
  lastImuSampleMs = now;
  // The device is almost always sitting still on the bench at power-up, so
  // take a zero straight away; the driver only has to press SET if it is not.
  startCalibration();
}

void loop() {
  const uint32_t now = millis();

  bleServer.drainEvents(pipeline);
  serviceGps(now);
  serviceImu(now);
  serviceButton(now);
  pipeline.tick(now);
  updateStatusLed(currentPattern(now), now);
#if KARTGPS_LOG_LEVEL > 1
  logHeartbeat(now);
#endif
}
