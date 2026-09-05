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
#include "diagnostics.h"
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
imu::AccelCharacterizer characterizer;
kart::ButtonDebouncer setButton;

imu::RawAccelBias makeRawAccelBias() {
  imu::RawAccelBias bias;
  bias.count[0] = KARTGPS_ACCEL_OFFSET_X;
  bias.count[1] = KARTGPS_ACCEL_OFFSET_Y;
  bias.count[2] = KARTGPS_ACCEL_OFFSET_Z;
  return bias;
}

const imu::RawAccelBias rawAccelBias = makeRawAccelBias();

/// Set by the console 'c' command while a six position capture is running.
bool characterizing = false;
uint32_t lastCharacterizeLogMs = 0;

bool imuPresent = false;
uint32_t lastImuSampleMs = 0;
constexpr uint32_t kImuPeriodMs = 1000 / KARTGPS_IMU_SAMPLE_HZ;

hal::Mpu6050::Config makeImuConfig() {
  hal::Mpu6050::Config config;
  config.address = KARTGPS_MPU_ADDRESS;
  config.sampleRateHz = KARTGPS_IMU_SAMPLE_HZ;
  return config;
}

/// What the start-up probe found. The USB CDC on this board discards anything
/// printed before the host opens the port, so the boot log is routinely lost;
/// keeping the result lets the console reprint it on demand.
struct BootReport {
  bool imuPresent = false;
  bool whoAmIRead = false;
  uint8_t whoAmI = 0;
  uint8_t probeError = 0;
};

BootReport bootReport;

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
  // Before anything else looks at the sample, including the characteriser and
  // the 1 g check inside the zeroing.
  imu::applyRawAccelBias(raw, rawAccelBias);
  const telemetry::ImuSample sample =
      imu::toPhysicalUnits(raw, mpu.config().accelRange, mpu.config().gyroRange);
  pipeline.setDeviceTemperature(sample.temperatureC);

  if (characterizing) {
    characterizer.addSample(raw);
  }

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

const char* calibratorStateName() {
  switch (calibrator.state()) {
    case imu::BiasCalibrator::State::kIdle:
      return "idle";
    case imu::BiasCalibrator::State::kRunning:
      return "running";
    case imu::BiasCalibrator::State::kComplete:
      return "complete";
    case imu::BiasCalibrator::State::kRejected:
      return "REJECTED";
  }
  return "unknown";
}

// --- Console ----------------------------------------------------------------
//
// One key per command on the USB serial port. This exists because the two
// buses that matter are internal wires: without a logic analyser the only way
// to see the I2C traffic and the GPS UART is to ask the firmware what it sees.

#if KARTGPS_ENABLE_CONSOLE

const char* ledPatternName(LedPattern pattern) {
  switch (pattern) {
    case LedPattern::kSearching:
      return "searching (one wink a second): advertising, no phone connected";
    case LedPattern::kConnected:
      return "connected (even blink): phone attached, no usable GNSS fix";
    case LedPattern::kReady:
      return "ready (solid): connected and logging";
    case LedPattern::kCalibrating:
      return "calibrating (fast flicker): the zeroing window is open";
    case LedPattern::kFault:
      return "FAULT (two quick flashes, then a pause)";
  }
  return "unknown";
}

void printBootReport() {
  Serial.println("[boot] kart-gps, replayed start-up probe");
  if (bootReport.whoAmIRead) {
    Serial.printf("[boot] WHO_AM_I at 0x%02X read back 0x%02X (expected 0x%02X)\n",
                  KARTGPS_MPU_ADDRESS, bootReport.whoAmI, imu::kWhoAmIValue);
  } else {
    Serial.printf("[boot] WHO_AM_I at 0x%02X did not answer: %s\n", KARTGPS_MPU_ADDRESS,
                  diag::wireErrorName(bootReport.probeError));
  }
  Serial.printf("[boot] MPU-6050 %s\n", bootReport.imuPresent ? "ready" : "NOT FOUND");
  Serial.printf("[boot] GPS configured for %d Hz at %lu baud on RX=GPIO%d TX=GPIO%d\n",
                KARTGPS_GPS_RATE_HZ, static_cast<unsigned long>(KARTGPS_GPS_RUN_BAUD),
                KARTGPS_PIN_GPS_RX, KARTGPS_PIN_GPS_TX);
  Serial.printf("[boot] BLE advertising as %s\n", KARTGPS_BLE_DEVICE_NAME);
}

void printStatus(uint32_t nowMs) {
  const LedPattern pattern = currentPattern(nowMs);
  Serial.printf("\n--- status at %lu ms ---\n", static_cast<unsigned long>(nowMs));
  Serial.printf("[led ] %s\n", ledPatternName(pattern));
  if (pattern == LedPattern::kFault) {
    // The two-flash pattern has exactly two causes and they need different
    // fixes, so name the one that is actually firing.
    if (!imuPresent) {
      Serial.println("[led ] cause: the MPU-6050 did not answer on I2C. Press 'i' for a scan.");
    } else {
      Serial.println("[led ] cause: the last zeroing was rejected; the sensor itself "
                     "is fine. Hold the board still and press 'z'.");
    }
  }

  Serial.printf("[imu ] accel trim correction: %+d %+d %+d counts%s\n", rawAccelBias.count[0],
                rawAccelBias.count[1], rawAccelBias.count[2],
                rawAccelBias.isZero() ? " (none configured)" : "");
  Serial.printf("[imu ] present=%s i2cErrors=%lu calibrator=%s (%u/%u) biasApplied=%s\n",
                imuPresent ? "yes" : "NO", static_cast<unsigned long>(mpu.errorCount()),
                calibratorStateName(), static_cast<unsigned>(calibrator.collected()),
                static_cast<unsigned>(calibrator.required()),
                motionProcessor.isCalibrated() ? "yes" : "no");

  const nmea::SentenceAssembler::Stats& sentences = gpsReceiver.sentenceStats();
  const uint32_t lastByteMs = gpsReceiver.lastByteMs();
  Serial.printf("[gps ] rxBytes=%lu sentences=%lu checksumErr=%lu overflow=%lu fixes=%lu\n",
                static_cast<unsigned long>(gpsReceiver.rxByteCount()),
                static_cast<unsigned long>(sentences.accepted),
                static_cast<unsigned long>(sentences.checksumErrors),
                static_cast<unsigned long>(sentences.overflows),
                static_cast<unsigned long>(gpsReceiver.fixCount()));
  if (lastByteMs == 0) {
    Serial.printf("[gps ] not one byte has arrived since boot. That is a wiring or "
                  "power fault, not a parsing one: check that the module TX reaches "
                  "GPIO%d and that the module has 3V3.\n",
                  KARTGPS_PIN_GPS_RX);
  } else {
    Serial.printf("[gps ] last byte %lu ms ago\n", static_cast<unsigned long>(nowMs - lastByteMs));
    if (sentences.accepted == 0) {
      Serial.println("[gps ] bytes arrive but no sentence validates. Press 'n' to watch "
                     "the raw stream: hex rather than text means the baud rate is wrong.");
    }
  }

  const telemetry::GnssFix& partial = gpsReceiver.partialFix();
  Serial.printf("[gps ] partial fix: quality=%u satellites=%u position=%s\n",
                static_cast<unsigned>(partial.fixQuality),
                static_cast<unsigned>(partial.satellites),
                partial.positionValid ? "valid" : "none");

  const kart::TelemetryPipeline::Stats& stats = pipeline.stats();
  Serial.printf("[ble ] connected=%s sent gps=%lu motion=%lu status=%lu\n",
                bleServer.isConnected() ? "yes" : "no",
                static_cast<unsigned long>(stats.gpsMainSent),
                static_cast<unsigned long>(stats.motionSent),
                static_cast<unsigned long>(stats.statusSent));
  Serial.printf("[ble ] dropped: disconnected=%lu byFilter=%lu byRate=%lu transportErr=%lu\n",
                static_cast<unsigned long>(stats.droppedDisconnected),
                static_cast<unsigned long>(stats.droppedByFilter),
                static_cast<unsigned long>(stats.droppedByRate),
                static_cast<unsigned long>(stats.transportErrors));
  if (!bleServer.isConnected()) {
    Serial.println("[ble ] the three sent counters only move while a phone is "
                   "connected, so zeros here are expected on the bench.");
  }
  Serial.printf("[sys ] heap=%lu nmeaEcho=%s\n", static_cast<unsigned long>(ESP.getFreeHeap()),
                gpsReceiver.isEchoing() ? "on" : "off");
}

void runI2cDiagnostics() {
  Serial.println();
  diag::reportBusLines(KARTGPS_PIN_I2C_SDA, KARTGPS_PIN_I2C_SCL, Serial);
  const diag::I2cScanResult scan = diag::scanI2c(Wire, Serial);
  for (uint8_t i = 0; i < scan.foundCount; ++i) {
    if (scan.found[i] == imu::kI2cAddressLow || scan.found[i] == imu::kI2cAddressHigh) {
      diag::dumpMpuRegisters(Wire, scan.found[i], Serial, rawAccelBias);
    }
  }
  if (scan.foundCount == 0) {
    // Dump anyway: the per-register error codes from a silent address say more
    // than the scan summary does.
    diag::dumpMpuRegisters(Wire, KARTGPS_MPU_ADDRESS, Serial, rawAccelBias);
  }
}

void retryImu(uint32_t nowMs) {
  Serial.println("[imu ] re-probing");
  imuPresent = mpu.begin(makeImuConfig());
  bootReport.imuPresent = imuPresent;
  Serial.printf("[imu ] %s\n", imuPresent ? "MPU-6050 ready" : "still not answering");
  if (imuPresent) {
    pipeline.resetSession(nowMs);
    startCalibration();
  }
}

void printCharacterization() {
  const float nominal = imu::accelScaleLsbPerG(mpu.config().accelRange);
  Serial.printf("\n--- accelerometer characterisation, %lu samples ---\n",
                static_cast<unsigned long>(characterizer.acceptedSamples()));
  Serial.printf("[cal ] nominal sensitivity for this range: %.0f counts/g\n", nominal);

  static const char* kAxisNames[3] = {"X", "Y", "Z"};
  for (size_t i = 0; i < 3; ++i) {
    const imu::AccelAxisFit fit = characterizer.axis(i);
    // Counts alone do not say what to do next. Stating the extremes in g on the
    // nominal scale does: an axis that never went near -1 g or +1 g has a face
    // that was never rested on.
    Serial.printf("[cal ] %s: reached %+.2f g .. %+.2f g%s\n", kAxisNames[i],
                  static_cast<float>(fit.minCount) / nominal,
                  static_cast<float>(fit.maxCount) / nominal,
                  fit.complete ? "" : "   <- needs both faces of this axis");
    if (!fit.complete) {
      continue;
    }
    Serial.printf("[cal ]    offset %+.0f counts (%+.3f g)   sensitivity %.0f counts/g "
                  "(%.0f%% of nominal)\n",
                  fit.offsetCount, fit.offsetCount / nominal, fit.sensitivityLsbPerG,
                  100.0f * fit.sensitivityLsbPerG / nominal);
  }

  if (!characterizer.isComplete()) {
    Serial.println("[cal ] rest the board on each of its six faces, a couple of "
                   "seconds each, then press 'c' again.");
    return;
  }
  Serial.println("[cal ] all six faces seen.");
  Serial.println("[cal ] Sensitivities near 100% mean the scale is right and the part is "
                 "usable; what is left is trim error. Add these to platformio.ini and "
                 "reflash, then press 'z' to zero:");
  Serial.printf("[cal ]     -DKARTGPS_ACCEL_OFFSET_X=%ld\n",
                static_cast<long>(characterizer.axis(0).offsetCount) + KARTGPS_ACCEL_OFFSET_X);
  Serial.printf("[cal ]     -DKARTGPS_ACCEL_OFFSET_Y=%ld\n",
                static_cast<long>(characterizer.axis(1).offsetCount) + KARTGPS_ACCEL_OFFSET_Y);
  Serial.printf("[cal ]     -DKARTGPS_ACCEL_OFFSET_Z=%ld\n",
                static_cast<long>(characterizer.axis(2).offsetCount) + KARTGPS_ACCEL_OFFSET_Z);
  Serial.println("[cal ] A sensitivity far from 100% on any axis is a different fault: "
                 "the part does not match the datasheet scale and no offset fixes that.");
}

void printConsoleHelp() {
  Serial.println("\n--- kart-gps console ---");
  Serial.println("  s  status: what the LED means right now, plus every counter");
  Serial.println("  b  replay the boot report (lost when the monitor attaches late)");
  Serial.println("  i  I2C bus lines, address scan and MPU-6050 register dump");
  Serial.println("  n  toggle the raw GPS UART echo");
  Serial.println("  c  six position accelerometer characterisation (start/report)");
  Serial.println("  z  re-run the zeroing (hold the board still)");
  Serial.println("  r  re-probe the MPU-6050 over I2C");
  Serial.println("  ?  this help");
}

void serviceCharacterizationProgress(uint32_t nowMs) {
  if (!characterizing || static_cast<uint32_t>(nowMs - lastCharacterizeLogMs) < 3000) {
    return;
  }
  lastCharacterizeLogMs = nowMs;
  const imu::AccelAxisFit x = characterizer.axis(0);
  const imu::AccelAxisFit y = characterizer.axis(1);
  const imu::AccelAxisFit z = characterizer.axis(2);
  const float nominal = imu::accelScaleLsbPerG(mpu.config().accelRange);
  Serial.printf("[cal ] X %+.2f..%+.2f%s  Y %+.2f..%+.2f%s  Z %+.2f..%+.2f%s (g)\n",
                x.minCount / nominal, x.maxCount / nominal, x.complete ? " ok" : "",
                y.minCount / nominal, y.maxCount / nominal, y.complete ? " ok" : "",
                z.minCount / nominal, z.maxCount / nominal, z.complete ? " ok" : "");
}

void serviceConsole(uint32_t nowMs) {
  while (Serial.available() > 0) {
    switch (Serial.read()) {
      case 's':
      case 'S':
        printStatus(nowMs);
        break;
      case 'b':
      case 'B':
        printBootReport();
        break;
      case 'i':
      case 'I':
        runI2cDiagnostics();
        break;
      case 'n':
      case 'N':
        if (gpsReceiver.isEchoing()) {
          gpsReceiver.setRawEcho(nullptr);
          Serial.println("\n[gps ] raw echo off");
        } else {
          Serial.println("\n[gps ] raw echo on: printable bytes as text, the rest as <XX>");
          gpsReceiver.setRawEcho(&Serial);
        }
        break;
      case 'c':
      case 'C':
        if (characterizing) {
          characterizing = false;
          printCharacterization();
        } else if (!imuPresent) {
          Serial.println("\n[cal ] no sensor to characterise");
        } else {
          characterizer.reset();
          characterizing = true;
          lastCharacterizeLogMs = nowMs;
          Serial.println("\n[cal ] capturing. Rest the board on each of its six faces "
                         "in turn, a couple of seconds each, then press 'c' to finish.");
        }
        break;
      case 'z':
      case 'Z':
        Serial.println("\n[imu ] zeroing, hold still");
        startCalibration();
        break;
      case 'r':
      case 'R':
        retryImu(nowMs);
        break;
      case '?':
      case 'h':
      case 'H':
        printConsoleHelp();
        break;
      default:
        break; // newlines and stray bytes from the terminal
    }
  }
}

#endif // KARTGPS_ENABLE_CONSOLE

#if KARTGPS_LOG_LEVEL > 1
void logHeartbeat(uint32_t nowMs) {
  static uint32_t lastLogMs = 0;
  if (static_cast<uint32_t>(nowMs - lastLogMs) < 5000) {
    return;
  }
  lastLogMs = nowMs;
  const kart::TelemetryPipeline::Stats& stats = pipeline.stats();
  // The counters that say *why* something is wrong come first. The three send
  // counters only move while a phone is connected, so on the bench they read
  // zero however healthy the device is, and leading with them misleads.
  KARTGPS_LOG("[stat] imu=%s cal=%s rxBytes=%lu nmea=%lu nmeaErr=%lu fixes=%lu ble=%s "
              "gps=%lu motion=%lu status=%lu heap=%lu",
              imuPresent ? "ok" : "MISSING", calibratorStateName(),
              static_cast<unsigned long>(gpsReceiver.rxByteCount()),
              static_cast<unsigned long>(gpsReceiver.sentenceStats().accepted),
              static_cast<unsigned long>(gpsReceiver.sentenceStats().checksumErrors),
              static_cast<unsigned long>(gpsReceiver.fixCount()),
              bleServer.isConnected() ? "up" : "down",
              static_cast<unsigned long>(stats.gpsMainSent),
              static_cast<unsigned long>(stats.motionSent),
              static_cast<unsigned long>(stats.statusSent),
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

  // Probe WHO_AM_I before the driver does, so the console can say later
  // whether the sensor was silent or answered with the wrong identity.
  bootReport.whoAmIRead = diag::readRegisters(Wire, KARTGPS_MPU_ADDRESS, imu::reg::kWhoAmI,
                                              &bootReport.whoAmI, 1, bootReport.probeError);
  imuPresent = mpu.begin(makeImuConfig());
  bootReport.imuPresent = imuPresent;
  KARTGPS_LOG("[imu] %s", imuPresent ? "sensor ready" : "sensor NOT FOUND, check wiring");
  if (imuPresent && mpu.whoAmI() != imu::kWhoAmIValue) {
    // Worth saying out loud: the part is compatible enough to drive, but it is
    // not the one the temperature curve and the accel filter were tuned for.
    KARTGPS_LOG("[imu] WHO_AM_I is 0x%02X, not the MPU-6050's 0x%02X: this is a "
                "substituted part, see docs/hardware.md",
                mpu.whoAmI(), imu::kWhoAmIValue);
  }

  hal::GpsReceiver::Config gpsConfig;
  gpsConfig.rxPin = KARTGPS_PIN_GPS_RX;
  gpsConfig.txPin = KARTGPS_PIN_GPS_TX;
  gpsConfig.bootBaud = KARTGPS_GPS_BOOT_BAUD;
  gpsConfig.runBaud = KARTGPS_GPS_RUN_BAUD;
  gpsConfig.rateHz = KARTGPS_GPS_RATE_HZ;
  gpsReceiver.begin(gpsConfig);
#if KARTGPS_ENABLE_NMEA_ECHO
  gpsReceiver.setRawEcho(&Serial);
#endif
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

#if KARTGPS_ENABLE_CONSOLE
  KARTGPS_LOG("[boot] press ? on the serial console for diagnostics");
#endif
}

void loop() {
  const uint32_t now = millis();

  bleServer.drainEvents(pipeline);
  serviceGps(now);
  serviceImu(now);
  serviceButton(now);
  pipeline.tick(now);
  updateStatusLed(currentPattern(now), now);
#if KARTGPS_ENABLE_CONSOLE
  serviceCharacterizationProgress(now);
  serviceConsole(now);
#endif
#if KARTGPS_LOG_LEVEL > 1
  logHeartbeat(now);
#endif
}
