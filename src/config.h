// SPDX-License-Identifier: MIT
//
// Board wiring and tuning for the ESP32-C3 Super Mini.
//
// Every value can be overridden from platformio.ini with -D, so a different
// board or a different mounting does not need a source edit.

#ifndef KARTGPS_CONFIG_H
#define KARTGPS_CONFIG_H

// --- Pin map ----------------------------------------------------------------
//
// The Super Mini breaks out GPIO0-GPIO10 plus GPIO20/GPIO21. GPIO8 drives the
// on-board LED (active low) and GPIO9 is the BOOT strapping pin, so neither is
// used for anything that must be free at reset.

#ifndef KARTGPS_PIN_GPS_RX
#define KARTGPS_PIN_GPS_RX 20 ///< ESP32 RX, wired to the GPS module TX
#endif
#ifndef KARTGPS_PIN_GPS_TX
#define KARTGPS_PIN_GPS_TX 21 ///< ESP32 TX, wired to the GPS module RX
#endif

#ifndef KARTGPS_PIN_I2C_SDA
#define KARTGPS_PIN_I2C_SDA 5
#endif
#ifndef KARTGPS_PIN_I2C_SCL
#define KARTGPS_PIN_I2C_SCL 6
#endif

/// SET button, wired between the pin and ground; the internal pull-up holds it
/// high, so the pressed level is LOW.
#ifndef KARTGPS_PIN_SET_BUTTON
#define KARTGPS_PIN_SET_BUTTON 3
#endif

/// On-board LED. Active low on this board.
#ifndef KARTGPS_PIN_STATUS_LED
#define KARTGPS_PIN_STATUS_LED 8
#endif
#ifndef KARTGPS_LED_ACTIVE_LOW
#define KARTGPS_LED_ACTIVE_LOW 1
#endif

// --- GNSS -------------------------------------------------------------------

/// The NEO-6M boots at 9600 baud. We only stay there long enough to tell it to
/// change; see docs/hardware.md for why 9600 cannot carry 5 Hz.
#ifndef KARTGPS_GPS_BOOT_BAUD
#define KARTGPS_GPS_BOOT_BAUD 9600
#endif
#ifndef KARTGPS_GPS_RUN_BAUD
#define KARTGPS_GPS_RUN_BAUD 115200
#endif

/// Navigation rate in Hz. 5 Hz is the ceiling for the NEO-6M; the field is
/// configurable so the same firmware can drive a faster receiver.
#ifndef KARTGPS_GPS_RATE_HZ
#define KARTGPS_GPS_RATE_HZ 5
#endif

// --- IMU --------------------------------------------------------------------

/// I2C address of the MPU-6050. The GY-521 ties AD0 low, giving 0x68; some
/// breakouts tie it high instead, giving 0x69. The console 'i' command reports
/// which address actually answers.
#ifndef KARTGPS_MPU_ADDRESS
#define KARTGPS_MPU_ADDRESS 0x68
#endif

/// How often the MPU-6050 is read. Sampling above the publish rate lets the
/// low pass filter do its job and keeps aliasing off the g-g diagram.
#ifndef KARTGPS_IMU_SAMPLE_HZ
#define KARTGPS_IMU_SAMPLE_HZ 100
#endif

/// How often processed motion data goes out over BLE.
#ifndef KARTGPS_MOTION_PUBLISH_HZ
#define KARTGPS_MOTION_PUBLISH_HZ 25
#endif

/// Rotation from the device X axis to the direction of travel, in degrees.
/// Leave at 0 when the USB connector points backwards and the board is flat.
#ifndef KARTGPS_MOUNTING_YAW_DEGREES
#define KARTGPS_MOUNTING_YAW_DEGREES 0.0f
#endif

/// Samples averaged when the SET button zeroes the sensors. At 100 Hz this is
/// about 2.5 seconds of standing still.
#ifndef KARTGPS_CALIBRATION_SAMPLES
#define KARTGPS_CALIBRATION_SAMPLES 256
#endif

// --- Button -----------------------------------------------------------------

#ifndef KARTGPS_BUTTON_DEBOUNCE_MS
#define KARTGPS_BUTTON_DEBOUNCE_MS 30
#endif

/// Hold this long for the full reset (sensors, session and a GPS hot restart).
#ifndef KARTGPS_BUTTON_LONG_PRESS_MS
#define KARTGPS_BUTTON_LONG_PRESS_MS 3000
#endif

// --- BLE --------------------------------------------------------------------

#ifndef KARTGPS_BLE_DEVICE_NAME
#define KARTGPS_BLE_DEVICE_NAME "KartGPS"
#endif

/// Connection interval bounds in 1.25 ms units. 12 to 24 asks for 15 to 30 ms,
/// which leaves room for 25 Hz of motion data plus the GPS stream.
#ifndef KARTGPS_BLE_MIN_INTERVAL
#define KARTGPS_BLE_MIN_INTERVAL 12
#endif
#ifndef KARTGPS_BLE_MAX_INTERVAL
#define KARTGPS_BLE_MAX_INTERVAL 24
#endif
/// Supervision timeout in 10 ms units.
#ifndef KARTGPS_BLE_TIMEOUT
#define KARTGPS_BLE_TIMEOUT 400
#endif

/// Transmit power in dBm. The phone is usually a metre away in a pocket or on
/// the steering wheel, so maximum power is not needed and costs battery.
#ifndef KARTGPS_BLE_TX_POWER_DBM
#define KARTGPS_BLE_TX_POWER_DBM 3
#endif

// --- Logging ----------------------------------------------------------------

/// 0 silent, 1 errors and lifecycle, 2 verbose, 4 everything.
#ifndef KARTGPS_LOG_LEVEL
#define KARTGPS_LOG_LEVEL 1
#endif

// --- Diagnostics ------------------------------------------------------------

/// A one-key console on the USB serial port: bus scans, register dumps and a
/// raw UART echo, on demand. It costs a Serial.available() per loop, so it is
/// on wherever logging is, including the release build -- the boot log is the
/// one thing you cannot go back and ask for, and this is how you ask.
#ifndef KARTGPS_ENABLE_CONSOLE
#define KARTGPS_ENABLE_CONSOLE (KARTGPS_LOG_LEVEL > 0)
#endif

/// Echo raw GPS UART bytes to the console from boot. The console 'n' key
/// toggles it at runtime, so this only sets the starting state.
#ifndef KARTGPS_ENABLE_NMEA_ECHO
#define KARTGPS_ENABLE_NMEA_ECHO 0
#endif

#endif // KARTGPS_CONFIG_H
