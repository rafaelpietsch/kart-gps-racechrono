// SPDX-License-Identifier: MIT
//
// UBX protocol frame builder for u-blox 6 receivers (NEO-6M).
//
// The NEO-6M powers up at 9600 baud, 1 Hz, with a chatty default NMEA message
// set. Reaching a usable rate for lap timing needs three things in order:
//   1. raise the port baud rate, because 5 Hz of RMC+GGA+GSA does not fit in
//      9600 baud (see docs/hardware.md for the arithmetic),
//   2. turn off the sentences we do not consume,
//   3. raise the navigation rate and switch the dynamic model to automotive.
//
// Frames are built rather than hardcoded so the checksum can never drift out of
// sync with the payload; the tests pin the output against the byte sequences
// published by the bonogps project.

#ifndef KARTGPS_UBLOX_H
#define KARTGPS_UBLOX_H

#include <stddef.h>
#include <stdint.h>

namespace ublox {

constexpr uint8_t kSync1 = 0xB5;
constexpr uint8_t kSync2 = 0x62;

/// Header (2 sync + class + id + length) plus the two checksum bytes.
constexpr size_t kFrameOverhead = 8;

// --- Message classes and identifiers ---------------------------------------

constexpr uint8_t kClassNav = 0x01;
constexpr uint8_t kClassCfg = 0x06;

constexpr uint8_t kIdCfgPrt = 0x00;
constexpr uint8_t kIdCfgMsg = 0x01;
constexpr uint8_t kIdCfgRst = 0x04;
constexpr uint8_t kIdCfgRate = 0x08;
constexpr uint8_t kIdCfgNav5 = 0x24;

/// NMEA standard messages live in class 0xF0.
constexpr uint8_t kClassNmea = 0xF0;
constexpr uint8_t kIdNmeaGga = 0x00;
constexpr uint8_t kIdNmeaGll = 0x01;
constexpr uint8_t kIdNmeaGsa = 0x02;
constexpr uint8_t kIdNmeaGsv = 0x03;
constexpr uint8_t kIdNmeaRmc = 0x04;
constexpr uint8_t kIdNmeaVtg = 0x05;
constexpr uint8_t kIdNmeaGbs = 0x09;

/// CFG-NAV5 dynamic platform models.
enum class DynamicModel : uint8_t {
  kPortable = 0,
  kStationary = 2,
  kPedestrian = 3,
  kAutomotive = 4,
  kSea = 5,
  kAirborne1G = 6,
};

/// The receiver has one rate slot per I/O port; UART1 is index 1.
constexpr size_t kPortCount = 6;
constexpr uint8_t kPortUart1 = 1;

struct Checksum {
  uint8_t a = 0;
  uint8_t b = 0;
};

/// 8-bit Fletcher checksum over class, id, length and payload.
Checksum computeChecksum(const uint8_t* data, size_t length);

/// Writes a complete UBX frame. Returns the number of bytes written, or 0 when
/// the output buffer is too small.
size_t buildFrame(uint8_t messageClass, uint8_t messageId, const uint8_t* payload,
                  size_t payloadLength, uint8_t* out, size_t outCapacity);

/// Verifies sync bytes, declared length and checksum of a complete frame.
bool validateFrame(const uint8_t* frame, size_t length);

// --- Configuration messages -------------------------------------------------

/// CFG-RATE: navigation solution period in milliseconds.
/// The NEO-6M accepts down to 200 ms (5 Hz); see docs/hardware.md.
size_t buildCfgRate(uint16_t measurementPeriodMs, uint16_t navigationRate, uint16_t timeReference,
                    uint8_t* out, size_t outCapacity);

/// CFG-RATE at a given frequency, a convenience wrapper over buildCfgRate.
size_t buildCfgRateHz(uint16_t hertz, uint8_t* out, size_t outCapacity);

/// CFG-MSG, long form: one output rate per port.
size_t buildCfgMsg(uint8_t messageClass, uint8_t messageId, const uint8_t rates[kPortCount],
                   uint8_t* out, size_t outCapacity);

/// CFG-MSG for a single port, leaving the other ports silent.
size_t buildCfgMsgForPort(uint8_t messageClass, uint8_t messageId, uint8_t port, uint8_t rate,
                          uint8_t* out, size_t outCapacity);

/// CFG-PRT: UART1 at the given baud rate, 8N1, UBX+NMEA in and out.
size_t buildCfgPrtUart1(uint32_t baudRate, uint8_t* out, size_t outCapacity);

/// CFG-NAV5: switch the dynamic platform model.
size_t buildCfgNav5DynamicModel(DynamicModel model, uint8_t* out, size_t outCapacity);

/// CFG-RST: controlled software reset of the GNSS engine only, keeping the
/// battery-backed ephemeris so the receiver hot starts.
size_t buildCfgRstHotStart(uint8_t* out, size_t outCapacity);

} // namespace ublox

#endif // KARTGPS_UBLOX_H
