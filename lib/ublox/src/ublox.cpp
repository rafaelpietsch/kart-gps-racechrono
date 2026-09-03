// SPDX-License-Identifier: MIT

#include "ublox.h"

#include <string.h>

namespace ublox {
namespace {

void writeLittleEndian16(uint8_t* out, uint16_t value) {
  out[0] = static_cast<uint8_t>(value);
  out[1] = static_cast<uint8_t>(value >> 8);
}

void writeLittleEndian32(uint8_t* out, uint32_t value) {
  out[0] = static_cast<uint8_t>(value);
  out[1] = static_cast<uint8_t>(value >> 8);
  out[2] = static_cast<uint8_t>(value >> 16);
  out[3] = static_cast<uint8_t>(value >> 24);
}

/// CFG-PRT mode word: 8 data bits, no parity, 1 stop bit.
constexpr uint32_t kUartMode8N1 = 0x000008D0;
/// Accept and emit both UBX and NMEA.
constexpr uint16_t kProtocolMaskUbxAndNmea = 0x0003;

} // namespace

Checksum computeChecksum(const uint8_t* data, size_t length) {
  Checksum checksum;
  for (size_t i = 0; i < length; ++i) {
    checksum.a = static_cast<uint8_t>(checksum.a + data[i]);
    checksum.b = static_cast<uint8_t>(checksum.b + checksum.a);
  }
  return checksum;
}

size_t buildFrame(uint8_t messageClass, uint8_t messageId, const uint8_t* payload,
                  size_t payloadLength, uint8_t* out, size_t outCapacity) {
  if (out == nullptr || payloadLength > 0xFFFF) {
    return 0;
  }
  if (payloadLength > 0 && payload == nullptr) {
    return 0;
  }
  const size_t total = kFrameOverhead + payloadLength;
  if (outCapacity < total) {
    return 0;
  }

  out[0] = kSync1;
  out[1] = kSync2;
  out[2] = messageClass;
  out[3] = messageId;
  writeLittleEndian16(&out[4], static_cast<uint16_t>(payloadLength));
  if (payloadLength > 0) {
    memcpy(&out[6], payload, payloadLength);
  }

  // The checksum covers class, id, length and payload -- everything but the
  // two sync bytes and the checksum itself.
  const Checksum checksum = computeChecksum(&out[2], payloadLength + 4);
  out[6 + payloadLength] = checksum.a;
  out[7 + payloadLength] = checksum.b;
  return total;
}

bool validateFrame(const uint8_t* frame, size_t length) {
  if (frame == nullptr || length < kFrameOverhead) {
    return false;
  }
  if (frame[0] != kSync1 || frame[1] != kSync2) {
    return false;
  }
  const size_t payloadLength = static_cast<size_t>(frame[4]) | (static_cast<size_t>(frame[5]) << 8);
  if (length != kFrameOverhead + payloadLength) {
    return false;
  }
  const Checksum checksum = computeChecksum(&frame[2], payloadLength + 4);
  return frame[6 + payloadLength] == checksum.a && frame[7 + payloadLength] == checksum.b;
}

size_t buildCfgRate(uint16_t measurementPeriodMs, uint16_t navigationRate, uint16_t timeReference,
                    uint8_t* out, size_t outCapacity) {
  uint8_t payload[6];
  writeLittleEndian16(&payload[0], measurementPeriodMs);
  writeLittleEndian16(&payload[2], navigationRate);
  writeLittleEndian16(&payload[4], timeReference);
  return buildFrame(kClassCfg, kIdCfgRate, payload, sizeof(payload), out, outCapacity);
}

size_t buildCfgRateHz(uint16_t hertz, uint8_t* out, size_t outCapacity) {
  if (hertz == 0) {
    return 0;
  }
  const uint16_t periodMs = static_cast<uint16_t>(1000u / hertz);
  // timeReference 1 = align the solution to GPS time.
  return buildCfgRate(periodMs, 1, 1, out, outCapacity);
}

size_t buildCfgMsg(uint8_t messageClass, uint8_t messageId, const uint8_t rates[kPortCount],
                   uint8_t* out, size_t outCapacity) {
  if (rates == nullptr) {
    return 0;
  }
  uint8_t payload[2 + kPortCount];
  payload[0] = messageClass;
  payload[1] = messageId;
  memcpy(&payload[2], rates, kPortCount);
  return buildFrame(kClassCfg, kIdCfgMsg, payload, sizeof(payload), out, outCapacity);
}

size_t buildCfgMsgForPort(uint8_t messageClass, uint8_t messageId, uint8_t port, uint8_t rate,
                          uint8_t* out, size_t outCapacity) {
  if (port >= kPortCount) {
    return 0;
  }
  uint8_t rates[kPortCount] = {0, 0, 0, 0, 0, 0};
  rates[port] = rate;
  return buildCfgMsg(messageClass, messageId, rates, out, outCapacity);
}

size_t buildCfgPrtUart1(uint32_t baudRate, uint8_t* out, size_t outCapacity) {
  uint8_t payload[20];
  memset(payload, 0, sizeof(payload));
  payload[0] = kPortUart1;
  // payload[1] reserved, payload[2..3] txReady: both left at zero.
  writeLittleEndian32(&payload[4], kUartMode8N1);
  writeLittleEndian32(&payload[8], baudRate);
  writeLittleEndian16(&payload[12], kProtocolMaskUbxAndNmea); // inProtoMask
  writeLittleEndian16(&payload[14], kProtocolMaskUbxAndNmea); // outProtoMask
  // payload[16..19]: flags and reserved, left at zero.
  return buildFrame(kClassCfg, kIdCfgPrt, payload, sizeof(payload), out, outCapacity);
}

size_t buildCfgNav5DynamicModel(DynamicModel model, uint8_t* out, size_t outCapacity) {
  uint8_t payload[36];
  memset(payload, 0, sizeof(payload));
  // mask bit 0 selects "apply dynamic model"; every other setting is left
  // untouched so we do not clobber the receiver's defaults.
  writeLittleEndian16(&payload[0], 0x0001);
  payload[2] = static_cast<uint8_t>(model);
  return buildFrame(kClassCfg, kIdCfgNav5, payload, sizeof(payload), out, outCapacity);
}

size_t buildCfgRstHotStart(uint8_t* out, size_t outCapacity) {
  uint8_t payload[4];
  writeLittleEndian16(&payload[0], 0x0000); // navBbrMask: keep all backed-up data
  payload[2] = 0x02;                        // controlled software reset, GNSS only
  payload[3] = 0x00;                        // reserved
  return buildFrame(kClassCfg, kIdCfgRst, payload, sizeof(payload), out, outCapacity);
}

} // namespace ublox
