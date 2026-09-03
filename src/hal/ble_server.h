// SPDX-License-Identifier: MIT
//
// NimBLE implementation of the RaceChrono DIY device service.
//
// NimBLE runs its callbacks on its own FreeRTOS task, so nothing here touches
// the pipeline directly. Connection events and filter writes are queued and
// drained from the main loop, which keeps the pipeline single threaded and
// free of locks on the hot path.

#ifndef KARTGPS_HAL_BLE_SERVER_H
#define KARTGPS_HAL_BLE_SERVER_H

#include <Arduino.h>
#include <NimBLEDevice.h>

#include "pipeline.h"
#include "rc_protocol.h"

namespace hal {

class BleTelemetryServer : public kart::PacketSink {
public:
  struct Config {
    const char* deviceName = "KartGPS";
    uint16_t minConnectionInterval = 12; ///< units of 1.25 ms
    uint16_t maxConnectionInterval = 24;
    uint16_t supervisionTimeout = 400; ///< units of 10 ms
    int8_t txPowerDbm = 3;
  };

  /// Events raised on the NimBLE task and consumed by the main loop.
  enum class Event : uint8_t { kNone, kConnected, kDisconnected };

  void begin(const Config& config);

  /// Applies queued connection events and filter writes to `pipeline`.
  /// Call once per loop iteration.
  void drainEvents(kart::TelemetryPipeline& pipeline);

  // --- kart::PacketSink -----------------------------------------------------

  bool isConnected() const override;
  bool notifyGpsMain(const uint8_t* data, size_t length) override;
  bool notifyGpsTime(const uint8_t* data, size_t length) override;
  bool notifyCan(const uint8_t* data, size_t length) override;

  // --- Called from the NimBLE task ------------------------------------------

  void onConnectionChanged(bool connected, uint16_t connectionHandle);
  void enqueueFilterWrite(const uint8_t* data, size_t length);

  uint32_t droppedFilterWrites() const { return droppedFilterWrites_; }

private:
  static constexpr size_t kFilterQueueDepth = 8;
  static constexpr size_t kFilterWriteMaxLength = 8;

  struct FilterWrite {
    uint8_t data[kFilterWriteMaxLength] = {0};
    uint8_t length = 0;
  };

  bool notify(NimBLECharacteristic* characteristic, const uint8_t* data, size_t length);

  NimBLEServer* server_ = nullptr;
  NimBLECharacteristic* canMain_ = nullptr;
  NimBLECharacteristic* canFilter_ = nullptr;
  NimBLECharacteristic* gpsMain_ = nullptr;
  NimBLECharacteristic* gpsTime_ = nullptr;
  Config config_;

  portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
  FilterWrite filterQueue_[kFilterQueueDepth];
  size_t filterHead_ = 0;
  size_t filterCount_ = 0;
  volatile bool connected_ = false;
  volatile uint16_t connectionHandle_ = 0;
  volatile Event pendingEvent_ = Event::kNone;
  volatile uint32_t droppedFilterWrites_ = 0;
};

} // namespace hal

#endif // KARTGPS_HAL_BLE_SERVER_H
