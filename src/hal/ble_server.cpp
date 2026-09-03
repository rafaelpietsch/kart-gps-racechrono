// SPDX-License-Identifier: MIT

#include "ble_server.h"

#include <string.h>

namespace hal {
namespace {

BleTelemetryServer* g_server = nullptr;

class ServerCallbacks : public NimBLEServerCallbacks {
public:
  explicit ServerCallbacks(const BleTelemetryServer::Config& config) : config_(config) {}

  void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
    // RaceChrono does not renegotiate the interval itself, so ask for one fast
    // enough to carry 25 Hz of motion data as soon as the link is up.
    server->updateConnParams(connInfo.getConnHandle(), config_.minConnectionInterval,
                             config_.maxConnectionInterval, 0, config_.supervisionTimeout);
    if (g_server != nullptr) {
      g_server->onConnectionChanged(true, connInfo.getConnHandle());
    }
  }

  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
    (void)connInfo;
    (void)reason;
    if (g_server != nullptr) {
      g_server->onConnectionChanged(false, 0);
    }
    // Without this the device disappears after the first session.
    server->startAdvertising();
  }

private:
  BleTelemetryServer::Config config_;
};

class FilterCallbacks : public NimBLECharacteristicCallbacks {
public:
  void onWrite(NimBLECharacteristic* characteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;
    const NimBLEAttValue value = characteristic->getValue();
    if (g_server != nullptr) {
      g_server->enqueueFilterWrite(value.data(), value.length());
    }
  }
};

} // namespace

void BleTelemetryServer::begin(const Config& config) {
  config_ = config;
  g_server = this;

  NimBLEDevice::init(config.deviceName);
  NimBLEDevice::setPower(config.txPowerDbm);

  server_ = NimBLEDevice::createServer();
  server_->setCallbacks(new ServerCallbacks(config));
  // RaceChrono expects to find the device again after a reconnect rather than
  // the device going quiet once the first phone drops.
  server_->advertiseOnDisconnect(true);

  NimBLEService* service = server_->createService(NimBLEUUID(rc::kServiceUuid));

  canMain_ = service->createCharacteristic(NimBLEUUID(rc::kCharCanMainUuid),
                                           NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  canFilter_ =
      service->createCharacteristic(NimBLEUUID(rc::kCharCanFilterUuid), NIMBLE_PROPERTY::WRITE);
  canFilter_->setCallbacks(new FilterCallbacks());
  gpsMain_ = service->createCharacteristic(NimBLEUUID(rc::kCharGpsMainUuid),
                                           NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
  gpsTime_ = service->createCharacteristic(NimBLEUUID(rc::kCharGpsTimeUuid),
                                           NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  service->start();

  NimBLEAdvertising* advertising = NimBLEDevice::getAdvertising();
  advertising->setName(config.deviceName);
  advertising->addServiceUUID(service->getUUID());
  advertising->enableScanResponse(true);
  advertising->start();
}

void BleTelemetryServer::onConnectionChanged(bool connected, uint16_t connectionHandle) {
  connected_ = connected;
  connectionHandle_ = connectionHandle;
  pendingEvent_ = connected ? Event::kConnected : Event::kDisconnected;
}

void BleTelemetryServer::enqueueFilterWrite(const uint8_t* data, size_t length) {
  if (data == nullptr || length == 0 || length > kFilterWriteMaxLength) {
    ++droppedFilterWrites_;
    return;
  }
  portENTER_CRITICAL(&lock_);
  if (filterCount_ >= kFilterQueueDepth) {
    ++droppedFilterWrites_;
  } else {
    const size_t slot = (filterHead_ + filterCount_) % kFilterQueueDepth;
    memcpy(filterQueue_[slot].data, data, length);
    filterQueue_[slot].length = static_cast<uint8_t>(length);
    ++filterCount_;
  }
  portEXIT_CRITICAL(&lock_);
}

void BleTelemetryServer::drainEvents(kart::TelemetryPipeline& pipeline) {
  const Event event = pendingEvent_;
  if (event != Event::kNone) {
    pendingEvent_ = Event::kNone;
    if (event == Event::kConnected) {
      pipeline.onConnect();
    } else {
      pipeline.onDisconnect();
    }
  }

  // Copy each queued write out under the lock, then apply it outside: the
  // pipeline call is long enough that holding a spinlock across it would risk
  // stalling the NimBLE task.
  for (;;) {
    FilterWrite write;
    portENTER_CRITICAL(&lock_);
    const bool hasWrite = filterCount_ > 0;
    if (hasWrite) {
      write = filterQueue_[filterHead_];
      filterHead_ = (filterHead_ + 1) % kFilterQueueDepth;
      --filterCount_;
    }
    portEXIT_CRITICAL(&lock_);

    if (!hasWrite) {
      break;
    }
    pipeline.onCanFilterWrite(write.data, write.length);
  }
}

bool BleTelemetryServer::isConnected() const {
  return connected_;
}

bool BleTelemetryServer::notify(NimBLECharacteristic* characteristic, const uint8_t* data,
                                size_t length) {
  if (characteristic == nullptr || !connected_) {
    return false;
  }
  characteristic->setValue(data, length);
  return characteristic->notify();
}

bool BleTelemetryServer::notifyGpsMain(const uint8_t* data, size_t length) {
  return notify(gpsMain_, data, length);
}

bool BleTelemetryServer::notifyGpsTime(const uint8_t* data, size_t length) {
  return notify(gpsTime_, data, length);
}

bool BleTelemetryServer::notifyCan(const uint8_t* data, size_t length) {
  return notify(canMain_, data, length);
}

} // namespace hal
