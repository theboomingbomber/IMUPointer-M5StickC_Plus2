#include "BleMouse.h"

#include <Arduino.h>
#include "HIDTypes.h"

namespace {
constexpr uint16_t kConnMinInterval = 0x10;  // 20 ms
constexpr uint16_t kConnMaxInterval = 0x20;  // 40 ms
constexpr uint16_t kConnLatency = 0;
constexpr uint16_t kConnTimeout = 400;       // 4 seconds
constexpr uint16_t kPairingDisconnectWaitMs = 1000;

static const uint8_t kHidReportDescriptor[] = {
  USAGE_PAGE(1),       0x01,
  USAGE(1),            0x02,
  COLLECTION(1),       0x01,
  USAGE(1),            0x01,
  COLLECTION(1),       0x00,
  USAGE_PAGE(1),       0x09,
  USAGE_MINIMUM(1),    0x01,
  USAGE_MAXIMUM(1),    0x05,
  LOGICAL_MINIMUM(1),  0x00,
  LOGICAL_MAXIMUM(1),  0x01,
  REPORT_SIZE(1),      0x01,
  REPORT_COUNT(1),     0x05,
  HIDINPUT(1),         0x02,
  REPORT_SIZE(1),      0x03,
  REPORT_COUNT(1),     0x01,
  HIDINPUT(1),         0x03,
  USAGE_PAGE(1),       0x01,
  USAGE(1),            0x30,
  USAGE(1),            0x31,
  USAGE(1),            0x38,
  LOGICAL_MINIMUM(1),  0x81,
  LOGICAL_MAXIMUM(1),  0x7f,
  REPORT_SIZE(1),      0x08,
  REPORT_COUNT(1),     0x03,
  HIDINPUT(1),         0x06,
  USAGE_PAGE(1),       0x0c,
  USAGE(2),      0x38, 0x02,
  LOGICAL_MINIMUM(1),  0x81,
  LOGICAL_MAXIMUM(1),  0x7f,
  REPORT_SIZE(1),      0x08,
  REPORT_COUNT(1),     0x01,
  HIDINPUT(1),         0x06,
  END_COLLECTION(0),
  END_COLLECTION(0)
};
}  // namespace

class BleMouse::ServerCallbacks : public NimBLEServerCallbacks {
 public:
  explicit ServerCallbacks(BleMouse* owner) : owner_(owner) {}

  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    owner_->connected = true;
    pServer->updateConnParams(connInfo.getConnHandle(),
                              kConnMinInterval,
                              kConnMaxInterval,
                              kConnLatency,
                              kConnTimeout);
  }

  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    (void)connInfo;
    (void)reason;
    owner_->connected = false;
    if (owner_->advertising != nullptr && !owner_->advertising->isAdvertising()) {
      owner_->advertising->start();
    } else {
      pServer->startAdvertising();
    }
  }

 private:
  BleMouse* owner_;
};

BleMouse::BleMouse(std::string deviceName, std::string deviceManufacturer, uint8_t batteryLevel)
    : _buttons(0),
      hid(nullptr),
      inputMouse(nullptr),
      server(nullptr),
      advertising(nullptr),
      connected(false),
      batteryLevel(batteryLevel),
      deviceManufacturer(deviceManufacturer),
      deviceName(deviceName),
      callbacks(nullptr) {}

void BleMouse::begin(void) {
  if (!NimBLEDevice::isInitialized()) {
    NimBLEDevice::init(this->deviceName);
  } else {
    NimBLEDevice::setDeviceName(this->deviceName);
  }

  NimBLEDevice::setSecurityAuth(true, false, false);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

  this->server = NimBLEDevice::createServer();
  if (this->callbacks == nullptr) {
    this->callbacks = new ServerCallbacks(this);
  }
  this->server->setCallbacks(this->callbacks, false);
  this->server->advertiseOnDisconnect(true);

  this->hid = new NimBLEHIDDevice(this->server);
  this->inputMouse = this->hid->getInputReport(0);

  this->hid->setManufacturer(this->deviceManufacturer);
  this->hid->setPnp(0x02, 0xe502, 0xa111, 0x0210);
  this->hid->setHidInfo(0x00, 0x02);
  this->hid->setReportMap((uint8_t*)kHidReportDescriptor, sizeof(kHidReportDescriptor));
  this->hid->startServices();
  this->hid->setBatteryLevel(this->batteryLevel);

  this->configureAdvertising();
  this->onStarted(this->server);
  this->advertising->start();
}

void BleMouse::end(void) {}

void BleMouse::configureAdvertising() {
  this->advertising = this->server->getAdvertising();
  this->advertising->stop();
  this->advertising->removeServices();
  this->advertising->setAppearance(HID_MOUSE);
  this->advertising->addServiceUUID(this->hid->getHidService()->getUUID());
  this->advertising->setName(this->deviceName);
  this->advertising->enableScanResponse(true);
  this->advertising->setPreferredParams(0x06, 0x12);
}

void BleMouse::click(uint8_t b) {
  _buttons = b;
  move(0, 0, 0, 0);
  _buttons = 0;
  move(0, 0, 0, 0);
}

void BleMouse::move(signed char x, signed char y, signed char wheel, signed char hWheel) {
  if (this->isConnected() && this->inputMouse != nullptr) {
    uint8_t m[5];
    m[0] = _buttons;
    m[1] = static_cast<uint8_t>(x);
    m[2] = static_cast<uint8_t>(y);
    m[3] = static_cast<uint8_t>(wheel);
    m[4] = static_cast<uint8_t>(hWheel);
    this->inputMouse->setValue(m, sizeof(m));
    this->inputMouse->notify();
  }
}

void BleMouse::buttons(uint8_t b) {
  if (b != _buttons) {
    _buttons = b;
    move(0, 0, 0, 0);
  }
}

void BleMouse::press(uint8_t b) {
  buttons(_buttons | b);
}

void BleMouse::release(uint8_t b) {
  buttons(_buttons & ~b);
}

bool BleMouse::isPressed(uint8_t b) {
  return (b & _buttons) > 0;
}

bool BleMouse::isConnected(void) {
  return this->connected;
}

bool BleMouse::startPairingMode(void) {
  if (this->server == nullptr || this->advertising == nullptr) {
    return false;
  }

  auto peers = this->server->getPeerDevices();
  for (size_t i = 0; i < peers.size(); ++i) {
    this->server->disconnect(peers[i]);
  }

  const uint32_t waitStart = millis();
  while (this->server->getConnectedCount() > 0 && millis() - waitStart < kPairingDisconnectWaitMs) {
    delay(10);
  }
  this->connected = (this->server->getConnectedCount() > 0);

  this->advertising->stop();
  NimBLEDevice::deleteAllBonds();
  this->configureAdvertising();
  return this->advertising->start();
}

void BleMouse::setBatteryLevel(uint8_t level) {
  this->batteryLevel = level;
  if (this->hid != nullptr) {
    this->hid->setBatteryLevel(this->batteryLevel);
  }
}
