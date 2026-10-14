// SPDX-License-Identifier: MIT
// Codex Micro BLE 协议层 —— 从 codex-micro-waveshare 移植。
//
// 相对上游的改动（step1）：
//   1. 删除 USB 麦克风（CODEX_STOPWATCH_USB_MIC）全部分支：Preferences bond 迁移、
//      bootloader RPC、usbPowerPresent 等，这些是 M5 Cardputer(C152) 专属。
//   2. 删除 CODEX_STOPWATCH_COMPACT_GATT 手写 GATT 表分支，本固件先走标准
//      BLEHIDDevice 路径；是否需要在 3.x 上手写精简表，由实测决定。
//   3. 新增 NimBLE 兼容：Arduino-ESP32 3.3.7-cn 默认 CONFIG_BT_NIMBLE_ENABLED，
//      Bluedroid 未启用，回调签名是 ble_gap_conn_desc* 而非 esp_ble_gatts_cb_param_t*。
//   4. ArduinoJson 7.x：StaticJsonDocument/DynamicJsonDocument 已合并为 JsonDocument。

#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <BLECharacteristic.h>
#include <BLEHIDDevice.h>
#include <BLEServer.h>
#include <freertos/queue.h>

#include <array>
#include <atomic>

#include "ConnectionHealth.h"

struct ThreadLight {
  uint32_t color = 0;
  float brightness = 0.0f;
  String effect = "off";
  float speed = 0.0f;
};

struct QuotaState {
  float remainingPercent = 0.0f;
  uint32_t resetInSeconds = 0;
  uint32_t receivedAtMs = 0;
  bool available = false;
};

struct CodexMicroState {
  std::array<ThreadLight, 6> threads;
  QuotaState quota;
  bool connected = false;
  bool hostRpcObserved = false;
  uint32_t lastHostRpcAtMs = 0;
  uint32_t connectionEpoch = 0;
  bool dirty = true;
};

class CodexMicroBle {
 public:
  static constexpr uint16_t kVendorId = 0x303A;
  static constexpr uint16_t kProductId = 0x8360;
  static constexpr uint8_t kReportId = 6;

  void begin();
  void poll();
  void setBattery(uint8_t percentage, bool charging);
  void sendKey(const char* key, uint8_t action, int8_t agent = -1);
  void sendJoystick(float angle, float distance);
  bool connected();
  CodexMicroState snapshot();

  // step1 验证用：0x1812 HID 服务是否真的拿到了 handle（为 0 = 注册失败）。
  uint16_t hidServiceHandle() const;
  const char* stackName() const;

 private:
  class ServerCallbacks;
  class OutputCallbacks;
  class InputReportCallbacks;
  class QuotaCallbacks;

  using PeerAddress = std::array<uint8_t, 6>;

  struct PendingOutputReport {
    uint8_t length = 0;
    uint16_t connectionId = 0;
    PeerAddress peerAddress = {};
    std::array<uint8_t, 64> data = {};
  };

  struct PendingConnectionEvent {
    uint16_t id = 0;
    bool connected = false;
  };

  struct PendingQuotaWrite {
    uint16_t length = 0;
    bool responseExpected = false;
    PeerAddress peerAddress = {};
    std::array<uint8_t, 512> data = {};
  };

  void onConnectionEvent(bool connected, uint16_t id);
  void processConnectionEvents();
  void applyConnectionEvent(const PendingConnectionEvent& event);
  void reconcileConnectionSet();
  void clearHostRpcIdentity();
  void noteHostRpcActivity(uint16_t connectionId,
                           const PeerAddress& peerAddress);
  void onOutput(const uint8_t* data, size_t length, uint16_t connectionId,
                const PeerAddress& peerAddress);
  void processOutput(const uint8_t* data, size_t length,
                     uint16_t connectionId, const PeerAddress& peerAddress);
  void onQuotaWrite(const uint8_t* data, size_t length,
                    bool responseExpected, const PeerAddress& peerAddress);
  void processQuotaWrites();
  void processQuotaWrite(const uint8_t* data, size_t length,
                         bool responseExpected,
                         const PeerAddress& peerAddress);
  bool handleRpc(const JsonDocument& request);
  void sendResult(JsonVariantConst id, JsonVariantConst result);
  void sendSuccess(JsonVariantConst id);
  void sendJson(const String& json);
  void updateThreadLighting(JsonArrayConst values);

  BLEHIDDevice* hid_ = nullptr;
  BLEServer* server_ = nullptr;
  BLEService* hidService_ = nullptr;
  BLECharacteristic* input_ = nullptr;
  BLECharacteristic* battery_ = nullptr;
  SemaphoreHandle_t stateMutex_ = nullptr;
  QueueHandle_t outputQueue_ = nullptr;
  QueueHandle_t connectionEventQueue_ = nullptr;
  QueueHandle_t quotaWriteQueue_ = nullptr;
  CodexMicroState state_;
  String rpcBuffer_;
  uint16_t rpcBufferConnectionId_ = 0;
  bool rpcBufferConnectionValid_ = false;
  uint16_t hostRpcConnectionId_ = 0;
  bool hostRpcConnectionValid_ = false;
  uint8_t batteryPercentage_ = 100;
  bool charging_ = false;
  connection_health::ConnectionSet connections_;
  std::atomic<bool> connectionEventLost_{false};
};
