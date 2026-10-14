// SPDX-License-Identifier: MIT
// Codex Micro BLE 协议层实现 —— 从 codex-micro-waveshare 移植，适配
// Arduino-ESP32 3.3.7-cn（NimBLE 后端）+ ArduinoJson 7.x。
//
// 传输要点（与上游一致，任何硬件上都一样）：
//   * HID over GATT 0x1812，VID 303A / PID 8360，vendor usage page 0xFF00，
//     report ID 6，报告体 63 字节。
//   * 设备 -> 主机：{"method":"v.oai.hid","params":{"k":"ACT10","act":1}}
//     {"method":"v.oai.rad","params":{"a":角度,"d":距离}}
//   * 主机 -> 设备：sys.version / device.status / v.oai.thstatus / v.oai.rgbcfg
//   * 分片：json + '\n'，按 61 字节切，每片 report[0]=2（分片标记）、report[1]=len。

#include "CodexMicroBle.h"

#include "HostRpcRequest.h"
#include "QuotaPayload.h"

#include <BLE2902.h>
#include <BLE2904.h>
#include <BLEDevice.h>
#include <BLEDescriptor.h>
#include <BLESecurity.h>
#include <BLEUtils.h>
#include <algorithm>

#if defined(CONFIG_BLUEDROID_ENABLED)
#include <esp_gap_ble_api.h>
#endif

namespace {

constexpr char kDeviceName[] = "Codex Micro";
constexpr char kManufacturer[] = "Work Louder";
constexpr char kFirmwareVersion[] = "0.1.0-waveshare-step1";
constexpr char kQuotaServiceUuid[] = "7f0d4e66-2ac2-4a71-bfbe-4ef61a0e5c01";
constexpr char kQuotaWriteUuid[] = "7f0d4e66-2ac2-4a71-bfbe-4ef61a0e5c02";
constexpr size_t kPayloadSize = 61;
constexpr size_t kReportBodySize = 63;
// 主机侧 HID 报告总长 = 1 字节 report ID + 63 字节报告体。
constexpr size_t kReportBytes = kReportBodySize + 1;

constexpr uint16_t swapBytes(uint16_t value) {
  return static_cast<uint16_t>((value << 8) | (value >> 8));
}

// 一个 vendor-defined input/output report。HIDAPI 会自己加/删 Report ID 6，
// BLE characteristic 上只承载剩下的 63 字节报告体。
const uint8_t kReportMap[] = {
    0x06, 0x00, 0xFF,  // Usage Page (Vendor Defined 0xFF00)
    0x09, 0x01,        // Usage (1)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x06,        // Report ID (6)
    0x15, 0x00,        // Logical Minimum (0)
    0x26, 0xFF, 0x00,  // Logical Maximum (255)
    0x75, 0x08,        // Report Size (8)
    0x95, 0x3F,        // Report Count (63)
    0x09, 0x01,        // Usage (1)
    0x81, 0x02,        // Input (Data, Variable, Absolute)
    0x95, 0x3F,        // Report Count (63)
    0x09, 0x02,        // Usage (2)
    0x91, 0x02,        // Output (Data, Variable, Absolute)
    0xC0               // End Collection
};

class SecurityCallbacks final : public BLESecurityCallbacks {
 public:
  bool onSecurityRequest() override { return true; }
  uint32_t onPassKeyRequest() override { return 0; }
  void onPassKeyNotify(uint32_t) override {}
  bool onConfirmPIN(uint32_t) override { return true; }
#if defined(CONFIG_BLUEDROID_ENABLED)
  void onAuthenticationComplete(esp_ble_auth_cmpl_t result) override {
    Serial.printf("BLE pairing %s\n", result.success ? "complete" : "failed");
  }
#elif defined(CONFIG_NIMBLE_ENABLED)
  void onAuthenticationComplete(ble_gap_conn_desc* result) override {
    const bool secure =
        result != nullptr && result->sec_state.encrypted && result->sec_state.bonded;
    Serial.printf("BLE pairing %s\n", secure ? "complete" : "failed");
  }
#endif
};

}  // namespace

class CodexMicroBle::ServerCallbacks final : public BLEServerCallbacks {
 public:
  explicit ServerCallbacks(CodexMicroBle& owner) : owner_(owner) {}

  // Arduino-ESP32 对每个事件会同时调用无参版本和带参版本，
  // 这里让无参版本保持安静，只在带参版本里做实际工作。
  void onConnect(BLEServer*) override {}
  void onDisconnect(BLEServer*) override {}

#if defined(CONFIG_BLUEDROID_ENABLED)
  void onConnect(BLEServer*, esp_ble_gatts_cb_param_t* param) override {
    if (param == nullptr) return;
    owner_.onConnectionEvent(true, param->connect.conn_id);

    // Windows(Intel 蓝牙) 重连时不会重新订阅 input report 的 CCCD，
    // 导致 notify 被主机静默忽略。强制把本地 CCCD 写成"通知已使能"。
    if (owner_.input_ != nullptr) {
      BLEDescriptor* cccd =
          owner_.input_->getDescriptorByUUID(BLEUUID(static_cast<uint16_t>(0x2902)));
      if (cccd != nullptr) {
        const uint8_t enabled[2] = {0x01, 0x00};
        cccd->setValue(const_cast<uint8_t*>(enabled), sizeof(enabled));
        Serial.println("BLE input CCCD forced to notifications");
      }
    }

    // 默认连接参数会在配对后几秒掉线，这里请求 15-30ms / latency 0 / 10s 超时。
    esp_ble_conn_update_params_t conn{};
    memcpy(conn.bda, param->connect.remote_bda, sizeof(conn.bda));
    conn.min_int = 0x0C;  // 15 ms
    conn.max_int = 0x18;  // 30 ms
    conn.latency = 0;
    conn.timeout = 1000;  // 10 s（单位 10ms）
    const esp_err_t rc = esp_ble_gap_update_conn_params(&conn);
    Serial.printf("BLE update_conn_params rc=%d\n", static_cast<int>(rc));
    BLEDevice::startAdvertising();
  }

  void onDisconnect(BLEServer*, esp_ble_gatts_cb_param_t* param) override {
    if (param != nullptr) {
      owner_.onConnectionEvent(false, param->disconnect.conn_id);
    }
    BLEDevice::startAdvertising();
  }
#elif defined(CONFIG_NIMBLE_ENABLED)
  void onConnect(BLEServer*, ble_gap_conn_desc* desc) override {
    if (desc == nullptr) return;
    owner_.onConnectionEvent(true, desc->conn_handle);
    // NimBLE 自动维护 CCCD 订阅状态，不需要（也无法）像 Bluedroid 那样强写。
    Serial.printf("BLE connect handle=%u encrypted=%u bonded=%u\n",
                  static_cast<unsigned>(desc->conn_handle),
                  static_cast<unsigned>(desc->sec_state.encrypted),
                  static_cast<unsigned>(desc->sec_state.bonded));
    // 注意：NimBLE 下不能在 onConnect 里 startAdvertising()。连接建立时
    // 广播已由协议栈停掉，此时再 adv_start() 会返回 BLE_ERR_CMD_DISALLOWED，
    // 并让协议栈后续对"是否需要重新广播"的判断失效——实测结果是断开后设备
    // 既不广播也不重连，Windows 写 output report 一律 0x57。重广播交给
    // onDisconnect 和主循环的兜底检查。
    // TODO(step1): 若实测 Windows 上仍会几秒掉线，再补 ble_gap_update_params()，
    // 需要 #include <host/ble_gap.h> 并填 ble_gap_upd_params。
  }

  void onDisconnect(BLEServer*, ble_gap_conn_desc* desc) override {
    if (desc != nullptr) {
      owner_.onConnectionEvent(false, desc->conn_handle);
    }
    Serial.printf("BLE disconnect handle=%u -> restart advertising\n",
                  static_cast<unsigned>(desc != nullptr ? desc->conn_handle
                                                        : 0xFFFF));
    // 断开后必须显式重启广播，否则 Windows 无法重连，
    // 表现为 ChatGPT 侧 hid_write 报 0x57 (ERROR_INVALID_PARAMETER)。
    BLEDevice::getAdvertising()->stop();
    BLEDevice::startAdvertising();
  }
#endif

 private:
  CodexMicroBle& owner_;
};

class CodexMicroBle::InputReportCallbacks final : public BLECharacteristicCallbacks {
 public:
  InputReportCallbacks() = default;

  // Arduino-ESP32 3.3.7-cn 的 BLECharacteristic::notify() 返回 **void**（不是
  // Bluedroid 时代的 bool），失败只能通过 onStatus() 异步上报。这是本机版本里
  // "notify 到底有没有真的交给协议栈"的唯一官方判据。
  void onStatus(BLECharacteristic*, Status status, uint32_t code) override {
    if (status == SUCCESS_NOTIFY || status == SUCCESS_INDICATE) {
      if (!reportedOk_) {
        reportedOk_ = true;
        Serial.println("notify path OK: 主机已订阅 input report (CCCD)");
      }
      return;
    }
    switch (status) {
      case ERROR_NO_SUBSCRIBER:
        Serial.println("notify FAILED: 主机未订阅 input report (CCCD=0)");
        break;
      case ERROR_NOTIFY_DISABLED:
        Serial.println("notify FAILED: characteristic 属性缺 PROPERTY_NOTIFY");
        break;
      case ERROR_NO_CLIENT:
        Serial.println("notify FAILED: 无已连接客户端");
        break;
      case ERROR_GATT:
        Serial.printf("notify FAILED: ble_gatts_notify_custom rc=%u\n",
                      static_cast<unsigned>(code));
        break;
      default:
        Serial.printf("notify status=%d code=%u\n", static_cast<int>(status),
                      static_cast<unsigned>(code));
        break;
    }
  }

 private:
  bool reportedOk_ = false;
};

class CodexMicroBle::OutputCallbacks final : public BLECharacteristicCallbacks {
 public:
  explicit OutputCallbacks(CodexMicroBle& owner) : owner_(owner) {}

#if defined(CONFIG_BLUEDROID_ENABLED)
  void onWrite(BLECharacteristic* characteristic,
               esp_ble_gatts_cb_param_t* param) override {
    if (param == nullptr) return;
    const auto value = characteristic->getValue();
    PeerAddress peer{};
    memcpy(peer.data(), param->write.bda, peer.size());
    owner_.onOutput(reinterpret_cast<const uint8_t*>(value.c_str()),
                    value.length(), param->write.conn_id, peer);
  }
#elif defined(CONFIG_NIMBLE_ENABLED)
  void onWrite(BLECharacteristic* characteristic,
               ble_gap_conn_desc* desc) override {
    const auto value = characteristic->getValue();
    PeerAddress peer{};
    uint16_t connId = 0;
    if (desc != nullptr) {
      memcpy(peer.data(), desc->peer_ota_addr.val, peer.size());
      connId = desc->conn_handle;
    }
    owner_.onOutput(reinterpret_cast<const uint8_t*>(value.c_str()),
                    value.length(), connId, peer);
  }
#endif

 private:
  CodexMicroBle& owner_;
};

class CodexMicroBle::QuotaCallbacks final : public BLECharacteristicCallbacks {
 public:
  explicit QuotaCallbacks(CodexMicroBle& owner) : owner_(owner) {}

#if defined(CONFIG_BLUEDROID_ENABLED)
  void onWrite(BLECharacteristic* characteristic,
               esp_ble_gatts_cb_param_t* param) override {
    if (param == nullptr) return;
    const auto value = characteristic->getValue();
    PeerAddress peer{};
    memcpy(peer.data(), param->write.bda, peer.size());
    owner_.onQuotaWrite(reinterpret_cast<const uint8_t*>(value.c_str()),
                        value.length(), param->write.need_rsp, peer);
  }
#elif defined(CONFIG_NIMBLE_ENABLED)
  void onWrite(BLECharacteristic* characteristic,
               ble_gap_conn_desc* desc) override {
    const auto value = characteristic->getValue();
    PeerAddress peer{};
    if (desc != nullptr) memcpy(peer.data(), desc->peer_ota_addr.val, peer.size());
    // NimBLE 的 onWrite 拿不到 need_rsp 语义，配额写入本就不需要 ATT 响应。
    owner_.onQuotaWrite(reinterpret_cast<const uint8_t*>(value.c_str()),
                        value.length(), false, peer);
  }
#endif

 private:
  CodexMicroBle& owner_;
};

void CodexMicroBle::begin() {
  stateMutex_ = xSemaphoreCreateMutex();
  outputQueue_ = xQueueCreate(16, sizeof(PendingOutputReport));
  if (outputQueue_ == nullptr) Serial.println("BLE output queue allocation failed");
  connectionEventQueue_ = xQueueCreate(16, sizeof(PendingConnectionEvent));
  if (connectionEventQueue_ == nullptr) {
    Serial.println("BLE connection event queue allocation failed");
  }
  quotaWriteQueue_ = xQueueCreate(4, sizeof(PendingQuotaWrite));
  if (quotaWriteQueue_ == nullptr) Serial.println("BLE quota write queue allocation failed");

  BLEDevice::init(kDeviceName);
  // 一次 HID 报告 64 字节，默认 MTU 装不下就写不进去。
  BLEDevice::setMTU(247);
  BLEDevice::setSecurityCallbacks(new SecurityCallbacks());

  auto* security = new BLESecurity();
  security->setCapability(ESP_IO_CAP_NONE);
  security->setAuthenticationMode(ESP_LE_AUTH_BOND);

  server_ = BLEDevice::createServer();
  server_->setCallbacks(new ServerCallbacks(*this));

  // ---- 手写 0x1812 HID over GATT 服务 ----
  // BLEHIDDevice 内部用的是 createService(0x1812, 40)（显式指定 40 个 handle），
  // 在 NimBLE 下这条注册会静默失败：实测 GATT 表里根本没有 0x1812，而同样流程
  // 的 180A / 180F（不指定 handle 数）都注册成功了。Windows 之所以还显示着一个
  // HID 设备，只是配对那一刻缓存下来的旧记录，所以它枚举得到、却永远写不进去
  // （hid_write 恒返回 ERROR_INVALID_PARAMETER 87）。
  // 这里改成自己按 HOGP 建表：特性一个不少，handle 数交给 NimBLE 自己算。
  BLEService* deviceInfo =
      server_->createService(BLEUUID(static_cast<uint16_t>(0x180A)));
  hidService_ = server_->createService(BLEUUID(static_cast<uint16_t>(0x1812)));
  BLEService* batteryService =
      server_->createService(BLEUUID(static_cast<uint16_t>(0x180F)));

  BLECharacteristic* manufacturer = deviceInfo->createCharacteristic(
      static_cast<uint16_t>(0x2A29), BLECharacteristic::PROPERTY_READ);
  manufacturer->setValue(kManufacturer);
  BLECharacteristic* pnp = deviceInfo->createCharacteristic(
      static_cast<uint16_t>(0x2A50), BLECharacteristic::PROPERTY_READ);
  // PnP characteristic 规范是小端，手写就不用 BLEHIDDevice 那套预交换。
  const uint8_t pnpValue[] = {0x02,
                              static_cast<uint8_t>(kVendorId & 0xFF),
                              static_cast<uint8_t>(kVendorId >> 8),
                              static_cast<uint8_t>(kProductId & 0xFF),
                              static_cast<uint8_t>(kProductId >> 8),
                              0x01,
                              0x01};
  pnp->setValue(const_cast<uint8_t*>(pnpValue), sizeof(pnpValue));

  BLECharacteristic* hidInfoChar = hidService_->createCharacteristic(
      static_cast<uint16_t>(0x2A4A), BLECharacteristic::PROPERTY_READ);
  const uint8_t hidInfoValue[] = {0x11, 0x01, 0x00, 0x01};
  hidInfoChar->setValue(const_cast<uint8_t*>(hidInfoValue),
                        sizeof(hidInfoValue));
  BLECharacteristic* reportMapChar = hidService_->createCharacteristic(
      static_cast<uint16_t>(0x2A4B), BLECharacteristic::PROPERTY_READ);
  reportMapChar->setValue(const_cast<uint8_t*>(kReportMap), sizeof(kReportMap));
  hidService_->createCharacteristic(static_cast<uint16_t>(0x2A4C),
                                    BLECharacteristic::PROPERTY_WRITE_NR);
  BLECharacteristic* protocolMode = hidService_->createCharacteristic(
      static_cast<uint16_t>(0x2A4E),
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE_NR);
  const uint8_t reportProtocol = 0x01;
  protocolMode->setValue(const_cast<uint8_t*>(&reportProtocol), 1);

  input_ = hidService_->createCharacteristic(
      static_cast<uint16_t>(0x2A4D),
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  // NimBLE 下 characteristic 的长度由第一次 setValue 决定；不预置的话 GATT 侧
  // 长度是 0，主机写 64 字节会被拒。预置成完整报告长度（report ID + 63）。
  uint8_t reportTemplate[kReportBytes];
  memset(reportTemplate, 0, sizeof(reportTemplate));
  reportTemplate[0] = kReportId;
  input_->setValue(reportTemplate, sizeof(reportTemplate));
  input_->setAccessPermissions(ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE);
  auto* inputReference =
      new BLEDescriptor(BLEUUID(static_cast<uint16_t>(0x2908)));
  const uint8_t inputReferenceValue[] = {kReportId, 0x01};
  inputReference->setValue(const_cast<uint8_t*>(inputReferenceValue),
                           sizeof(inputReferenceValue));
  inputReference->setAccessPermissions(ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE);
  input_->addDescriptor(inputReference);
  auto* inputCccd = new BLE2902();
  inputCccd->setAccessPermissions(ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE);
  input_->addDescriptor(inputCccd);
  // 3.3.7-cn 的 notify() 返回 void，失败只走 onStatus() 回调 —— 挂上它才有判据。
  input_->setCallbacks(new InputReportCallbacks());

  BLECharacteristic* output = hidService_->createCharacteristic(
      static_cast<uint16_t>(0x2A4D),
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE |
          BLECharacteristic::PROPERTY_WRITE_NR);
  output->setValue(reportTemplate, sizeof(reportTemplate));
  // 明文权限，不要 ENCRYPTED：Windows 的 hidbthle 会在完整 SMP 加密握手之前
  // 就来写 control point / protocol mode，ENCRYPTED 门会让链路刚配对就掉。
  output->setAccessPermissions(ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE);
  auto* outputReference =
      new BLEDescriptor(BLEUUID(static_cast<uint16_t>(0x2908)));
  const uint8_t outputReferenceValue[] = {kReportId, 0x02};
  outputReference->setValue(const_cast<uint8_t*>(outputReferenceValue),
                            sizeof(outputReferenceValue));
  outputReference->setAccessPermissions(ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE);
  output->addDescriptor(outputReference);
  output->setCallbacks(new OutputCallbacks(*this));

  battery_ = batteryService->createCharacteristic(
      static_cast<uint16_t>(0x2A19),
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  auto* batteryFormat = new BLE2904();
  batteryFormat->setFormat(BLE2904::FORMAT_UINT8);
  batteryFormat->setNamespace(1);
  batteryFormat->setUnit(0x27AD);
  battery_->addDescriptor(batteryFormat);
  auto* batteryCccd = new BLE2902();
  batteryCccd->setNotifications(true);
  batteryCccd->setAccessPermissions(ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE);
  battery_->addDescriptor(batteryCccd);
  battery_->setValue(&batteryPercentage_, 1);

  // NimBLE 的 GATT 属性表在 BLEServer::start() 之后就固定了，所以自定义服务
  // 必须在 server_->start() 之前 start。
  BLEService* quotaService = server_->createService(kQuotaServiceUuid);
  BLECharacteristic* quotaWrite = quotaService->createCharacteristic(
      kQuotaWriteUuid,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  quotaWrite->setAccessPermissions(ESP_GATT_PERM_WRITE);
  quotaWrite->setCallbacks(new QuotaCallbacks(*this));

  // 逐个打印 start 结果：这是唯一能确认 0x1812 真的进了 GATT 表的手段
  // （NimBLE 下 BLEService::getHandle() 恒为 0xFFFF，不能当判据）。
  Serial.printf("start deviceInfo=%d hid1812=%d battery=%d quota=%d\n",
                static_cast<int>(deviceInfo->start()),
                static_cast<int>(hidService_->start()),
                static_cast<int>(batteryService->start()),
                static_cast<int>(quotaService->start()));
  server_->start();

  // Service 的 handle 在 NimBLE 下拿不到（BLEService::m_handle 恒为 NULL_HANDLE），
  // 但 characteristic 的 handle 会被 ble_gatts_start() 通过 val_handle 指针写回来
  // —— 这是"某个 characteristic 到底有没有进 GATT 表"的真判据。必须放在
  // server_->start() 之后：handle 是 ble_gatts_start() 那一刻才分配的。
  // HOGP 要求 input/output 两个 Report characteristic 共用 0x2A4D，只靠 0x2908
  // 区分；两者 handle 都必须有效。
  Serial.printf("hid chr handle input=%u output=%u battery=%u (0xFFFF=未注册)\n",
                static_cast<unsigned>(input_ != nullptr ? input_->getHandle() : 0xFFFF),
                static_cast<unsigned>(output->getHandle()),
                static_cast<unsigned>(battery_->getHandle()));

  BLEAdvertising* advertising = BLEDevice::getAdvertising();
  advertising->setAppearance(GENERIC_HID);
  advertising->addServiceUUID(hidService_->getUUID());
  advertising->addServiceUUID(kQuotaServiceUuid);
  advertising->setScanResponse(true);
#if defined(CONFIG_BLUEDROID_ENABLED)
  advertising->setMinPreferred(0x06);  // 7.5 ms
  advertising->setMaxPreferred(0x12);  // 22.5 ms
#endif
  BLEDevice::startAdvertising();

  Serial.printf("BLE vendor HID ready VID=%04X PID=%04X usage=FF00 report=%u "
                "reportBytes=%u mtu=%u\n",
                kVendorId, kProductId, kReportId,
                static_cast<unsigned>(kReportBytes), BLEDevice::getMTU());
  Serial.printf("Quota GATT ready service=%s write=%s\n", kQuotaServiceUuid,
                kQuotaWriteUuid);
}

uint16_t CodexMicroBle::hidServiceHandle() const {
  return hidService_ != nullptr ? hidService_->getHandle() : 0;
}

const char* CodexMicroBle::stackName() const {
#if defined(CONFIG_BLUEDROID_ENABLED)
  return "bluedroid";
#elif defined(CONFIG_NIMBLE_ENABLED)
  return "nimble";
#else
  return "unknown";
#endif
}

void CodexMicroBle::poll() {
  processConnectionEvents();
  reconcileConnectionSet();
  processQuotaWrites();

  if (outputQueue_ == nullptr) return;

  PendingOutputReport report;
  while (xQueueReceive(outputQueue_, &report, 0) == pdTRUE) {
    if (report.length == 0) {
      rpcBuffer_.clear();
      continue;
    }
    processOutput(report.data.data(), report.length, report.connectionId,
                  report.peerAddress);
  }
}

void CodexMicroBle::setBattery(uint8_t percentage, bool charging) {
  batteryPercentage_ = constrain(percentage, 0, 100);
  charging_ = charging;
  // battery_ 现在由手写 GATT 表创建，不再走 BLEHIDDevice。
  if (battery_ != nullptr) {
    battery_->setValue(&batteryPercentage_, 1);
    if (connected()) battery_->notify();
  }
}

void CodexMicroBle::sendKey(const char* key, uint8_t action, int8_t agent) {
  JsonDocument message;
  message["method"] = "v.oai.hid";
  JsonObject params = message.createNestedObject("params");
  params["k"] = key;
  params["act"] = action;
  if (agent >= 0) params["ag"] = agent;

  String json;
  serializeJson(message, json);
  sendJson(json);
  Serial.printf("HID key=%s action=%u\n", key, action);
}

void CodexMicroBle::sendJoystick(float angle, float distance) {
  JsonDocument message;
  message["method"] = "v.oai.rad";
  JsonObject params = message.createNestedObject("params");
  params["a"] = angle;
  params["d"] = distance;

  String json;
  serializeJson(message, json);
  sendJson(json);
  Serial.printf("HID joystick angle=%.2f distance=%.2f\n", angle, distance);
}

bool CodexMicroBle::connected() {
  if (stateMutex_ != nullptr) {
    xSemaphoreTake(stateMutex_, portMAX_DELAY);
    const bool result = state_.connected;
    xSemaphoreGive(stateMutex_);
    if (result) return true;
  }
  if (server_ != nullptr && server_->getConnectedCount() > 0) return true;
  // 上游在 Windows 上观察到：server 自己的计数器报 0，但主机明明在写。
  // 兜底去看真实 peer 列表，它不依赖 connect/disconnect 记账。
  if (server_ != nullptr) {
    const auto peers = server_->getPeerDevices(false);
    if (!peers.empty()) return true;
  }
  return false;
}

CodexMicroState CodexMicroBle::snapshot() {
  CodexMicroState copy;
  if (stateMutex_ == nullptr) return copy;
  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  copy = state_;
  state_.dirty = false;
  xSemaphoreGive(stateMutex_);
  return copy;
}

void CodexMicroBle::onConnectionEvent(bool connected, uint16_t id) {
  if (connectionEventQueue_ == nullptr) {
    connectionEventLost_.store(true, std::memory_order_release);
    return;
  }
  PendingConnectionEvent event;
  event.id = id;
  event.connected = connected;
  if (xQueueSend(connectionEventQueue_, &event, 0) != pdTRUE) {
    connectionEventLost_.store(true, std::memory_order_release);
  }
}

void CodexMicroBle::applyConnectionEvent(const PendingConnectionEvent& event) {
  const connection_health::ConnectionSet::Transition transition =
      connections_.apply(event.connected, event.id);
  if (transition.overflow) {
    connectionEventLost_.store(true, std::memory_order_release);
    return;
  }
  if (!transition.changed || stateMutex_ == nullptr) return;

  const uint8_t count = static_cast<uint8_t>(connections_.count());

  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  const bool hostDisconnected =
      !event.connected && hostRpcConnectionValid_ &&
      hostRpcConnectionId_ == event.id;
  if (transition.becameConnected) {
    ++state_.connectionEpoch;
    state_.lastHostRpcAtMs = 0;
    state_.hostRpcObserved = false;
    clearHostRpcIdentity();
  } else if (transition.becameDisconnected || hostDisconnected) {
    state_.lastHostRpcAtMs = 0;
    state_.hostRpcObserved = false;
    clearHostRpcIdentity();
  }
  state_.connected = count > 0;
  state_.dirty = true;
  const uint32_t connectionEpoch = state_.connectionEpoch;
  xSemaphoreGive(stateMutex_);

  if (transition.becameConnected || transition.becameDisconnected ||
      (!event.connected && rpcBufferConnectionValid_ &&
       rpcBufferConnectionId_ == event.id)) {
    rpcBuffer_.clear();
    rpcBufferConnectionValid_ = false;
  }
  Serial.printf("BLE host event=%s id=%u count=%u epoch=%lu\n",
                event.connected ? "connected" : "disconnected", event.id,
                count, static_cast<unsigned long>(connectionEpoch));
}

void CodexMicroBle::processConnectionEvents() {
  if (connectionEventQueue_ == nullptr) return;
  PendingConnectionEvent event;
  while (xQueueReceive(connectionEventQueue_, &event, 0) == pdTRUE) {
    applyConnectionEvent(event);
  }
}

void CodexMicroBle::reconcileConnectionSet() {
  if (server_ == nullptr || stateMutex_ == nullptr) return;
  const bool eventLost =
      connectionEventLost_.exchange(false, std::memory_order_acq_rel);
  if (!eventLost) return;
  connection_health::ConnectionSet observed;
  bool overflow = false;
  const auto peers = server_->getPeerDevices(false);
  for (const auto& peer : peers) {
    if (observed.apply(true, peer.first).overflow) overflow = true;
  }
  if (overflow) {
    connectionEventLost_.store(true, std::memory_order_release);
    return;
  }

  connections_ = observed;
  const uint8_t count = static_cast<uint8_t>(connections_.count());
  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  if (count > 0) ++state_.connectionEpoch;
  state_.connected = count > 0;
  state_.lastHostRpcAtMs = 0;
  state_.hostRpcObserved = false;
  clearHostRpcIdentity();
  state_.dirty = true;
  const uint32_t epoch = state_.connectionEpoch;
  xSemaphoreGive(stateMutex_);
  rpcBuffer_.clear();
  rpcBufferConnectionValid_ = false;
  Serial.printf("BLE connections reconciled count=%u epoch=%lu\n", count,
                static_cast<unsigned long>(epoch));
}

void CodexMicroBle::clearHostRpcIdentity() { hostRpcConnectionValid_ = false; }

void CodexMicroBle::noteHostRpcActivity(uint16_t connectionId,
                                        const PeerAddress& peerAddress) {
  (void)peerAddress;
  if (stateMutex_ == nullptr || !connections_.contains(connectionId)) return;
  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  if (!state_.connected) {
    xSemaphoreGive(stateMutex_);
    return;
  }
  const bool promoted = !state_.hostRpcObserved;
  state_.hostRpcObserved = true;
  state_.lastHostRpcAtMs = millis();
  hostRpcConnectionId_ = connectionId;
  hostRpcConnectionValid_ = true;
  if (promoted) state_.dirty = true;
  xSemaphoreGive(stateMutex_);
}

void CodexMicroBle::onOutput(const uint8_t* data, size_t length,
                             uint16_t connectionId,
                             const PeerAddress& peerAddress) {
  if (data == nullptr || length == 0 || outputQueue_ == nullptr) return;

  PendingOutputReport report;
  report.length = static_cast<uint8_t>(std::min<size_t>(length, report.data.size()));
  report.connectionId = connectionId;
  report.peerAddress = peerAddress;
  memcpy(report.data.data(), data, report.length);
  xQueueSend(outputQueue_, &report, 0);
}

void CodexMicroBle::processOutput(const uint8_t* data, size_t length,
                                  uint16_t connectionId,
                                  const PeerAddress& peerAddress) {
  // step1 诊断：主机写没到达设备，就看这一行出不出来。
  if (data != nullptr && length > 0) {
    Serial.printf("HID OUT len=%u conn=%u head=%02x %02x %02x\n",
                  static_cast<unsigned>(length), connectionId, data[0],
                  length > 1 ? data[1] : 0, length > 2 ? data[2] : 0);
  }
  if (data == nullptr || length < 2 || !connections_.contains(connectionId)) {
    return;
  }

  if (rpcBufferConnectionValid_ && rpcBufferConnectionId_ != connectionId) {
    rpcBuffer_.clear();
    rpcBufferConnectionValid_ = false;
  }

  // HOGP 通常会剥掉 report ID；也接受带着 ID 的原始转发。
  size_t offset = (length >= 3 && data[0] == kReportId) ? 1 : 0;
  if (length < offset + 2 || data[offset] != 2) return;

  const size_t payloadLength = std::min<size_t>(data[offset + 1], kPayloadSize);
  if (length < offset + 2 + payloadLength) return;
  const char* payload = reinterpret_cast<const char*>(data + offset + 2);
  constexpr char kTopLevelPrefix[] = "{\"method\"";
  const bool startsTopLevel =
      payloadLength >= sizeof(kTopLevelPrefix) - 1 &&
      memcmp(payload, kTopLevelPrefix, sizeof(kTopLevelPrefix) - 1) == 0;
  if (startsTopLevel && !rpcBuffer_.isEmpty()) {
    // 出现新的顶层对象说明上一次分片写丢了，立刻重新同步，别污染下一个请求。
    rpcBuffer_.clear();
  }
  if (rpcBuffer_.isEmpty()) {
    size_t jsonStart = 0;
    while (jsonStart < payloadLength && payload[jsonStart] != '{') ++jsonStart;
    if (jsonStart == payloadLength) return;
    rpcBuffer_.concat(payload + jsonStart, payloadLength - jsonStart);
    rpcBufferConnectionId_ = connectionId;
    rpcBufferConnectionValid_ = true;
  } else {
    rpcBuffer_.concat(payload, payloadLength);
  }

  JsonDocument request;
  const DeserializationError error = deserializeJson(request, rpcBuffer_);
  if (error == DeserializationError::IncompleteInput) return;
  if (error) {
    Serial.printf("RPC parse error: %s\n", error.c_str());
    rpcBuffer_.clear();
    return;
  }

  if (handleRpc(request)) {
    noteHostRpcActivity(connectionId, peerAddress);
  }
  rpcBuffer_.clear();
  rpcBufferConnectionValid_ = false;
}

void CodexMicroBle::onQuotaWrite(const uint8_t* data, size_t length,
                                 bool responseExpected,
                                 const PeerAddress& peerAddress) {
  if (data == nullptr || length == 0 || length > 512 ||
      quotaWriteQueue_ == nullptr) {
    Serial.println("Quota update rejected: invalid length");
    return;
  }

  PendingQuotaWrite write;
  write.length = static_cast<uint16_t>(length);
  write.responseExpected = responseExpected;
  write.peerAddress = peerAddress;
  memcpy(write.data.data(), data, length);
  if (xQueueSend(quotaWriteQueue_, &write, 0) != pdTRUE) {
    Serial.println("Quota update rejected: queue full");
  }
}

void CodexMicroBle::processQuotaWrites() {
  if (quotaWriteQueue_ == nullptr) return;
  PendingQuotaWrite write;
  while (xQueueReceive(quotaWriteQueue_, &write, 0) == pdTRUE) {
    processQuotaWrite(write.data.data(), write.length, write.responseExpected,
                      write.peerAddress);
  }
}

void CodexMicroBle::processQuotaWrite(const uint8_t* data, size_t length,
                                      bool responseExpected,
                                      const PeerAddress& peerAddress) {
  (void)responseExpected;
  (void)peerAddress;
  if (data == nullptr || length == 0 || length > 512) {
    Serial.println("Quota update rejected: invalid length");
    return;
  }

  JsonDocument update;
  const DeserializationError error = deserializeJson(update, data, length);
  if (error || !update.is<JsonObject>()) {
    Serial.printf("Quota update rejected: %s\n", error.c_str());
    return;
  }

  quota_payload::Snapshot snapshot;
  if (!quota_payload::parse(update.as<JsonObjectConst>(), snapshot)) {
    Serial.println("Quota update rejected: invalid fields");
    return;
  }

  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  state_.quota.remainingPercent = snapshot.remainingPercent;
  state_.quota.resetInSeconds = snapshot.resetInSeconds;
  state_.quota.receivedAtMs = millis();
  state_.quota.available = true;
  state_.dirty = true;
  xSemaphoreGive(stateMutex_);

  Serial.printf("Quota update remaining=%.1f reset=%lus\n",
                snapshot.remainingPercent,
                static_cast<unsigned long>(snapshot.resetInSeconds));
}

bool CodexMicroBle::handleRpc(const JsonDocument& request) {
  const JsonObjectConst requestObject = request.as<JsonObjectConst>();
  const char* method = requestObject["method"] | "";
  const JsonVariantConst id = requestObject["id"];
  const JsonVariantConst params = requestObject["params"];
  const host_rpc::Method supportedMethod = host_rpc::classify(requestObject);
  Serial.printf("RPC method=%s id=%s\n", method,
                id.isNull() ? "-" : String(id.as<long>()).c_str());

  if (supportedMethod == host_rpc::Method::SystemVersion) {
    JsonDocument resultDoc;
    resultDoc["version"] = kFirmwareVersion;
    sendResult(id, resultDoc.as<JsonVariantConst>());
    return true;
  }

  if (supportedMethod == host_rpc::Method::DeviceStatus) {
    JsonDocument resultDoc;
    resultDoc["version"] = kFirmwareVersion;
    resultDoc["profile_index"] = 0;
    resultDoc["layer_index"] = 1;
    resultDoc["battery"] = batteryPercentage_;
    resultDoc["is_charging"] = charging_;
    sendResult(id, resultDoc.as<JsonVariantConst>());
    return true;
  }

  if (supportedMethod == host_rpc::Method::ThreadStatus) {
    updateThreadLighting(params.as<JsonArrayConst>());
    sendSuccess(id);
    return true;
  }

  if (supportedMethod == host_rpc::Method::RgbConfig ||
      supportedMethod == host_rpc::Method::LightsPreview ||
      supportedMethod == host_rpc::Method::HostFocusedApp) {
    sendSuccess(id);
    return true;
  }

  JsonDocument response;
  response["id"] = id;
  JsonObject errorObject = response.createNestedObject("error");
  errorObject["code"] = -32601;
  errorObject["message"] = "Method not found";
  String json;
  serializeJson(response, json);
  sendJson(json);
  return false;
}

void CodexMicroBle::sendResult(JsonVariantConst id, JsonVariantConst result) {
  JsonDocument response;
  response["id"] = id;
  response["result"] = result;
  String json;
  serializeJson(response, json);
  sendJson(json);
}

void CodexMicroBle::sendSuccess(JsonVariantConst id) {
  JsonDocument resultDoc;
  resultDoc["ok"] = true;
  sendResult(id, resultDoc.as<JsonVariantConst>());
}

void CodexMicroBle::sendJson(const String& json) {
  if (input_ == nullptr) {
    Serial.println("sendJson dropped: input_=null");
    return;
  }
  if (!connected()) {
    Serial.println("sendJson dropped: not connected");
    return;
  }

  String framed = json;
  framed += '\n';
  size_t offset = 0;
  uint16_t sent = 0;
  while (offset < framed.length()) {
    const size_t chunk = std::min<size_t>(kPayloadSize, framed.length() - offset);
    uint8_t report[kReportBodySize] = {};
    report[0] = 2;
    report[1] = chunk;
    memcpy(report + 2, framed.c_str() + offset, chunk);
    input_->setValue(report, sizeof(report));
    // 注意：3.3.7-cn 的 notify() 返回 void，拿不到成败。失败由挂在 input_ 上的
    // InputReportCallbacks::onStatus() 异步打印（ERROR_NO_SUBSCRIBER / ERROR_GATT ...）。
    input_->notify();
    ++sent;
    offset += chunk;
    delay(4);
  }
  Serial.printf("sendJson sent chunks=%u bytes=%u\n", sent,
                static_cast<unsigned>(framed.length()));
}

void CodexMicroBle::updateThreadLighting(JsonArrayConst values) {
  xSemaphoreTake(stateMutex_, portMAX_DELAY);
  for (JsonObjectConst value : values) {
    const int id = value["id"] | -1;
    if (id < 0 || id >= static_cast<int>(state_.threads.size())) continue;
    ThreadLight& light = state_.threads[id];
    light.color = value["c"] | light.color;
    light.brightness = value["b"] | light.brightness;
    light.effect = value["e"] | light.effect;
    light.speed = value["s"] | light.speed;
    // step1：串口就是显示器，直接把 RGB 打出来，方便肉眼确认通道真的通了。
    Serial.printf("TH[%d] rgb=#%06X b=%.2f e=%s s=%.2f\n", id,
                  static_cast<unsigned>(light.color & 0xFFFFFF),
                  light.brightness, light.effect.c_str(), light.speed);
  }
  state_.dirty = true;
  xSemaphoreGive(stateMutex_);
}
