// Codex Micro —— Waveshare 移植 step1：BLE 探针固件
//
// 目的：在投屏/触摸之前，先单独验证协议层在 Arduino-ESP32 3.3.7-cn 上能跑通：
//   1. 0x1812 HID 服务是否真的注册成功（串口看 "registered handle=" 是否非 0）
//   2. Windows 能否把它当 HID 设备配对
//   3. ChatGPT Desktop 是否下发 sys.version / device.status / v.oai.thstatus
//
// 不接屏、不接触摸、不接电源芯片。所有交互走串口。

#include <Arduino.h>
#include <BLEDevice.h>

#include "CodexMicroBle.h"

namespace {

// Waveshare ESP32-S3-Touch-AMOLED-1.8 的 BOOT 键接 GPIO0（低电平按下）。
constexpr uint8_t kButtonPin = 0;
constexpr uint32_t kStatusIntervalMs = 5000;
constexpr uint32_t kAdvCheckIntervalMs = 3000;

CodexMicroBle ble;
uint32_t lastStatusAtMs = 0;
uint32_t lastAdvCheckAtMs = 0;
bool lastButtonPressed = false;
uint32_t buttonPressAtMs = 0;

// NimBLE 兜底：断开后协议栈不一定可靠地恢复广播。一旦设备不广播，
// Windows 就无法重连，于是 ChatGPT 侧每次写 output report 都报
// 0x57 (ERROR_INVALID_PARAMETER)。这里每 3 秒兜一次底。
void ensureAdvertising() {
  BLEServer* server = BLEDevice::getServer();
  if (server == nullptr) return;
  if (server->getConnectedCount() > 0) return;
  if (BLEDevice::getAdvertising()->isAdvertising()) return;
  Serial.println("[ble] no peer and not advertising -> restart advertising");
  BLEDevice::startAdvertising();
}

void printHelp() {
  Serial.println();
  Serial.println("== Codex Micro step1 BLE probe ==");
  Serial.println("k <KEY> [act] [agent]  发按键，例如: k ACT10 1");
  Serial.println("j <angle> <distance>   发摇杆，例如: j 0.75 1.0 (上)");
  Serial.println("b <0..100>             设置电量百分比");
  Serial.println("s                      打印状态");
  Serial.println("h                      帮助");
  Serial.println("按 BOOT 键(GPIO0) = 发 ACT10");
  Serial.println();
}

void printStatus() {
  const CodexMicroState state = ble.snapshot();
  BLEServer* server = BLEDevice::getServer();
  const int peers = (server != nullptr) ? server->getConnectedCount() : -1;
  const bool advertising =
      (BLEDevice::getAdvertising() != nullptr) &&
      BLEDevice::getAdvertising()->isAdvertising();
  Serial.printf(
      "[status] connected=%d peers=%d adv=%d rpc=%d age=%lums quota=%s "
      "remaining=%.1f reset=%lus heap=%u\n",
      state.connected ? 1 : 0, peers, advertising ? 1 : 0,
      state.hostRpcObserved ? 1 : 0,
      state.lastHostRpcAtMs == 0
          ? 0UL
          : static_cast<unsigned long>(millis() - state.lastHostRpcAtMs),
      state.quota.available ? "yes" : "no", state.quota.remainingPercent,
      static_cast<unsigned long>(state.quota.resetInSeconds),
      static_cast<unsigned>(ESP.getFreeHeap()));
}

// 从串口读一整行（非阻塞由调用方轮询）。
bool readLine(String& out) {
  static String buffer;
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') continue;
    if (c == '\n') {
      out = buffer;
      buffer = "";
      return true;
    }
    buffer += c;
    if (buffer.length() > 128) buffer = "";
  }
  return false;
}

void handleCommand(const String& line) {
  String command = line;
  command.trim();
  if (command.isEmpty()) return;

  const int space = command.indexOf(' ');
  String head = command;
  String rest = "";
  if (space >= 0) {
    head = command.substring(0, space);
    rest = command.substring(space + 1);
    rest.trim();
  }
  head.toLowerCase();

  if (head == "h" || head == "help") {
    printHelp();
    return;
  }
  if (head == "s" || head == "status") {
    printStatus();
    return;
  }
  if (head == "k") {
    // k <KEY> [act] [agent]
    String key = rest;
    uint8_t action = 1;
    int8_t agent = -1;
    const int s1 = rest.indexOf(' ');
    if (s1 >= 0) {
      key = rest.substring(0, s1);
      String tail = rest.substring(s1 + 1);
      tail.trim();
      const int s2 = tail.indexOf(' ');
      if (s2 >= 0) {
        action = static_cast<uint8_t>(tail.substring(0, s2).toInt());
        agent = static_cast<int8_t>(tail.substring(s2 + 1).toInt());
      } else {
        action = static_cast<uint8_t>(tail.toInt());
      }
    }
    if (key.isEmpty()) key = "ACT10";
    ble.sendKey(key.c_str(), action, agent);
    return;
  }
  if (head == "j") {
    const int s1 = rest.indexOf(' ');
    if (s1 < 0) {
      Serial.println("usage: j <angle> <distance>");
      return;
    }
    const float angle = rest.substring(0, s1).toFloat();
    const float distance = rest.substring(s1 + 1).toFloat();
    ble.sendJoystick(angle, distance);
    return;
  }
  if (head == "b") {
    const uint8_t level = static_cast<uint8_t>(constrain(rest.toInt(), 0, 100));
    ble.setBattery(level, false);
    Serial.printf("battery=%u\n", static_cast<unsigned>(level));
    return;
  }
  Serial.printf("unknown command: %s (h for help)\n", command.c_str());
}

void handleButton() {
  const bool pressed = digitalRead(kButtonPin) == LOW;
  if (pressed && !lastButtonPressed) {
    buttonPressAtMs = millis();
    Serial.println("BOOT pressed");
  }
  if (!pressed && lastButtonPressed) {
    if (millis() - buttonPressAtMs >= 30) {
      ble.sendKey("ACT10", 1);
    }
  }
  lastButtonPressed = pressed;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("Codex Micro / Waveshare port / step1 BLE probe");
  Serial.printf("chip=%s rev=%u cores=%u flash=%uKB psram=%uKB\n",
                ESP.getChipModel(), static_cast<unsigned>(ESP.getChipRevision()),
                static_cast<unsigned>(ESP.getChipCores()),
                static_cast<unsigned>(ESP.getFlashChipSize() / 1024),
                static_cast<unsigned>(ESP.getPsramSize() / 1024));

  pinMode(kButtonPin, INPUT_PULLUP);

  ble.begin();
  // 注意：NimBLE 下 BLEService::m_handle 始终是 NULL_HANDLE(0xFFFF)，
  // getHandle() 拿不到真值，不能作为"0x1812 是否注册成功"的判据。
  // 真正的判据只有两个：Windows 能不能把它当 HID 配对、ChatGPT Desktop 会不会下发 RPC。
  Serial.printf("BLE stack=%s hid_handle=%u (NimBLE 下恒为 65535，无意义)\n",
                ble.stackName(), static_cast<unsigned>(ble.hidServiceHandle()));
  printHelp();
  lastStatusAtMs = millis();
}

void loop() {
  ble.poll();

  String line;
  if (readLine(line)) handleCommand(line);
  handleButton();

  if (millis() - lastAdvCheckAtMs >= kAdvCheckIntervalMs) {
    lastAdvCheckAtMs = millis();
    ensureAdvertising();
  }

  if (millis() - lastStatusAtMs >= kStatusIntervalMs) {
    lastStatusAtMs = millis();
    printStatus();
  }
  delay(10);
}
