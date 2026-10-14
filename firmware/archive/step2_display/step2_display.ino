// Codex Micro —— Waveshare 移植 step2：BLE 协议层 + CO5300 屏
//
// 复刻顺序的第 ② 步：只加「屏」这一个变量，验证它能否和已跑通的 BLE
// 协议层共存。屏上先放一个状态页（不追求上游的最终版式，那是第 ④ 步），
// 目的是把下面这些一次性看全：
//   - CO5300 368x448 起没起来、帧缓冲在不在 PSRAM
//   - 16 位色是否正确（6 路线程灯按 ChatGPT 下发的 RGB 上色）
//   - BLE 连接 / RPC 活性 / 配额 是否随主机轮询实时变化
//   - 持续刷屏会不会影响 BLE 收发（上游 1.75C 上刷屏和协议栈是抢 CPU 的）
//
// 触摸（FT3168/CST820）留到第 ③ 步，本文件不碰 I2C。

#include <Arduino.h>
#include <BLEDevice.h>

#include "BoardDisplay.h"
#include "CodexMicroBle.h"

namespace {

// Waveshare ESP32-S3-Touch-AMOLED-1.8 的 BOOT 键接 GPIO0（低电平按下）。
constexpr uint8_t kButtonPin = 0;
constexpr uint32_t kStatusIntervalMs = 5000;   // 串口状态行
constexpr uint32_t kAdvCheckIntervalMs = 3000; // 广播兜底
constexpr uint32_t kRedrawIntervalMs = 500;    // 全屏刷新周期

BoardDisplay display;
CodexMicroBle ble;

uint32_t lastStatusAtMs = 0;
uint32_t lastAdvCheckAtMs = 0;
uint32_t lastRedrawAtMs = 0;
bool lastButtonPressed = false;
uint32_t buttonPressAtMs = 0;
uint32_t redrawCount = 0;

constexpr uint16_t kColBg = 0x0841;      // 近黑
constexpr uint16_t kColPanel = 0x18E3;   // 卡片底
constexpr uint16_t kColTitleBg = 0x01A7; // 标题条
constexpr uint16_t kColText = RGB565_WHITE;
constexpr uint16_t kColDim = RGB565_DARKGRAY;
constexpr uint16_t kColOk = RGB565_GREEN;
constexpr uint16_t kColWarn = RGB565_YELLOW;
constexpr uint16_t kColOff = RGB565_RED;
constexpr uint16_t kColAccent = RGB565_CYAN;

// 0xRRGGBB -> RGB565，并按亮度缩放（线程灯的 brightness 是 0..1）。
uint16_t scaledColor(uint32_t rgb, float scale) {
  if (scale < 0.0f) scale = 0.0f;
  if (scale > 1.0f) scale = 1.0f;
  const uint8_t r = static_cast<uint8_t>(((rgb >> 16) & 0xFF) * scale);
  const uint8_t g = static_cast<uint8_t>(((rgb >> 8) & 0xFF) * scale);
  const uint8_t b = static_cast<uint8_t>((rgb & 0xFF) * scale);
  return static_cast<uint16_t>((static_cast<uint16_t>(r & 0xF8) << 8) |
                               (static_cast<uint16_t>(g & 0xFC) << 3) |
                               (b >> 3));
}

// 单行文本。size=1 -> 6x8，size=2 -> 12x16。
void text(const char* s, int16_t x, int16_t y, uint8_t size, uint16_t fg,
          uint16_t bg) {
  Arduino_GFX* g = display.gfx();
  g->setTextColor(fg, bg);
  g->setTextSize(size);
  g->setCursor(x, y);
  g->print(s);
}

// 打印"标签 + 右对齐数值"的一行，背景填成卡片色。
void labelValue(const char* label, const String& value, int16_t x, int16_t y,
                int16_t w, uint16_t valueColor) {
  Arduino_GFX* g = display.gfx();
  g->fillRect(x, y, w, 16, kColPanel);
  text(label, x, y + 4, 1, kColDim, kColPanel);
  const int16_t vw = static_cast<int16_t>(value.length()) * 6;
  int16_t vx = x + w - vw;
  if (vx < x) vx = x;
  text(value.c_str(), vx, y + 4, 1, valueColor, kColPanel);
}

void drawFrame() {
  Arduino_GFX* g = display.gfx();
  const CodexMicroState state = ble.snapshot();
  BLEServer* server = BLEDevice::getServer();
  const int peers = (server != nullptr) ? server->getConnectedCount() : -1;
  const bool advertising = (BLEDevice::getAdvertising() != nullptr) &&
                           BLEDevice::getAdvertising()->isAdvertising();

  g->fillScreen(kColBg);

  // ---- 标题条 ----
  g->fillRect(0, 0, BoardDisplay::kWidth, 40, kColTitleBg);
  text("CODEX MICRO", 12, 12, 2, kColText, kColTitleBg);
  text("step2", BoardDisplay::kWidth - 12 - 5 * 6, 16, 1, kColAccent,
       kColTitleBg);

  // ---- 板信息 ----
  char line[64];
  snprintf(line, sizeof(line), "CO5300 %dx%d  canvas=%d",
           BoardDisplay::kWidth, BoardDisplay::kHeight,
           display.doubleBuffered() ? 1 : 0);
  text(line, 12, 52, 1, kColDim, kColBg);

  // ---- 链路状态卡片 ----
  const int16_t cx = 12;
  const int16_t cw = BoardDisplay::kWidth - 24;
  g->fillRect(cx, 76, cw, 16 * 4 + 8, kColPanel);

  const bool linkOk = state.connected && state.hostRpcObserved;
  labelValue("LINK", linkOk ? "codex live"
                            : (state.connected ? "ble only" : "offline"),
              cx + 8, 82, cw - 16, linkOk ? kColOk : (state.connected ? kColWarn : kColOff));

  snprintf(line, sizeof(line), "conn=%d peers=%d adv=%d",
           state.connected ? 1 : 0, peers, advertising ? 1 : 0);
  labelValue("BLE", line, cx + 8, 102, cw - 16,
             state.connected ? kColText : kColDim);

  if (state.hostRpcObserved) {
    snprintf(line, sizeof(line), "age=%lums",
             static_cast<unsigned long>(millis() - state.lastHostRpcAtMs));
  } else {
    snprintf(line, sizeof(line), "no rpc yet");
  }
  labelValue("RPC", line, cx + 8, 122, cw - 16,
             state.hostRpcObserved ? kColText : kColDim);

  if (state.quota.available) {
    snprintf(line, sizeof(line), "%.1f%% reset=%lus",
             state.quota.remainingPercent,
             static_cast<unsigned long>(state.quota.resetInSeconds));
  } else {
    snprintf(line, sizeof(line), "waiting");
  }
  labelValue("QUOTA", line, cx + 8, 142, cw - 16,
             state.quota.available ? kColAccent : kColDim);

  // ---- 6 路线程灯：上游只会通过 thstatus 下发 RGB 阈值，这里把
  //      拿到的 color/brightness 直接画出来，顺带验证 16 位色。----
  text("THREADS", 12, 176, 1, kColDim, kColBg);

  constexpr int16_t kCellW = 108;
  constexpr int16_t kCellH = 78;
  constexpr int16_t kGapX = 10;
  constexpr int16_t kGapY = 10;
  for (int i = 0; i < 6; ++i) {
    const int16_t col = i % 3;
    const int16_t row = i / 3;
    const int16_t bx = 12 + col * (kCellW + kGapX);
    const int16_t by = 190 + row * (kCellH + kGapY);

    const ThreadLight& tl = state.threads[i];
    const uint16_t swatch = scaledColor(tl.color, tl.brightness);

    g->fillRect(bx, by, kCellW, kCellH, kColPanel);
    // 色块
    g->fillRect(bx + 6, by + 6, kCellW - 12, 36, swatch);
    g->drawRect(bx + 6, by + 6, kCellW - 12, 36, kColDim);

    snprintf(line, sizeof(line), "A%d #%06lX", i,
             static_cast<unsigned long>(tl.color & 0xFFFFFF));
    text(line, bx + 6, by + 48, 1, kColText, kColPanel);

    snprintf(line, sizeof(line), "b=%.2f %s", tl.brightness,
             tl.effect.c_str());
    text(line, bx + 6, by + 62, 1, kColDim, kColPanel);
  }

  // ---- 底部 ----
  snprintf(line, sizeof(line), "heap=%u redraw=%lu",
           static_cast<unsigned>(ESP.getFreeHeap()),
           static_cast<unsigned long>(redrawCount));
  text(line, 12, BoardDisplay::kHeight - 16, 1, kColDim, kColBg);

  display.flush();
  ++redrawCount;
  lastRedrawAtMs = millis();
}

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
  Serial.println("== Codex Micro step2 (BLE + CO5300) ==");
  Serial.println("k <KEY> [act] [agent]  发按键，例如: k ACT10 1");
  Serial.println("j <angle> <distance>   发摇杆，例如: j 0.75 1.0");
  Serial.println("b <0..100>             设置电量百分比");
  Serial.println("s                      打印状态");
  Serial.println("t                      屏自检（红/绿/蓝/彩条，驻留 12s 后自动复位）");
  Serial.println("h                      帮助");
  Serial.println();
}

void printStatus() {
  const CodexMicroState state = ble.snapshot();
  BLEServer* server = BLEDevice::getServer();
  const int peers = (server != nullptr) ? server->getConnectedCount() : -1;
  const bool advertising = (BLEDevice::getAdvertising() != nullptr) &&
                           BLEDevice::getAdvertising()->isAdvertising();
  Serial.printf(
      "[status] connected=%d peers=%d adv=%d rpc=%d age=%lums quota=%s "
      "remaining=%.1f reset=%lus heap=%u canvas=%d\n",
      state.connected ? 1 : 0, peers, advertising ? 1 : 0,
      state.hostRpcObserved ? 1 : 0,
      state.lastHostRpcAtMs == 0
          ? 0UL
          : static_cast<unsigned long>(millis() - state.lastHostRpcAtMs),
      state.quota.available ? "yes" : "no", state.quota.remainingPercent,
      static_cast<unsigned long>(state.quota.resetInSeconds),
      static_cast<unsigned>(ESP.getFreeHeap()),
      display.doubleBuffered() ? 1 : 0);
}

// 屏自检：驻留循环模式。触发后循环播放 红/绿/蓝/彩条，每帧停 1.5 秒，
// 播两轮后自动回到状态页。
//
// 为什么不做成"延时连播"：延时里板子会在 selfTest() 内阻塞，命令行发完
// 就结束了，而每一帧只停几百毫秒——人眼根本来不及判断色序/花屏/偏移。
// 驻留循环把这段拉长到 12 秒，且自检期间暂停常规 drawFrame()，画面不会被
// 500ms 的状态刷新盖掉。复位后没人能再敲命令，所以必须自己退出。
constexpr uint32_t kSelfTestFrameMs = 1500;
constexpr int kSelfTestFrames = 4;    // 红 / 绿 / 蓝 / 彩条
constexpr int kSelfTestRounds = 2;
constexpr uint32_t kSelfTestTotalMs =
    kSelfTestFrameMs * kSelfTestFrames * kSelfTestRounds;

uint32_t selfTestEndAtMs = 0;        // 0 = 未处于自检模式
uint32_t selfTestNextFrameAtMs = 0;
int selfTestFrameIndex = 0;

void drawSelfTestFrame(int index) {
  Arduino_GFX* g = display.gfx();
  switch (index) {
    case 0:
      Serial.println("[screen] frame 1/4 PURE RED");
      g->fillScreen(RGB565_RED);
      break;
    case 1:
      Serial.println("[screen] frame 2/4 PURE GREEN");
      g->fillScreen(RGB565_GREEN);
      break;
    case 2:
      Serial.println("[screen] frame 3/4 PURE BLUE");
      g->fillScreen(RGB565_BLUE);
      break;
    default: {
      Serial.println("[screen] frame 4/4 BARS top->bottom "
                     "R,G,B,WHITE,BLACK,YELLOW,CYAN,MAGENTA");
      const uint16_t bars[] = {RGB565_RED,    RGB565_GREEN, RGB565_BLUE,
                               RGB565_WHITE,  RGB565_BLACK, RGB565_YELLOW,
                               RGB565_CYAN,   RGB565_MAGENTA};
      const int16_t h = BoardDisplay::kHeight / 8;
      for (int i = 0; i < 8; ++i) {
        g->fillRect(0, i * h, BoardDisplay::kWidth, h, bars[i]);
      }
      // 1px 白边框：判断有没有整体偏移、边缘被裁或行错位。
      g->drawRect(0, 0, BoardDisplay::kWidth, BoardDisplay::kHeight,
                  RGB565_WHITE);
      break;
    }
  }
  display.flush();
}

void beginSelfTest() {
  Serial.printf("[screen] self test start: %d frames x %lums x %d rounds "
                "(~%lus)\n",
                kSelfTestFrames, static_cast<unsigned long>(kSelfTestFrameMs),
                kSelfTestRounds,
                static_cast<unsigned long>(kSelfTestTotalMs / 1000));
  selfTestFrameIndex = 0;
  const uint32_t now = millis();
  selfTestEndAtMs = now + kSelfTestTotalMs;
  selfTestNextFrameAtMs = now;  // 下一次 loop 立刻画第一帧
}

// 自检期间的帧推进 + 到期自动退出。用有符号差值比较，规避 millis() 溢出。
void pollSelfTest() {
  if (selfTestEndAtMs == 0) return;
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - selfTestEndAtMs) >= 0) {
    selfTestEndAtMs = 0;
    Serial.println("[screen] self test done -> back to status page");
    drawFrame();
    return;
  }
  if (static_cast<int32_t>(now - selfTestNextFrameAtMs) >= 0) {
    drawSelfTestFrame(selfTestFrameIndex);
    selfTestFrameIndex = (selfTestFrameIndex + 1) % kSelfTestFrames;
    selfTestNextFrameAtMs = now + kSelfTestFrameMs;
  }
}

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
  if (head == "t") {
    beginSelfTest();
    return;
  }
  if (head == "k") {
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
  Serial.println("Codex Micro / Waveshare port / step2 display");
  Serial.printf("chip=%s rev=%u cores=%u flash=%uKB psram=%uKB\n",
                ESP.getChipModel(), static_cast<unsigned>(ESP.getChipRevision()),
                static_cast<unsigned>(ESP.getChipCores()),
                static_cast<unsigned>(ESP.getFlashChipSize() / 1024),
                static_cast<unsigned>(ESP.getPsramSize() / 1024));

  pinMode(kButtonPin, INPUT_PULLUP);

  // 屏先起来：如果这里失败，串口是唯一的输出通道，所以先打日志再决定。
  const uint32_t t0 = millis();
  if (!display.begin()) {
    Serial.println("[screen] 初始化失败，继续跑 BLE（只能用串口）");
  } else {
    Serial.printf("[screen] CO5300 就绪 %dx%d canvas=%d 耗时=%lums\n",
                  BoardDisplay::kWidth, BoardDisplay::kHeight,
                  display.doubleBuffered() ? 1 : 0,
                  static_cast<unsigned long>(millis() - t0));
  }

  ble.begin();
  Serial.printf("BLE stack=%s hid_handle=%u (NimBLE 下恒 65535，无意义)\n",
                ble.stackName(), static_cast<unsigned>(ble.hidServiceHandle()));

  drawFrame();
  printHelp();
  lastStatusAtMs = millis();
}

void loop() {
  ble.poll();

  String line;
  if (readLine(line)) handleCommand(line);
  handleButton();

  const uint32_t now = millis();
  if (now - lastAdvCheckAtMs >= kAdvCheckIntervalMs) {
    lastAdvCheckAtMs = now;
    ensureAdvertising();
  }
  if (now - lastStatusAtMs >= kStatusIntervalMs) {
    lastStatusAtMs = now;
    printStatus();
  }
  // 自检期间暂停常规刷新，否则 500ms 一次的状态页会把自检帧盖掉。
  pollSelfTest();
  if (selfTestEndAtMs == 0 && now - lastRedrawAtMs >= kRedrawIntervalMs) {
    drawFrame();
  }
  delay(10);
}
