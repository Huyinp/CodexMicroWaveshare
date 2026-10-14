// Codex Micro —— Waveshare 移植 step3：BLE 协议层 + CO5300 屏 + CST820 触摸
//
// 复刻顺序的第 ③ 步。这一版**刻意不做任何坐标变换**，唯一目的是把触摸的
// 三个未知量测出来，然后再谈映射：
//   1) 有效量程  —— 屏幕上的绘图区就是 raw 坐标系 1:1，点四角看 min/max
//   2) 镜像方向  —— 物理左上角对应绘图区的哪个角
//   3) INT 是否在动 —— irq 计数，配合 100ms 兜底轮询双路兜底
//
// 上游 1.75C 的 CST9217 用 (232-raw)*2，那是 232 量程的产物；
// 本板是同尺寸不同后缀的 CST820，量程/镜像**不能互推**。
//
// 界面：全屏网格 = raw 坐标系统，原点就是屏幕左上角，raw 直接当屏幕坐标画。
// 顶部信息条压在上面，串口同步打日志。

#include <Arduino.h>
#include <BLEDevice.h>

#include "BoardDisplay.h"
#include "BoardTouch.h"
#include "CodexMicroBle.h"

namespace {

// Waveshare ESP32-S3-Touch-AMOLED-1.8 的 BOOT 键接 GPIO0（低电平按下）。
constexpr uint8_t kButtonPin = 0;
constexpr uint32_t kStatusIntervalMs = 5000;
constexpr uint32_t kAdvCheckIntervalMs = 3000;
constexpr uint32_t kTouchRedrawMinMs = 50;  // 全屏重绘节流，别让刷屏饿死 BLE

BoardDisplay display;
BoardTouch touch;
CodexMicroBle ble;

uint32_t lastStatusAtMs = 0;
uint32_t lastAdvCheckAtMs = 0;
bool lastButtonPressed = false;
uint32_t buttonPressAtMs = 0;
uint32_t touchRedraws = 0;

// 串口日志节流：坐标要挪够这么远才再打一行，不然按住不动会刷屏。
constexpr int16_t kLogMovePx = 8;
int16_t lastLogX = 0;
int16_t lastLogY = 0;
bool lastLogDown = false;

// ---------------- 原始寄存器监视（诊断通道） ----------------
// 每 kRawPollMs 直接把 0x01..0x06 读一遍。只要有任何一格非零就说明
// 芯片在报触摸 —— 这是把"没人点"和"点了但驱动读不出来"分开的唯一手段。
constexpr uint32_t kRawPollMs = 150;
constexpr uint32_t kRawHeartbeatMs = 2000;  // 全零时也每 2 秒报一次，证明总线活着

uint8_t rawRegs[6] = {0};   // 0x01..0x06
bool rawReadOk = false;
uint32_t rawNonZeroCount = 0;   // 累计见到非零块的次数
uint32_t rawPollCount = 0;
uint32_t rawLastPollMs = 0;
uint32_t rawLastHeartbeatMs = 0;
uint8_t rawLastKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
bool rawDirty = false;  // 原始值变了 -> 让屏幕也重画一次

bool rawBlockActive(const uint8_t* r) {
  // 0x02 的**低四位**是手指数：有些固件在高四位塞了额外信息，
  // 驱动是整字节 switch 的，所以会漏。
  return (r[1] & 0x0F) != 0 || r[2] != 0 || r[3] != 0 || r[4] != 0 ||
         r[5] != 0 || (r[0] & 0x0F) != 0;
}

void printRawBlock(const char* tag) {
  Serial.printf(
      "[%s] 01=%02X 02=%02X 03=%02X 04=%02X 05=%02X 06=%02X | gpio21=%u "
      "irq=%lu nz=%lu reads=%lu ok=%d\n",
      tag, rawRegs[0], rawRegs[1], rawRegs[2], rawRegs[3], rawRegs[4],
      rawRegs[5], static_cast<unsigned>(touch.intLevel()),
      static_cast<unsigned long>(touch.irqCount()),
      static_cast<unsigned long>(rawNonZeroCount),
      static_cast<unsigned long>(rawPollCount), rawReadOk ? 1 : 0);
}

void pollRawTouch() {
  const uint32_t now = millis();
  if (now - rawLastPollMs < kRawPollMs) return;
  rawLastPollMs = now;

  rawReadOk = touch.readRegs(0x01, rawRegs, sizeof(rawRegs));
  ++rawPollCount;

  if (!rawReadOk) {
    // 总线层面就失败（跟"没手指"是两码事）
    if (rawLastKey[0] != 0xEE) {
      rawLastKey[0] = 0xEE;
      Serial.printf("[traw] I2C 读 0x15 失败（第 %lu 次）\n",
                    static_cast<unsigned long>(rawPollCount));
    }
    return;
  }

  const bool active = rawBlockActive(rawRegs);
  const bool changed = memcmp(rawRegs, rawLastKey, sizeof(rawRegs)) != 0;
  memcpy(rawLastKey, rawRegs, sizeof(rawRegs));
  if (changed || active) rawDirty = true;

  if (active) {
    ++rawNonZeroCount;
    // 值一变就打（滑动时每步都看得到），值不变就 500ms 打一次心跳
    static uint32_t lastSameMs = 0;
    if (changed || now - lastSameMs >= 500) {
      lastSameMs = now;
      printRawBlock("traw+");
    }
    return;
  }

  // 全零：2 秒一次心跳，证明"读得到但确实没人碰"
  if (now - rawLastHeartbeatMs >= kRawHeartbeatMs) {
    rawLastHeartbeatMs = now;
    rawDirty = true;
    printRawBlock("traw");
  }
}

// 轨迹环形缓冲：最近 kTrailMax 个点，用来一眼看出滑动方向。
constexpr int kTrailMax = 96;
int16_t trailX[kTrailMax];
int16_t trailY[kTrailMax];
int trailCount = 0;
int trailHead = 0;

constexpr uint16_t kColBg = 0x0841;
constexpr uint16_t kColPanel = 0x18E3;
constexpr uint16_t kColTitleBg = 0x01A7;
constexpr uint16_t kColGrid = 0x2104;
constexpr uint16_t kColGridMajor = 0x4208;
constexpr uint16_t kColText = RGB565_WHITE;
constexpr uint16_t kColDim = RGB565_DARKGRAY;
constexpr uint16_t kColOk = RGB565_GREEN;
constexpr uint16_t kColWarn = RGB565_YELLOW;
constexpr uint16_t kColOff = RGB565_RED;
constexpr uint16_t kColAccent = RGB565_CYAN;

constexpr int16_t kInfoTop = 182;  // 绘图区从这条线往下（信息面板放大后变高了）
// 信息面板 6 行，字号统一 2（12x16 点阵），基线间距 22。
constexpr int16_t kInfoLineY[6] = {48, 70, 92, 114, 136, 158};
constexpr uint8_t kInfoTextSize = 2;

void textAt(const char* s, int16_t x, int16_t y, uint8_t size, uint16_t fg,
            uint16_t bg) {
  Arduino_GFX* g = display.gfx();
  g->setTextColor(fg, bg);
  g->setTextSize(size);
  g->setCursor(x, y);
  g->print(s);
}

void pushTrail(int16_t x, int16_t y) {
  trailX[trailHead] = x;
  trailY[trailHead] = y;
  trailHead = (trailHead + 1) % kTrailMax;
  if (trailCount < kTrailMax) ++trailCount;
}

void clearTrail() {
  trailCount = 0;
  trailHead = 0;
}

void drawGrid() {
  Arduino_GFX* g = display.gfx();
  for (int16_t x = 0; x < BoardDisplay::kWidth; x += 50) {
    g->drawFastVLine(x, 0, BoardDisplay::kHeight,
                     (x % 100 == 0) ? kColGridMajor : kColGrid);
  }
  for (int16_t y = 0; y < BoardDisplay::kHeight; y += 50) {
    g->drawFastHLine(0, y, BoardDisplay::kWidth,
                     (y % 100 == 0) ? kColGridMajor : kColGrid);
  }
  // 刻度数字：让"raw 数值"和"屏幕位置"能直接对上。字号 2，只在 100/200/300
  // 这几条主网格线上打，免得挤在一起。
  char buf[8];
  for (int16_t x = 100; x < BoardDisplay::kWidth; x += 100) {
    snprintf(buf, sizeof(buf), "%d", x);
    textAt(buf, x + 3, kInfoTop + 20, kInfoTextSize, kColDim, kColBg);
  }
  for (int16_t y = 100; y < BoardDisplay::kHeight; y += 100) {
    snprintf(buf, sizeof(buf), "%d", y);
    textAt(buf, 3, y + 22, kInfoTextSize, kColDim, kColBg);
  }
}

// 绘图区四角标记（屏幕坐标系）。点物理左上角时，看亮点落在哪个色块旁，
// 就知道 raw 轴跟屏幕轴是什么关系。方块放大到 24px，好点也好认。
void drawCornerMarks() {
  Arduino_GFX* g = display.gfx();
  constexpr int16_t m = 24;
  constexpr int16_t pad = 8;
  g->fillRect(pad, kInfoTop + pad, m, m, RGB565_RED);  // 屏左上
  g->fillRect(BoardDisplay::kWidth - pad - m, kInfoTop + pad, m, m,
              RGB565_GREEN);  // 屏右上
  g->fillRect(pad, BoardDisplay::kHeight - pad - m, m, m, RGB565_BLUE);  // 屏左下
  g->fillRect(BoardDisplay::kWidth - pad - m, BoardDisplay::kHeight - pad - m,
              m, m, RGB565_YELLOW);  // 屏右下
}

void drawTrail() {
  Arduino_GFX* g = display.gfx();
  for (int i = 0; i < trailCount; ++i) {
    const int idx = (trailHead - 1 - i + kTrailMax * 2) % kTrailMax;
    const int16_t x = trailX[idx];
    const int16_t y = trailY[idx];
    if (x < 0 || y < 0 || x >= BoardDisplay::kWidth ||
        y >= BoardDisplay::kHeight) {
      continue;
    }
    // 越新的点越亮
    const uint16_t c = (i < 8) ? kColWarn : ((i < 32) ? 0x6B4D : 0x39E7);
    g->fillRect(x - 1, y - 1, 3, 3, c);
  }
}

void drawInfo() {
  Arduino_GFX* g = display.gfx();
  const CodexMicroState st = ble.snapshot();

  // 标题条
  g->fillRect(0, 0, BoardDisplay::kWidth, 40, kColTitleBg);
  textAt("RAW TOUCH", 10, 30, 3, kColText, kColTitleBg);
  textAt("step3", BoardDisplay::kWidth - 10 - 5 * 12, 26, 2, kColAccent,
         kColTitleBg);

  g->fillRect(0, 40, BoardDisplay::kWidth, kInfoTop - 40, kColPanel);

  char line[64];
  // 字号 2 = 12x16 点阵，一行最多 30 个字符（368/12）。

  // ① 芯片状态
  snprintf(line, sizeof(line), "CST820 ok=%d id=%02X",
           touch.ready() ? 1 : 0, touch.deviceId() < 0 ? 0 : touch.deviceId());
  textAt(line, 8, kInfoLineY[0], kInfoTextSize,
         touch.ready() ? kColOk : kColOff, kColPanel);

  // ② 当前点
  if (touch.pressed()) {
    snprintf(line, sizeof(line), "now= %4d, %4d", touch.x(), touch.y());
    textAt(line, 8, kInfoLineY[1], kInfoTextSize, kColWarn, kColPanel);
  } else {
    snprintf(line, sizeof(line), "now= ----, ----  up");
    textAt(line, 8, kInfoLineY[1], kInfoTextSize, kColDim, kColPanel);
  }

  // ③ 实测量程 —— 这一行就是"要报给我的四个数"
  if (touch.samples() > 0) {
    snprintf(line, sizeof(line), "min= %d,%d  max= %d,%d", touch.minX(),
             touch.minY(), touch.maxX(), touch.maxY());
  } else {
    snprintf(line, sizeof(line), "min/max: tap 4 corners");
  }
  textAt(line, 8, kInfoLineY[2], kInfoTextSize, kColAccent, kColPanel);

  // ④ 计数
  snprintf(line, sizeof(line), "irq=%lu smp=%lu int=%u",
           static_cast<unsigned long>(touch.irqCount()),
           static_cast<unsigned long>(touch.samples()),
           static_cast<unsigned>(touch.intLevel()));
  textAt(line, 8, kInfoLineY[3], kInfoTextSize, kColText, kColPanel);

  // ⑤⑥ 原始寄存器（诊断用）—— 点屏幕时这两行如果动，说明芯片在报触摸
  if (rawReadOk) {
    const uint16_t c = rawBlockActive(rawRegs) ? kColWarn : kColDim;
    snprintf(line, sizeof(line), "02=%02X 03=%02X 04=%02X", rawRegs[1],
             rawRegs[2], rawRegs[3]);
    textAt(line, 8, kInfoLineY[4], kInfoTextSize, c, kColPanel);
    snprintf(line, sizeof(line), "05=%02X 06=%02X nz=%lu", rawRegs[4],
             rawRegs[5], static_cast<unsigned long>(rawNonZeroCount));
    textAt(line, 8, kInfoLineY[5], kInfoTextSize, c, kColPanel);
  } else {
    textAt("REG READ FAIL 0x15", 8, kInfoLineY[4], kInfoTextSize, kColOff,
           kColPanel);
  }

  // BLE 状态挪到绘图区左上角的空白处，用不着占面板的高度
  snprintf(line, sizeof(line), "ble=%d rpc=%d", st.connected ? 1 : 0,
           st.hostRpcObserved ? 1 : 0);
  textAt(line, BoardDisplay::kWidth - 12 * 13, kInfoTop + 20, kInfoTextSize,
         kColDim, kColBg);
}

// raw 直接当屏幕坐标画（1:1，不缩放不镜像）。越界就夹到边内 ——
// "夹到边界"本身就是"raw 量程比屏幕大"的直接证据，配合顶部真实数值一起看。
void drawTouchPoint(int16_t x, int16_t y) {
  Arduino_GFX* g = display.gfx();
  int16_t px = x < 0 ? 0 : (x >= BoardDisplay::kWidth ? BoardDisplay::kWidth - 1 : x);
  int16_t py = y < 0 ? 0 : (y >= BoardDisplay::kHeight ? BoardDisplay::kHeight - 1 : y);

  g->drawFastVLine(px, kInfoTop, BoardDisplay::kHeight - kInfoTop, kColWarn);
  g->drawFastHLine(0, py, BoardDisplay::kWidth, kColWarn);
  g->fillCircle(px, py, 10, kColText);
  g->drawCircle(px, py, 14, kColOff);
  g->drawCircle(px, py, 15, kColOff);
}

void drawTouchPage() {
  Arduino_GFX* g = display.gfx();
  g->fillScreen(kColBg);
  drawGrid();
  drawCornerMarks();
  drawTrail();
  drawInfo();
  if (touch.samples() > 0) drawTouchPoint(touch.x(), touch.y());
  display.flush();
  ++touchRedraws;
}

// ---------------- 屏自检（同 step2 的驻留循环版） ----------------
constexpr uint32_t kSelfTestFrameMs = 1500;
constexpr int kSelfTestFrames = 4;
constexpr int kSelfTestRounds = 2;
constexpr uint32_t kSelfTestTotalMs =
    kSelfTestFrameMs * kSelfTestFrames * kSelfTestRounds;

uint32_t selfTestEndAtMs = 0;
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
      const uint16_t bars[] = {RGB565_RED,   RGB565_GREEN, RGB565_BLUE,
                               RGB565_WHITE, RGB565_BLACK, RGB565_YELLOW,
                               RGB565_CYAN,  RGB565_MAGENTA};
      const int16_t h = BoardDisplay::kHeight / 8;
      for (int i = 0; i < 8; ++i) {
        g->fillRect(0, i * h, BoardDisplay::kWidth, h, bars[i]);
      }
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
  selfTestNextFrameAtMs = now;
}

void pollSelfTest() {
  if (selfTestEndAtMs == 0) return;
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - selfTestEndAtMs) >= 0) {
    selfTestEndAtMs = 0;
    Serial.println("[screen] self test done -> back to touch page");
    drawTouchPage();
    return;
  }
  if (static_cast<int32_t>(now - selfTestNextFrameAtMs) >= 0) {
    drawSelfTestFrame(selfTestFrameIndex);
    selfTestFrameIndex = (selfTestFrameIndex + 1) % kSelfTestFrames;
    selfTestNextFrameAtMs = now + kSelfTestFrameMs;
  }
}

// ---------------- 触摸 ----------------
void handleTouchSample(const BoardTouchSample& s) {
  // 状态或位移够大才打日志，不然按住不动会刷屏
  const int16_t dx = static_cast<int16_t>(abs(s.x - lastLogX));
  const int16_t dy = static_cast<int16_t>(abs(s.y - lastLogY));
  const bool stateChanged = (s.down != lastLogDown);
  if (stateChanged || (s.down && (dx >= kLogMovePx || dy >= kLogMovePx))) {
    Serial.printf("[tp] %s raw=(%d, %d) f=%u irq=%lu\n",
                  s.down ? (stateChanged ? "down" : "move") : "up",
                  s.x, s.y, static_cast<unsigned>(s.fingers),
                  static_cast<unsigned long>(touch.irqCount()));
    lastLogX = s.x;
    lastLogY = s.y;
    lastLogDown = s.down;
  }

  if (s.down) pushTrail(s.x, s.y);
}

// NimBLE 兜底：断开后协议栈不一定可靠地恢复广播。
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
  Serial.println("== Codex Micro step3 (BLE + CO5300 + CST820) ==");
  Serial.println("k <KEY> [act] [agent]  发按键，例如: k ACT10 1");
  Serial.println("j <angle> <distance>   发摇杆，例如: j 0.75 1.0");
  Serial.println("b <0..100>             设置电量百分比");
  Serial.println("s                      打印状态");
  Serial.println("t                      屏自检（红/绿/蓝/彩条，驻留 12s）");
  Serial.println("r                      清零触摸量程统计(min/max)与轨迹");
  Serial.println("d                      转储触摸原始寄存器 + 驱动解码值（诊断）");
  Serial.println("rd <reg_hex>           读触摸单个寄存器，如: rd fa");
  Serial.println("wr <reg_hex> <val_hex> 写触摸寄存器并回读，如: wr fa 40");
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
      "[status] connected=%d peers=%d adv=%d rpc=%d quota=%s heap=%u "
      "tp=%d id=0x%02X irq=%lu smp=%lu raw=(%d,%d) down=%d "
      "min=(%d,%d) max=(%d,%d)\n",
      state.connected ? 1 : 0, peers, advertising ? 1 : 0,
      state.hostRpcObserved ? 1 : 0, state.quota.available ? "yes" : "no",
      static_cast<unsigned>(ESP.getFreeHeap()), touch.ready() ? 1 : 0,
      touch.deviceId() < 0 ? 0 : touch.deviceId(),
      static_cast<unsigned long>(touch.irqCount()),
      static_cast<unsigned long>(touch.samples()), touch.x(), touch.y(),
      touch.pressed() ? 1 : 0, touch.minX(), touch.minY(), touch.maxX(),
      touch.maxY());
  Serial.printf(
      "[status] regs 01=%02X 02=%02X 03=%02X 04=%02X 05=%02X 06=%02X "
      "gpio21=%u nz=%lu reads=%lu ok=%d\n",
      rawRegs[0], rawRegs[1], rawRegs[2], rawRegs[3], rawRegs[4], rawRegs[5],
      static_cast<unsigned>(touch.intLevel()),
      static_cast<unsigned long>(rawNonZeroCount),
      static_cast<unsigned long>(rawPollCount), rawReadOk ? 1 : 0);
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
  if (head == "r") {
    touch.resetRange();
    clearTrail();
    Serial.println("[tp] range/trail reset");
    drawTouchPage();
    return;
  }
  if (head == "d") {
    // 一次性完整转储：把"驱动解码值"和"原始寄存器"并排打出来。
    // 两者对不上（原始非零、驱动报 -1）就是驱动的解码问题了。
    uint8_t b[6] = {0};
    const bool ok = touch.readRegs(0x01, b, sizeof(b));
    Serial.println("[tdump] ---- CST820 @0x15 ----");
    Serial.printf("[tdump] read 0x01..0x06 = %s\n", ok ? "OK" : "FAIL");
    Serial.printf("[tdump]  0x01 GestureID = 0x%02X\n", b[0]);
    Serial.printf("[tdump]  0x02 FingerNum = 0x%02X (低4位=%u)\n", b[1],
                  static_cast<unsigned>(b[1] & 0x0F));
    Serial.printf("[tdump]  X = 0x%02X%02X (高4位掩码后 %u)\n", b[2], b[3],
                  static_cast<unsigned>(((b[2] & 0x0F) << 8) | b[3]));
    Serial.printf("[tdump]  Y = 0x%02X%02X (高4位掩码后 %u)\n", b[4], b[5],
                  static_cast<unsigned>(((b[4] & 0x0F) << 8) | b[5]));
    Serial.printf("[tdump] 驱动解码 finger=%d x=%d y=%d\n",
                  static_cast<int>(touch.driverValue(
                      Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER)),
                  static_cast<int>(touch.driverValue(
                      Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X)),
                  static_cast<int>(touch.driverValue(
                      Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y)));
    uint8_t id = 0, irc = 0, fe = 0, mtn = 0;
    const bool okId = touch.readRegs(0xA7, &id, 1);
    const bool okIr = touch.readRegs(0xFA, &irc, 1);
    const bool okFe = touch.readRegs(0xFE, &fe, 1);
    const bool okMt = touch.readRegs(0xE5, &mtn, 1);
    Serial.printf("[tdump] 0xA7 ChipID=%s0x%02X  0xFA IrqCtl=%s0x%02X  "
                  "0xFE=%s0x%02X  0xE5 Sleep=%s0x%02X\n",
                  okId ? "" : "?", id, okIr ? "" : "?", irc, okFe ? "" : "?", fe,
                  okMt ? "" : "?", mtn);
    Serial.printf("[tdump] gpio21=%u (1=空闲高, 0=芯片在拉低) irq=%lu\n",
                  static_cast<unsigned>(touch.intLevel()),
                  static_cast<unsigned long>(touch.irqCount()));
    Serial.println("[tdump] -----------------------");
    return;
  }
  if (head == "rd") {
    const long reg = strtol(rest.c_str(), nullptr, 16);
    uint8_t v = 0;
    if (touch.readRegs(static_cast<uint8_t>(reg), &v, 1)) {
      Serial.printf("[treg] 0x%02lX = 0x%02X (%u)\n", reg, v, v);
    } else {
      Serial.printf("[treg] 0x%02lX 读失败\n", reg);
    }
    return;
  }
  if (head == "wr") {
    const int sp = rest.indexOf(' ');
    if (sp < 0) {
      Serial.println("usage: wr <reg_hex> <val_hex>");
      return;
    }
    const long reg = strtol(rest.substring(0, sp).c_str(), nullptr, 16);
    const long val = strtol(rest.substring(sp + 1).c_str(), nullptr, 16);
    const bool ok = touch.writeReg(static_cast<uint8_t>(reg),
                                   static_cast<uint8_t>(val));
    uint8_t back = 0;
    const bool rb = touch.readRegs(static_cast<uint8_t>(reg), &back, 1);
    Serial.printf("[treg] write 0x%02lX = 0x%02lX -> %s, readback=%s0x%02X\n",
                  reg, val, ok ? "OK" : "FAIL", rb ? "" : "?", back);
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
  Serial.println("Codex Micro / Waveshare port / step3 touch");
  Serial.printf("chip=%s rev=%u cores=%u flash=%uKB psram=%uKB\n",
                ESP.getChipModel(), static_cast<unsigned>(ESP.getChipRevision()),
                static_cast<unsigned>(ESP.getChipCores()),
                static_cast<unsigned>(ESP.getFlashChipSize() / 1024),
                static_cast<unsigned>(ESP.getPsramSize() / 1024));

  pinMode(kButtonPin, INPUT_PULLUP);

  const uint32_t t0 = millis();
  if (!display.begin()) {
    Serial.println("[screen] 初始化失败，继续跑 BLE（只能用串口）");
  } else {
    Serial.printf("[screen] CO5300 就绪 %dx%d canvas=%d 耗时=%lums\n",
                  BoardDisplay::kWidth, BoardDisplay::kHeight,
                  display.doubleBuffered() ? 1 : 0,
                  static_cast<unsigned long>(millis() - t0));
  }

  // 触摸在屏之后、BLE 之前：I2C 用的是触摸/PMU 共用那条总线，
  // 先把手感链路打通，串口上就能看到它到底成没成。
  const uint32_t t1 = millis();
  const bool tpOk = touch.begin();
  Serial.printf("[touch] CST820 begin=%d id=0x%02X int=GPIO%d sda=%d scl=%d "
                "耗时=%lums\n",
                tpOk ? 1 : 0, touch.deviceId() < 0 ? 0 : touch.deviceId(),
                BoardTouch::kIntPin, BoardTouch::kSda, BoardTouch::kScl,
                static_cast<unsigned long>(millis() - t1));

  ble.begin();
  Serial.printf("BLE stack=%s hid_handle=%u (NimBLE 下恒 65535，无意义)\n",
                ble.stackName(), static_cast<unsigned>(ble.hidServiceHandle()));

  drawTouchPage();
  printHelp();
  lastStatusAtMs = millis();
}

void loop() {
  ble.poll();

  String line;
  if (readLine(line)) handleCommand(line);
  handleButton();

  // ---- 触摸：采样 -> 打日志 -> 需要时重绘 ----
  touch.poll();
  pollRawTouch();

  static uint32_t lastRedrawAtMs = 0;
  static bool redrawPending = false;

  BoardTouchSample sample;
  while (touch.takeSample(sample)) {
    handleTouchSample(sample);
    redrawPending = true;
  }

  const uint32_t now = millis();
  if (now - lastAdvCheckAtMs >= kAdvCheckIntervalMs) {
    lastAdvCheckAtMs = now;
    ensureAdvertising();
  }
  if (now - lastStatusAtMs >= kStatusIntervalMs) {
    lastStatusAtMs = now;
    printStatus();
  }

  pollSelfTest();
  if (selfTestEndAtMs == 0 && (redrawPending || rawDirty) &&
      (now - lastRedrawAtMs) >= kTouchRedrawMinMs) {
    redrawPending = false;
    rawDirty = false;
    lastRedrawAtMs = now;
    drawTouchPage();
  }
  delay(5);
}
