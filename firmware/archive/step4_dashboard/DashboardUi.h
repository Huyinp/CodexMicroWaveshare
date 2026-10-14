// SPDX-License-Identifier: MIT
// 版式层 —— 上游 dashboard::DashboardUi.h（466x466）重排到本板 368x448。
//
// 重排的三条硬约束（根因都是"466 是正方形、368x448 不是"）：
//   1) 六键环不能再用正圆。水平半宽 184 < 垂直半高 224，正圆会让左右两键
//      顶到屏幕边缘；改成椭圆轨道 kOrbitX / kOrbitY。
//   2) 判"键不撞表盘"的判据是 中心↔键心距离 > 表盘外径 + 键半径。
//      上游 466：164 > 104+56=160（余 4）。本板：139 > 95+40=135（余 4）。
//   3) 上游的 SpaceMono VLW 字体在 Arduino_GFX 上没有对应实现（VLW 是
//      TFT_eSPI / M5GFX 专用格式）。这里改用 GFX 内置点阵字体 + setTextSize，
//      每行的字号按"表盘内圆在该行的可用宽度"反推，不是照抄上游的 18/46px。
//
// 与上游的其余差别：
//   - fillSmoothCircle / fillSmoothRoundRect → fillCircle / fillRoundRect
//     （M5GFX 的抗锯齿版本 Arduino_GFX 没有；AMOLED 上硬边圆反而更利）。
//   - fillArc 的角度基准实测与 M5GFX 一致（0° 在 12 点、顺时针），
//     所以进度环那一段可以原样照搬，不做补偿。
//   - 颜色常量照搬上游的 RGB565 值，两边都是标准 5-6-5，无需转换。

#pragma once

#include <Arduino_GFX.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace dashboard {

// ---------------- 几何 ----------------
constexpr int kWidth = 368;
constexpr int kHeight = 448;
constexpr int kCenterX = 184;
constexpr int kQuotaCenterY = 224;

constexpr int kAgentRadius = 40;
constexpr int kDialOuterRadius = 95;
constexpr int kDialInnerRadius = 86;
constexpr int kSendRadius = 95;  // 整块表盘都是 Send 键

// 椭圆轨道：水平受限（368），竖直可放大（448）。
constexpr int kOrbitX = 111;  // 295 - 184
constexpr int kOrbitY = 168;  // 224 - 56

// ---------------- 调色板（照搬上游，标准 RGB565） ----------------
constexpr uint16_t kBackground = 0x0000;    // VOID       #000000
constexpr uint16_t kPanel = 0x10C4;         // GRAPHITE   #111820
constexpr uint16_t kPanelPressed = 0x1946;  // DEEP STEEL #1D2935
constexpr uint16_t kText = 0xEF7E;          // ICE        #E8EEF5
constexpr uint16_t kMuted = 0x7C53;         // STEEL      #7D8A98
constexpr uint16_t kTrack = 0x2187;         // TRACK      #26323D
constexpr uint16_t kAccent = 0x16B6;        // SIGNAL     #12D6B2
constexpr uint16_t kVoice = 0x5F17;         // VOICE      #5BE0BD
constexpr uint16_t kWarning = 0xFDA9;       // AMBER      #FFB44A
constexpr uint16_t kDanger = 0xFAED;        // CORAL      #FF5D6C

enum class AgentStatus : std::uint8_t {
  Unassigned,
  Idle,
  Thinking,
  Complete,
  RequiresInput,
  Error,
  Active,
};

enum class LinkHealth : std::uint8_t {
  Offline,
  BleOnly,
  CodexLive,
};

enum class PowerOverlay : std::uint8_t {
  None,
  HoldToPowerOff,
  PoweringOff,
};

struct ThreadVisual {
  std::uint32_t color = 0;
  float brightness = 0.0f;
  bool focused = false;
};

struct Point {
  std::int16_t x;
  std::int16_t y;
};

struct State {
  std::array<ThreadVisual, 6> threads = {};
  LinkHealth linkHealth = LinkHealth::Offline;
  std::int8_t batteryPercent = -1;
  bool charging = false;
  bool docked = false;
  bool quotaAvailable = false;
  bool quotaStale = false;
  float remainingPercent = 0.0f;
  std::uint32_t resetInSeconds = 0;
  bool leftPressed = false;
  bool rightPressed = false;
  bool sendPressed = false;
  int activeTouchAgent = -1;
  int swipeDirection = -1;
  int completedAgent = -1;
  PowerOverlay powerOverlay = PowerOverlay::None;
  float powerHoldProgress = 0.0f;
};

// 368x448 实际坐标（= 中心 + 椭圆轨道 60° 步进）。触摸判定和绘制共用这一份。
static constexpr std::array<Point, 6> kAgentCenters = {{
    {184, 56},   // A1 顶部
    {295, 140},  // A2 右上
    {295, 308},  // A3 右下
    {184, 392},  // A4 底部
    {73, 308},   // A5 左下
    {73, 140},   // A6 左上
}};

// ---------------- 语义层（与上游逐行一致） ----------------
// Agent 状态**只能**从 ChatGPT 下发的 RGB 阈值反推：没有独立的 done/run 字段。
inline AgentStatus classify(const ThreadVisual& light) {
  if (light.brightness <= 0.01f) return AgentStatus::Unassigned;

  const int red = (light.color >> 16) & 0xFF;
  const int green = (light.color >> 8) & 0xFF;
  const int blue = light.color & 0xFF;
  const int maximum = std::max(red, std::max(green, blue));
  const int minimum = std::min(red, std::min(green, blue));

  if (maximum - minimum < 48 && maximum > 150) return AgentStatus::Idle;
  if (green > red * 1.12f && green > blue * 1.15f) return AgentStatus::Complete;
  if (blue > red * 1.12f && blue > green * 1.05f) return AgentStatus::Thinking;
  if (red > 150 && green > 70 && green > blue * 1.45f) {
    return AgentStatus::RequiresInput;
  }
  if (red > green * 1.20f && red > blue * 1.20f) return AgentStatus::Error;
  return AgentStatus::Active;
}

// 状态名只给串口诊断用（`u` 命令）。屏幕上看不到这些词。
inline const char* agentStatusName(AgentStatus status) {
  switch (status) {
    case AgentStatus::Unassigned: return "unassigned";
    case AgentStatus::Idle: return "idle";
    case AgentStatus::Thinking: return "thinking";
    case AgentStatus::Complete: return "complete";
    case AgentStatus::RequiresInput: return "needs-input";
    case AgentStatus::Error: return "error";
    case AgentStatus::Active: return "active";
  }
  return "unknown";
}

inline const char* linkHealthLabel(LinkHealth health) {
  switch (health) {
    case LinkHealth::CodexLive: return "CODEX LIVE";
    case LinkHealth::BleOnly: return "BLE ONLY";
    case LinkHealth::Offline: return "OFFLINE";
  }
  return "OFFLINE";
}

inline std::uint16_t linkHealthColor(LinkHealth health) {
  switch (health) {
    case LinkHealth::CodexLive: return kVoice;
    case LinkHealth::BleOnly: return 0x4A5D;  // azure #4292F5 的 565
    case LinkHealth::Offline: return kDanger;
  }
  return kDanger;
}

inline void formatReset(std::uint32_t seconds, char* output, std::size_t size) {
  if (seconds == 0) {
    std::snprintf(output, size, "RESET --");
    return;
  }
  const std::uint32_t days = seconds / 86400;
  const std::uint32_t hours = (seconds % 86400) / 3600;
  const std::uint32_t minutes = (seconds % 3600) / 60;
  if (days > 0) {
    std::snprintf(output, size, "RESET %luD %02luH",
                  static_cast<unsigned long>(days),
                  static_cast<unsigned long>(hours));
  } else if (hours > 0) {
    std::snprintf(output, size, "RESET %luH %02luM",
                  static_cast<unsigned long>(hours),
                  static_cast<unsigned long>(minutes));
  } else {
    std::snprintf(output, size, "RESET %luM",
                  static_cast<unsigned long>(minutes));
  }
}

// RGB888 + 亮度 -> RGB565。手算，不依赖具体的 Surface 类型
// （上游那版要传 surface 调 color565，这里没有必要）。
inline std::uint16_t rgb565(std::uint32_t rgb, float brightness = 1.0f) {
  brightness = std::max(0.0f, std::min(1.0f, brightness));
  const auto r = static_cast<std::uint8_t>(((rgb >> 16) & 0xFF) * brightness);
  const auto g = static_cast<std::uint8_t>(((rgb >> 8) & 0xFF) * brightness);
  const auto b = static_cast<std::uint8_t>((rgb & 0xFF) * brightness);
  return static_cast<std::uint16_t>(((r & 0xF8) << 8) | ((g & 0xFC) << 3) |
                                    (b >> 3));
}

// 上游把 Mac 下发的原始 RGB 当"语义输入"（表示哪种状态），
// 显示色在这里重新配过，让六键落在同一色系里。未知色原样透传。
inline std::uint32_t statusFillRgb(AgentStatus status,
                                   const ThreadVisual& light) {
  switch (status) {
    case AgentStatus::Idle:          return 0xB7C2CD;  // quiet silver
    case AgentStatus::Thinking:      return 0x4292F5;  // azure
    case AgentStatus::Complete:      return 0x2BC96E;  // spring green
    case AgentStatus::RequiresInput: return 0xF7AC42;  // amber
    case AgentStatus::Error:         return 0xF55A68;  // coral
    default:                         return light.color;
  }
}

// ---------------- 文本 ----------------
// GFX 内置点阵字体每个字形步进 6 列、8 行，放大 size 倍即 6*size x 8*size。
// 没有 setTextDatum，所以宽度自己算，只做水平居中。
constexpr int kGlyphWidth = 6;
constexpr int kGlyphHeight = 8;

inline void textAt(Arduino_GFX* g, const char* text, int left, int top,
                   std::uint8_t size, std::uint16_t color) {
  g->setTextSize(size);
  g->setTextColor(color);
  g->setCursor(left, top);
  g->print(text);
}

inline void textCentered(Arduino_GFX* g, const char* text, int centerX, int top,
                         std::uint8_t size, std::uint16_t color) {
  const int width =
      static_cast<int>(std::strlen(text)) * kGlyphWidth * size;
  textAt(g, text, centerX - width / 2, top, size, color);
}

inline int textWidth(const char* text, std::uint8_t size) {
  return static_cast<int>(std::strlen(text)) * kGlyphWidth * size;
}

// ---------------- 绘制 ----------------
inline void drawAgentShapes(Arduino_GFX* g, const State& state) {
  for (int i = 0; i < 6; ++i) {
    const ThreadVisual& light = state.threads[i];
    const AgentStatus status = classify(light);
    const Point point = kAgentCenters[i];
    const bool pressed = state.activeTouchAgent == i;

    if (status == AgentStatus::Unassigned) {
      if (light.focused) {
        g->fillCircle(point.x, point.y, kAgentRadius + 4, kText);
      }
      g->fillCircle(point.x, point.y, kAgentRadius, kTrack);
      g->fillCircle(point.x, point.y, kAgentRadius - 2,
                    pressed ? kPanelPressed : kPanel);
      continue;
    }

    const bool emphasized = status == AgentStatus::Thinking ||
                            status == AgentStatus::Complete ||
                            status == AgentStatus::RequiresInput ||
                            status == AgentStatus::Error;
    const std::uint32_t fillRgb = statusFillRgb(status, light);
    if (emphasized) {
      g->fillCircle(point.x, point.y, kAgentRadius + 4,
                    rgb565(fillRgb, light.brightness * 0.16f));
    }

    // 焦点是 Codex 下发的实时信号，画成贴着键边缘的冰白环，中间不留黑缝。
    if (light.focused) {
      g->fillCircle(point.x, point.y, kAgentRadius + 4, kText);
    }

    // 实心键：整盘承载状态色，按下时整体压暗而不是换成面板色；
    // 外层同色但更暗的一圈让键像嵌进面板的实体按键。
    const float fillBrightness = light.brightness * (pressed ? 0.72f : 1.0f);
    g->fillCircle(point.x, point.y, kAgentRadius,
                  rgb565(fillRgb, fillBrightness * 0.55f));
    g->fillCircle(point.x, point.y, kAgentRadius - 4,
                  rgb565(fillRgb, fillBrightness));
  }
}

inline void drawDialShapes(Arduino_GFX* g, const State& state) {
  g->fillArc(kCenterX, kQuotaCenterY, kDialOuterRadius, kDialInnerRadius, 0,
             360, kTrack);

  if (state.quotaAvailable) {
    const float remaining =
        std::max(0.0f, std::min(100.0f, state.remainingPercent));
    const std::uint16_t color =
        state.quotaStale
            ? rgb565(0x745527)
            : (remaining > 20.0f ? kAccent : kWarning);
    g->fillArc(kCenterX, kQuotaCenterY, kDialOuterRadius, kDialInnerRadius, 0,
               remaining * 3.6f, color);
  }

  // 四道刻痕让进度环读起来像标定过的仪表。
  g->fillRect(kCenterX - 2, kQuotaCenterY - kDialOuterRadius, 4, 12,
              kBackground);
  g->fillRect(kCenterX + kDialInnerRadius - 2, kQuotaCenterY - 2, 12, 4,
              kBackground);
  g->fillRect(kCenterX - 2, kQuotaCenterY + kDialInnerRadius - 2, 4, 12,
              kBackground);
  g->fillRect(kCenterX - kDialOuterRadius, kQuotaCenterY - 2, 12, 4,
              kBackground);

  // 内圈细线让整块表盘看起来就是那颗 Send 键。
  g->fillCircle(kCenterX, kQuotaCenterY, kDialInnerRadius - 3, kTrack);
  g->fillCircle(kCenterX, kQuotaCenterY, kDialInnerRadius - 6,
                state.sendPressed ? kPanelPressed : kBackground);
}

inline void drawBattery(Arduino_GFX* g, const State& state) {
  constexpr int kWidth_ = 29;
  constexpr int kHeight_ = 15;
  const int x = kCenterX - 38;
  const int y = kQuotaCenterY + 50;
  const int battery =
      std::max(-1, std::min(100, static_cast<int>(state.batteryPercent)));
  const std::uint16_t color =
      battery < 0 ? kMuted
                  : (state.docked ? kAccent
                                  : (battery <= 20 ? kWarning : kText));

  g->fillRoundRect(x, y, kWidth_, kHeight_, 3, color);
  g->fillRoundRect(x + 2, y + 2, kWidth_ - 4, kHeight_ - 4, 2, kBackground);
  g->fillRoundRect(x + kWidth_, y + 4, 3, kHeight_ - 8, 1, color);
  if (battery > 0) {
    const int fillWidth = std::max(2, (kWidth_ - 6) * battery / 100);
    g->fillRoundRect(x + 3, y + 3, fillWidth, kHeight_ - 6, 1, color);
  }
  if (state.charging) {
    const int boltX = x + kWidth_ / 2;
    g->fillTriangle(boltX, y + 1, boltX - 5, y + 8, boltX - 1, y + 8, kText);
    g->fillTriangle(boltX - 1, y + 7, boltX + 4, y + 7, boltX - 3,
                    y + kHeight_ - 1, kText);
  }

  char label[12];
  if (battery < 0) {
    std::snprintf(label, sizeof(label), "--%%");
  } else {
    std::snprintf(label, sizeof(label), "%d%%", battery);
  }
  textAt(g, label, kCenterX + 18, y + 4, 1, color);
}

inline void drawTexts(Arduino_GFX* g, const State& state) {
  // 连结状态词就是状态灯：颜色 + 文案区分"Codex 真在对话""只有 BLE 链路"
  // "彻底断线"这三种。放在表盘内部，六个键就能占满边缘。
  textCentered(g, linkHealthLabel(state.linkHealth), kCenterX,
               kQuotaCenterY - 58, 2, linkHealthColor(state.linkHealth));

  const char* dialLabel =
      state.quotaStale
          ? "SYNC STALE"
          : (state.linkHealth == LinkHealth::Offline ? "WAITING CODEX"
                                                     : "LEFT");
  textCentered(g, dialLabel, kCenterX, kQuotaCenterY - 34, 2,
               state.quotaStale ? kWarning : kMuted);

  char reset[24];
  formatReset(state.resetInSeconds, reset, sizeof(reset));
  const char* resetLabel = state.quotaAvailable
                               ? reset
                               : (state.quotaStale ? "CHECK COMPANION"
                                                   : "NO QUOTA DATA");
  textCentered(g, resetLabel, kCenterX, kQuotaCenterY + 40, 1,
               state.quotaStale ? kMuted
                                : (state.quotaAvailable ? kText : kMuted));

  drawBattery(g, state);

  for (int i = 0; i < 6; ++i) {
    char label[4];
    std::snprintf(label, sizeof(label), "A%d", i + 1);
    const AgentStatus status = classify(state.threads[i]);
    // 亮底（银白/琥珀/绿）配深字，暗底配冰白 —— 可读性取决于填充色本身。
    std::uint16_t color = kMuted;
    if (status != AgentStatus::Unassigned) {
      const std::uint32_t fillRgb = statusFillRgb(status, state.threads[i]);
      const int r = static_cast<int>(((fillRgb >> 16) & 0xFF) *
                                     state.threads[i].brightness);
      const int gg = static_cast<int>(((fillRgb >> 8) & 0xFF) *
                                      state.threads[i].brightness);
      const int b = static_cast<int>((fillRgb & 0xFF) *
                                     state.threads[i].brightness);
      const int luminance = (r * 77 + gg * 150 + b * 29) >> 8;
      color = luminance > 150 ? rgb565(0x0D1218) : kText;
    }
    textCentered(g, label, kAgentCenters[i].x,
                 kAgentCenters[i].y - 4 * 2, 2, color);
  }
}

inline void drawQuotaValue(Arduino_GFX* g, const State& state) {
  char value[8];
  if (state.quotaAvailable) {
    std::snprintf(value, sizeof(value), "%.0f%%",
                  std::max(0.0f, std::min(100.0f, state.remainingPercent)));
    textCentered(g, value, kCenterX, kQuotaCenterY - 16, 4,
                 state.quotaStale ? kMuted : kText);
  } else {
    textCentered(g, "--", kCenterX, kQuotaCenterY - 16, 4, kMuted);
  }
}

inline void drawTransient(Arduino_GFX* g, const State& state) {
  const char* message = nullptr;
  std::uint16_t color = kVoice;
  char completed[12];
  if (state.sendPressed) {
    message = "SEND";
    color = kText;
  } else if (state.leftPressed) {
    message = "LISTENING";
    color = kAccent;
  } else if (state.rightPressed) {
    message = "VOICE CHAT";
  } else if (state.swipeDirection >= 0 && state.swipeDirection < 4) {
    static constexpr const char* kSwipeMessages[] = {
        "STICK / UP", "STICK / RIGHT", "STICK / DOWN", "STICK / LEFT"};
    message = kSwipeMessages[state.swipeDirection];
    color = kAccent;
  } else if (state.completedAgent >= 0) {
    std::snprintf(completed, sizeof(completed), "A%d DONE",
                  state.completedAgent + 1);
    message = completed;
  }
  if (message == nullptr) return;

  g->fillRoundRect(kCenterX - 100, kQuotaCenterY - 32, 200, 64, 18, color);
  g->fillRoundRect(kCenterX - 96, kQuotaCenterY - 28, 192, 56, 15, kPanel);
  textCentered(g, message, kCenterX, kQuotaCenterY - 8, 2, color);
}

inline void drawPowerOverlay(Arduino_GFX* g, const State& state) {
  if (state.powerOverlay == PowerOverlay::None) return;

  const bool confirming = state.powerOverlay == PowerOverlay::HoldToPowerOff;
  const char* heading = confirming ? "KEEP HOLDING" : "POWERING OFF";
  const char* detail = confirming ? "ALERTS PAUSE" : "RED OR USB WAKE";
  const std::uint16_t color = confirming ? kWarning : kDanger;

  g->fillCircle(kCenterX, kQuotaCenterY, kDialOuterRadius, kTrack);
  const float progress =
      confirming
          ? std::max(0.0f, std::min(1.0f, state.powerHoldProgress))
          : 1.0f;
  g->fillArc(kCenterX, kQuotaCenterY, kDialOuterRadius, kDialInnerRadius, 0,
             progress * 360.0f, color);
  g->fillCircle(kCenterX, kQuotaCenterY, kDialInnerRadius - 3, kPanel);
  textCentered(g, heading, kCenterX, kQuotaCenterY - 16, 1, color);
  textCentered(g, detail, kCenterX, kQuotaCenterY + 10, 1, kText);
}

inline void render(Arduino_GFX* g, const State& state) {
  g->fillScreen(kBackground);
  drawDialShapes(g, state);
  drawAgentShapes(g, state);
  drawTexts(g, state);
  drawQuotaValue(g, state);
  drawTransient(g, state);
  drawPowerOverlay(g, state);
}

// ---------------- 命中测试（与绘制共用同一份坐标） ----------------
inline int agentAtPoint(int x, int y) {
  for (int i = 0; i < 6; ++i) {
    const int dx = x - kAgentCenters[i].x;
    const int dy = y - kAgentCenters[i].y;
    if (dx * dx + dy * dy <= (kAgentRadius + 4) * (kAgentRadius + 4)) {
      return i;
    }
  }
  return -1;
}

inline bool sendAtPoint(int x, int y) {
  const int dx = x - kCenterX;
  const int dy = y - kQuotaCenterY;
  return dx * dx + dy * dy <= kSendRadius * kSendRadius;
}

}  // namespace dashboard
