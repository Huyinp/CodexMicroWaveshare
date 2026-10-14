// SPDX-License-Identifier: MIT
// 板级显示层：Waveshare ESP32-S3-Touch-AMOLED-1.8 (V2)
//   CO5300 AMOLED 368x448，QSPI 四线，16bpp，帧缓冲在 PSRAM。
//
// 只负责"把画好的帧缓冲推到屏上"，不含任何业务逻辑。
//
// 与上游的差别：上游 codex-micro-waveshare 用 M5GFX + M5Canvas（1.75C 板，
// CO5300 466x466）；本板改用 Arduino_GFX（GFX_Library_for_Arduino 自带
// Arduino_CO5300）。引脚与偏移量取自 PocketOracle 在同型板上的实测配置，
// 不是从 M5 或 1.75C 的 466x466 参数推出来的。

#pragma once

#include <Arduino.h>
#include <Arduino_GFX_Library.h>

class BoardDisplay {
 public:
  static constexpr int16_t kWidth = 368;
  static constexpr int16_t kHeight = 448;

  bool begin();

  // 绘制都落在帧缓冲上，画完必须调 flush() 才会出现在屏幕上。
  Arduino_GFX* gfx() { return gfx_; }

  void flush();
  void setBrightness(uint8_t brightness);

  // 帧缓冲是否真的分配成功。false 表示退回直写面板模式（会有闪烁）。
  bool doubleBuffered() const { return canvas_ != nullptr; }

 private:
  Arduino_DataBus* bus_ = nullptr;
  Arduino_CO5300* panel_ = nullptr;
  Arduino_Canvas* canvas_ = nullptr;
  Arduino_GFX* gfx_ = nullptr;
};
