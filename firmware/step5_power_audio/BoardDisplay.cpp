// SPDX-License-Identifier: MIT
#include "BoardDisplay.h"

namespace {

// CO5300 QSPI 引脚（Waveshare ESP32-S3-Touch-AMOLED-1.8 V2）
constexpr int8_t kLcdCs = 12;
constexpr int8_t kLcdSclk = 11;
constexpr int8_t kLcdSdio0 = 4;
constexpr int8_t kLcdSdio1 = 5;
constexpr int8_t kLcdSdio2 = 6;
constexpr int8_t kLcdSdio3 = 7;

// 面板有效区在控制器 480x480 里的列起点。同一颗 CO5300 在 1.75C 上是
// 466x466 / offset 0，1.8 是 368x448 / offset 16 —— 不能互相推，是实测值。
constexpr uint8_t kColOffset1 = 16;
constexpr uint8_t kRowOffset1 = 0;

constexpr uint8_t kBrightnessBoot = 200;

}  // namespace

bool BoardDisplay::begin() {
  bus_ = new Arduino_ESP32QSPI(kLcdCs, kLcdSclk, kLcdSdio0, kLcdSdio1,
                               kLcdSdio2, kLcdSdio3);
  panel_ = new Arduino_CO5300(bus_, GFX_NOT_DEFINED, 0, kWidth, kHeight,
                              kColOffset1, kRowOffset1, 0, 0);
  canvas_ = new Arduino_Canvas(kWidth, kHeight, panel_, 0, 0, 0);

  if (canvas_->begin()) {
    // Canvas::begin() 内部已经 begin 过 panel。368*448*2 = 330KB 的帧缓冲
    // 会落到 PSRAM（Arduino-ESP32 下 memset/alloc 对 >16KB 的请求优先 SPIRAM）。
    gfx_ = canvas_;
  } else {
    delete canvas_;
    canvas_ = nullptr;
    if (!panel_->begin()) {
      gfx_ = nullptr;
      return false;
    }
    gfx_ = panel_;
  }

  panel_->setBrightness(kBrightnessBoot);
  gfx_->fillScreen(RGB565_BLACK);
  flush();
  return true;
}

void BoardDisplay::flush() {
  if (canvas_ != nullptr) {
    canvas_->flush();
  }
}

void BoardDisplay::setBrightness(uint8_t brightness) {
  if (panel_ != nullptr) {
    panel_->setBrightness(brightness);
  }
}
