// SPDX-License-Identifier: MIT
// 板级音频层：Waveshare ESP32-S3-Touch-AMOLED-1.8 (V2) 上的 ES8311 + 板载喇叭。
//
// 引脚全部来自 PocketOracle（微雪官方 Arduino-v3.3.5-v2 的 pin_config.h，
// 且那份代码在这块板上放网络电台是实测通过的）：
//   ESP DOUT(8) -> ES8311 SDIN     ESP DIN(10) <- ES8311 ASDOUT（麦克风，本步不用）
//   BCK(9) WS/LRCK(45) MCK(16)     PA 功放使能 = GPIO46（高电平开）
//
// 第 ⑤ 步只做一件事：**在某个 Agent 跑完时响一声**（上游的完成提示音）。
// 所以这里不引入 ESP32-audioI2S，只用 ESP-IDF 的 I2S 标准驱动 + 直接写 PCM，
// 代码量与依赖都最小。
//
// 【与上游的区别】上游用 M5Unified 的 M5.Speaker.playRaw()（异步播放）。
// 本板没有 M5Unified，改成 i2s_channel_write 阻塞写，按 256 帧一块流式喂，
// **不预分配整段 PCM 缓冲**（12k 采样的上游做法在 48k 下要 52KB，没必要）。
// 单次提示音总阻塞约 270ms，对 BLE 无影响（step2 实测 500ms 全屏 flush 都不掉包）。
//
// 【ES8311 寄存器序列】照搬 PocketOracle 的 es8311.cpp（那份是这块板验证过的）
// 的 48kHz / 16bit 分支，只把 DAC 音量单独设成 0xE0。
// MCLK = 48000 × 256 = 12.288MHz，而 I2S_STD_CLK_DEFAULT_CONFIG 的
// mclk_multiple 默认就是 256，两边对得上。

#pragma once

#include <Arduino.h>

class BoardAudio {
 public:
  // 与触摸/PMU 同一条 I2C
  static constexpr int8_t kSda = 15;
  static constexpr int8_t kScl = 14;
  static constexpr uint8_t kCodecAddr = 0x18;

  // I2S / 功放引脚（实测值，别照抄 variant）
  static constexpr int8_t kMclkPin = 16;
  static constexpr int8_t kBclkPin = 9;
  static constexpr int8_t kWsPin = 45;
  static constexpr int8_t kDoutPin = 8;
  static constexpr int8_t kDinPin = 10;
  static constexpr int8_t kAmpPin = 46;

  static constexpr uint32_t kSampleRate = 48000;
  // 播放结束后多久关功放（给 I2S DMA 排空留余量，否则尾音被切、还会"啪"一声）
  static constexpr uint32_t kAmpTailMs = 150;
  // 每次写进 I2S 的帧数（立体声，所以缓冲是 2 倍）
  static constexpr size_t kChunkFrames = 256;

  // 初始化 I2S + ES8311。返回 true = 编解码器答话了。
  // I2S 起不来或 codec 不应答时返回 false，但**不影响看板**（只是没声音）。
  // ⚠️ 必须在 BoardTouch::begin() 之后调用（Wire 由它起来）。
  bool begin();

  bool codecReady() const { return codecOk_; }
  bool i2sReady() const { return i2sOk_; }

  // ---- 编解码器寄存器通道（排查用） ----
  int32_t readCodecReg(uint8_t reg);  // -1 = 无应答
  bool writeCodecReg(uint8_t reg, uint8_t value);

  // ---- 功放 ----
  void setAmp(bool on);
  bool ampOn() const { return ampOn_; }

  // ---- 播放 ----
  // loop 里调：负责到点关功放。不调的话功放就一直开着（有底噪、费电）。
  void poll();

  void playChime();  // 上游同款完成音：A4 -> C#5，270ms
  // 测试音：单频正弦 + 淡入淡出。用来把"喇叭/功放坏了"和"提示音路径坏了"分开。
  void playTone(float freqHz, uint32_t durationMs, float amplitude);

 private:
  bool codecOk_ = false;
  bool i2sOk_ = false;
  bool ampOn_ = false;
  uint32_t ampOffAtMs_ = 0;

  bool setupI2s();
  bool setupCodec();
  bool writeCodecRegChecked(uint8_t reg, uint8_t value, bool* ok);
  // 低层：把 mono 采样值复制成立体声写进 I2S（分块，不预分配）
  void writeMono(const float* samples, size_t frames);
};
