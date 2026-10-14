// SPDX-License-Identifier: MIT
// 见 BoardAudio.h 的说明。

#include "BoardAudio.h"

#include <Wire.h>
#include <driver/i2s_std.h>

namespace {

i2s_chan_handle_t s_tx = nullptr;
// 立体声块缓冲（mono 内容复制到左右两声道）。1KB，放 .bss。
int16_t s_stereo[BoardAudio::kChunkFrames * 2];

constexpr float kPi = 3.14159265358979323846f;

// 上游 main.cpp 的 softNoteEnvelope()：38ms 淡入 + 100ms 淡出，两端都用
// sin/cos 的平方做平滑，避免方波边沿那种"哒"声。
float softNoteEnvelope(float noteTime, float noteDuration) {
  constexpr float kAttackSeconds = 0.038f;
  constexpr float kReleaseSeconds = 0.100f;
  if (noteTime < 0.0f || noteTime >= noteDuration) return 0.0f;

  float envelope = 1.0f;
  if (noteTime < kAttackSeconds) {
    const float phase = noteTime / kAttackSeconds;
    const float fade = sinf(phase * kPi * 0.5f);
    envelope *= fade * fade;
  }
  const float releaseStart = noteDuration - kReleaseSeconds;
  if (noteTime > releaseStart) {
    const float phase = (noteTime - releaseStart) / kReleaseSeconds;
    const float fade = cosf(phase * kPi * 0.5f);
    envelope *= fade * fade;
  }
  return envelope;
}

}  // namespace

// ---------------------------------------------------------------- I2C 通道

int32_t BoardAudio::readCodecReg(uint8_t reg) {
  Wire.beginTransmission(kCodecAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;  // 只发寄存器号
  if (Wire.requestFrom(static_cast<int>(kCodecAddr), 1) != 1) return -1;
  return static_cast<int32_t>(Wire.read());
}

bool BoardAudio::writeCodecReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kCodecAddr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool BoardAudio::writeCodecRegChecked(uint8_t reg, uint8_t value, bool* ok) {
  const bool r = writeCodecReg(reg, value);
  *ok = *ok && r;
  if (!r) Serial.printf("[audio] ES8311 写 0x%02X=0x%02X 失败\n", reg, value);
  return r;
}

// ---------------------------------------------------------------- I2S

bool BoardAudio::setupI2s() {
  i2s_chan_config_t chanCfg =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chanCfg.auto_clear = true;  // 没数据时输出 0，别把上一段的电平留在 DMA 里
  esp_err_t err = i2s_new_channel(&chanCfg, &s_tx, nullptr);
  if (err != ESP_OK) {
    Serial.printf("[audio] i2s_new_channel 失败 err=%d\n", static_cast<int>(err));
    return false;
  }

  i2s_std_config_t stdCfg = {};
  stdCfg.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(kSampleRate);  // mclk_multiple=256
  stdCfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
      I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  stdCfg.gpio_cfg.mclk = static_cast<gpio_num_t>(kMclkPin);
  stdCfg.gpio_cfg.bclk = static_cast<gpio_num_t>(kBclkPin);
  stdCfg.gpio_cfg.ws = static_cast<gpio_num_t>(kWsPin);
  stdCfg.gpio_cfg.dout = static_cast<gpio_num_t>(kDoutPin);
  stdCfg.gpio_cfg.din = static_cast<gpio_num_t>(kDinPin);
  stdCfg.gpio_cfg.invert_flags.mclk_inv = false;
  stdCfg.gpio_cfg.invert_flags.bclk_inv = false;
  stdCfg.gpio_cfg.invert_flags.ws_inv = false;

  err = i2s_channel_init_std_mode(s_tx, &stdCfg);
  if (err != ESP_OK) {
    Serial.printf("[audio] i2s_channel_init_std_mode 失败 err=%d\n",
                  static_cast<int>(err));
    i2s_del_channel(s_tx);
    s_tx = nullptr;
    return false;
  }
  err = i2s_channel_enable(s_tx);
  if (err != ESP_OK) {
    Serial.printf("[audio] i2s_channel_enable 失败 err=%d\n",
                  static_cast<int>(err));
    i2s_del_channel(s_tx);
    s_tx = nullptr;
    return false;
  }
  return true;
}

// ---------------------------------------------------------------- ES8311

bool BoardAudio::setupCodec() {
  bool ok = true;
  // 软复位 -> 释放 -> 上电。PocketOracle 的 es8311.cpp 就是这个顺序。
  writeCodecRegChecked(0x00, 0x1F, &ok);  // reset
  delay(20);
  writeCodecRegChecked(0x00, 0x00, &ok);  // release reset
  writeCodecRegChecked(0x00, 0x80, &ok);  // power on
  writeCodecRegChecked(0x01, 0x3F, &ok);  // enable all clocks

  // SCLK(BCLK) 不反相（清 bit5）
  const int32_t c6 = readCodecReg(0x06);
  if (c6 >= 0) writeCodecRegChecked(0x06, static_cast<uint8_t>(c6 & ~0x20), &ok);

  // ---- 48kHz / 16bit 时钟系数（mclk = 12.288MHz = 48000 × 256）----
  // 取自 PocketOracle es8311.cpp 的 coeff_div 表 48k / 12288000 那一行：
  //   pre_div=1 pre_multi=0(1x) adc_div=1 dac_div=1 fs_mode=0(单速)
  //   lrck_h=0x00 lrck_l=0xFF bclk_div=4 adc_osr=0x10 dac_osr=0x10
  const int32_t c2 = readCodecReg(0x02);
  if (c2 >= 0) {
    writeCodecRegChecked(0x02,
                         static_cast<uint8_t>((c2 & 0x07) | ((1 - 1) << 5) |
                                              (0 << 3)),
                         &ok);
  }
  writeCodecRegChecked(0x03, 0x10, &ok);  // fs_mode=0 | adc_osr=0x10
  writeCodecRegChecked(0x04, 0x10, &ok);  // dac_osr
  writeCodecRegChecked(0x05, 0x00, &ok);  // adc_div = dac_div = 1
  const int32_t c6b = readCodecReg(0x06);
  if (c6b >= 0) {
    writeCodecRegChecked(0x06,
                         static_cast<uint8_t>((c6b & 0xE0) | (4 - 1)), &ok);
  }
  const int32_t c7 = readCodecReg(0x07);
  if (c7 >= 0) writeCodecRegChecked(0x07, static_cast<uint8_t>(c7 & 0xC0), &ok);
  writeCodecRegChecked(0x08, 0xFF, &ok);  // lrck_l

  // 16bit：0x09/0x0A 的 [4:2] = 011
  const int32_t c9 = readCodecReg(0x09);
  if (c9 >= 0) writeCodecRegChecked(0x09, static_cast<uint8_t>(c9 | 0x0C), &ok);
  const int32_t ca = readCodecReg(0x0A);
  if (ca >= 0) writeCodecRegChecked(0x0A, static_cast<uint8_t>(ca | 0x0C), &ok);

  // ---- 模拟 / DAC 上电 ----
  writeCodecRegChecked(0x0D, 0x01, &ok);  // 模拟电路上电
  writeCodecRegChecked(0x0E, 0x02, &ok);  // analog PGA + ADC modulator
  writeCodecRegChecked(0x12, 0x00, &ok);  // DAC 上电
  writeCodecRegChecked(0x13, 0x10, &ok);  // 打开 HP 输出
  writeCodecRegChecked(0x1C, 0x6A, &ok);  // ADC EQ bypass、取消直流偏置
  writeCodecRegChecked(0x37, 0x08, &ok);  // DAC EQ bypass
  // DAC 音量：0xE0 ≈ 87.5%。上游 M5 profile 用的是 0xEF（接近满），
  // 这里留一点余量压底噪 —— 提示音本身的峰值只有 0.135 满幅。
  writeCodecRegChecked(0x32, 0xE0, &ok);
  return ok;
}

// ---------------------------------------------------------------- 生命周期

bool BoardAudio::begin() {
  // 配置期间功放先关着（PB 使能拉低），否则寄存器还没配好就开功放会"啪"一声。
  pinMode(kAmpPin, OUTPUT);
  digitalWrite(kAmpPin, LOW);
  ampOn_ = false;
  ampOffAtMs_ = 0;

  Wire.beginTransmission(kCodecAddr);
  const bool ack = (Wire.endTransmission() == 0);

  i2sOk_ = setupI2s();
  codecOk_ = ack && setupCodec();

  Serial.printf(
      "[audio] ES8311 addr=0x%02X ack=%d codec=%d i2s=%d @%luHz "
      "bck=%d ws=%d mck=%d dout=%d pa=%d\n",
      kCodecAddr, ack ? 1 : 0, codecOk_ ? 1 : 0, i2sOk_ ? 1 : 0,
      static_cast<unsigned long>(kSampleRate), kBclkPin, kWsPin, kMclkPin,
      kDoutPin, kAmpPin);
  if (!ack) {
    Serial.println("[audio] 0x18 无应答 —— 编解码器不在总线上（或 Wire 还没起来）");
  }
  return codecOk_;
}

void BoardAudio::setAmp(bool on) {
  pinMode(kAmpPin, OUTPUT);
  digitalWrite(kAmpPin, on ? HIGH : LOW);
  ampOn_ = on;
}

void BoardAudio::poll() {
  if (ampOffAtMs_ == 0) return;
  if (static_cast<int32_t>(millis() - ampOffAtMs_) >= 0) {
    ampOffAtMs_ = 0;
    setAmp(false);
  }
}

// ---------------------------------------------------------------- 播放

void BoardAudio::writeMono(const float* samples, size_t frames) {
  if (!i2sOk_ || s_tx == nullptr) return;
  size_t off = 0;
  while (off < frames) {
    size_t n = frames - off;
    if (n > kChunkFrames) n = kChunkFrames;
    for (size_t i = 0; i < n; ++i) {
      float v = samples[off + i];
      if (v > 1.0f) v = 1.0f;
      if (v < -1.0f) v = -1.0f;
      const int16_t s = static_cast<int16_t>(lroundf(v * 32767.0f));
      s_stereo[2 * i] = s;
      s_stereo[2 * i + 1] = s;
    }
    size_t written = 0;
    // 阻塞写：返回时数据已被搬进 DMA。1 秒超时足够 256 帧的需求。
    i2s_channel_write(s_tx, s_stereo, n * 2 * sizeof(int16_t), &written, 1000);
    off += n;
  }
}

void BoardAudio::playChime() {
  if (!i2sOk_) {
    Serial.println("[audio] chime 跳过：I2S 没起来");
    return;
  }
  // 上游 main.cpp buildCompletionChime() 的参数：A4 -> C#5 大三度，
  // 两音重叠（第二音在第一音还在响的时候就进来），270ms 结束。
  constexpr float kFirstFrequency = 440.00f;
  constexpr float kSecondFrequency = 554.37f;
  constexpr float kFirstDuration = 0.145f;
  constexpr float kSecondStart = 0.105f;
  constexpr float kSecondDuration = 0.165f;
  constexpr float kPeakAmplitude = 0.135f;
  constexpr float kTotalSeconds = 0.270f;

  const size_t totalFrames =
      static_cast<size_t>(kSampleRate * kTotalSeconds + 0.5f);
  const uint32_t startMs = millis();

  setAmp(true);
  float chunk[kChunkFrames];
  size_t done = 0;
  while (done < totalFrames) {
    size_t n = totalFrames - done;
    if (n > kChunkFrames) n = kChunkFrames;
    for (size_t i = 0; i < n; ++i) {
      const float t = static_cast<float>(done + i) / kSampleRate;
      const float first =
          sinf(2.0f * kPi * kFirstFrequency * t) *
          softNoteEnvelope(t, kFirstDuration);
      const float second =
          sinf(2.0f * kPi * kSecondFrequency * (t - kSecondStart)) *
          softNoteEnvelope(t - kSecondStart, kSecondDuration);
      chunk[i] = kPeakAmplitude * (first + second);
    }
    writeMono(chunk, n);
    done += n;
  }
  ampOffAtMs_ = millis() + kAmpTailMs;
  Serial.printf("[audio] chime %u frames @%luHz, 写完耗时 %lums\n",
                static_cast<unsigned>(totalFrames),
                static_cast<unsigned long>(kSampleRate),
                static_cast<unsigned long>(millis() - startMs));
}

void BoardAudio::playTone(float freqHz, uint32_t durationMs, float amplitude) {
  if (!i2sOk_) {
    Serial.println("[audio] tone 跳过：I2S 没起来");
    return;
  }
  const float durationSec = durationMs / 1000.0f;
  const size_t totalFrames =
      static_cast<size_t>(static_cast<uint64_t>(kSampleRate) * durationMs / 1000);

  setAmp(true);
  float chunk[kChunkFrames];
  size_t done = 0;
  while (done < totalFrames) {
    size_t n = totalFrames - done;
    if (n > kChunkFrames) n = kChunkFrames;
    for (size_t i = 0; i < n; ++i) {
      const float t = static_cast<float>(done + i) / kSampleRate;
      chunk[i] = amplitude * softNoteEnvelope(t, durationSec) *
                 sinf(2.0f * kPi * freqHz * t);
    }
    writeMono(chunk, n);
    done += n;
  }
  ampOffAtMs_ = millis() + kAmpTailMs;
  Serial.printf("[audio] tone %.0fHz %lums amp=%.2f\n", freqHz,
                static_cast<unsigned long>(durationMs), amplitude);
}
