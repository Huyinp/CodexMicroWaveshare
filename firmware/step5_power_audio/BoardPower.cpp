// SPDX-License-Identifier: MIT
// 见 BoardPower.h 的说明。本文件只读 AXP2101，外加三处"必须写"的使能位。

#include "BoardPower.h"

#include <Wire.h>

// 单寄存器读；-1 = 从机没 ACK（或读回来的长度不对）
int32_t BoardPower::readReg(uint8_t reg) {
  uint8_t v = 0;
  return readRegs(reg, &v, 1) ? static_cast<int32_t>(v) : -1;
}

// 连读 [reg, reg+len)。AXP2101 支持地址自增，所以一次事务就能拿一整段。
bool BoardPower::readRegs(uint8_t reg, uint8_t* out, uint8_t len) {
  if (len == 0) return false;
  Wire.beginTransmission(kAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;  // 只发寄存器号，不释放总线
  const uint8_t got =
      Wire.requestFrom(static_cast<int>(kAddr), static_cast<int>(len));
  if (got != len) return false;
  for (uint8_t i = 0; i < len; ++i) out[i] = static_cast<uint8_t>(Wire.read());
  return true;
}

bool BoardPower::writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kAddr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool BoardPower::begin() {
  // 探针 1：总线通不通（这一步等价于"0x34 有没有 ACK"）
  const int32_t probe = readReg(kRegStatus1);
  if (probe < 0) {
    ok_ = false;
    Serial.println("[pmu] 0x34 无应答 —— AXP2101 不在总线上（或 Wire 还没起来）");
    return false;
  }

  // 探针 2：芯片身份。AXP2101 的 0x03 固定是 0x4A。
  // 报出来的是"实测值"，不是判断结果 —— 换板/换批次时这一行最有用。
  deviceId_ = readReg(kRegIcType);
  const bool idOk = (deviceId_ == 0x4A);

  // ---- ① 打开 vbat / vbus 两路 ADC ----
  // AXP2101 的 ADC 通道默认可能是关的，不开这两路 0x34/0x38 恒读到 0 ——
  // 那样就会得到"电池电压 0mV、电量 0%"这种看起来像坏了的读数。
  // bit0 = 电池电压，bit2 = VBUS 电压（bit1 是 TS 脚，本板没接热敏电阻，不动）。
  const int32_t adcOld = readReg(kRegAdcCtrl);
  uint8_t adcWritten = 0;
  if (adcOld >= 0) {
    const uint8_t want = static_cast<uint8_t>(adcOld) | 0x05;
    if (want != adcOld) writeReg(kRegAdcCtrl, want);
    const int32_t back = readReg(kRegAdcCtrl);
    adcWritten = back < 0 ? 0 : static_cast<uint8_t>(back);
  }

  // ---- ② 打开电池在位检测 ----
  // 不开的话 STATUS1.bit3 可能一直是 0，于是"插着电池也报没电池"。
  const int32_t detOld = readReg(kRegBatDet);
  uint8_t detWritten = 0;
  if (detOld >= 0) {
    const uint8_t want = static_cast<uint8_t>(detOld) | 0x01;
    if (want != detOld) writeReg(kRegBatDet, want);
    const int32_t back = readReg(kRegBatDet);
    detWritten = back < 0 ? 0 : static_cast<uint8_t>(back);
  }

  // ---- ③ 清掉历史中断标记 + 打开 PWR 键的四个事件位 ----
  // 清标记是为了"拿到一个干净的起点"，否则第一次 pollPek() 会把上电前
  // 残留的按键事件当成刚发生的。
  writeReg(0x48, 0xFF);  // INTSTS1
  writeReg(0x49, 0xFF);  // INTSTS2（PWR 键在这里）
  writeReg(0x4A, 0xFF);  // INTSTS3
  const int32_t en2 = readReg(kRegInten2);
  uint8_t en2Back = 0;
  if (en2 >= 0) {
    // 四个 PWRON 事件全开。边沿位（bit0/bit1）比"短按/长按"可靠 ——
    // 后者要 PMU 内部的计时窗口配合，窗口没配就永远不置位（PocketOracle 实测）。
    const uint8_t want = static_cast<uint8_t>(en2) | 0x0F;
    if (want != en2) writeReg(kRegInten2, want);
    const int32_t back = readReg(kRegInten2);
    en2Back = back < 0 ? 0 : static_cast<uint8_t>(back);
  }

  Serial.printf(
      "[pmu] AXP2101 probe=%d id=0x%02X(%s) ADC 0x30=%d->%d 电池检测 0x68=%d->%d "
      "INTEN2 %d->%d\n",
      static_cast<int>(probe), deviceId_ < 0 ? 0 : static_cast<int>(deviceId_),
      idOk ? "对" : "**不是 0x4A**", adcOld < 0 ? -1 : static_cast<int>(adcOld),
      static_cast<int>(adcWritten), detOld < 0 ? -1 : static_cast<int>(detOld),
      static_cast<int>(detWritten), en2 < 0 ? -1 : static_cast<int>(en2),
      static_cast<int>(en2Back));

  ok_ = idOk;
  readNow();
  return ok_;
}

void BoardPower::readNow() {
  BoardPowerReading n;
  uint8_t b[2] = {};

  if (!readRegs(kRegStatus1, b, 2)) {  // 0x00 + 0x01 一次事务
    r_ = n;                            // ok=false，其余保持 0
    return;
  }
  n.status1 = b[0];
  n.status2 = b[1];
  n.ok = true;

  const int32_t id = readReg(kRegIcType);
  n.icType = id < 0 ? 0 : static_cast<uint8_t>(id);
  const int32_t adc = readReg(kRegAdcCtrl);
  n.adcCtrl = adc < 0 ? 0 : static_cast<uint8_t>(adc);

  if (readRegs(kRegVbat, b, 2)) {
    // 高 5 位 + 低 8 位 = 13 位，单位 mV（AXP2101 的电压 ADC 直接输出 mV）
    n.vbatMv = static_cast<uint16_t>(((b[0] & 0x1F) << 8) | b[1]);
  }
  if (readRegs(kRegVbus, b, 2)) {
    n.vbusMv = static_cast<uint16_t>(((b[0] & 0x3F) << 8) | b[1]);
  }
  const int32_t soc = readReg(kRegSoc);
  n.socRaw = soc < 0 ? -1 : static_cast<int16_t>(soc);
  const int32_t pek = readReg(kRegPek);
  n.pekRaw = pek < 0 ? 0 : static_cast<uint8_t>(pek & 0x0F);

  n.batteryPresent = (n.status1 & 0x08) != 0;
  n.vbusGood = (n.status1 & 0x20) != 0;
  n.charging = ((n.status2 >> 5) & 0x07) == 1;
  n.chgState = static_cast<uint8_t>(n.status2 & 0x07);
  // 与 XPowersLib::isVbusIn() 同一判据：STATUS2.bit3 == 0 且 VBUS 电压正常
  n.vbusIn = ((n.status2 >> 3) & 0x01) == 0 && n.vbusGood;

  r_ = n;
  ++readCount_;
}

void BoardPower::poll() {
  if (!ok_) return;
  const uint32_t now = millis();
  if (lastPollMs_ != 0 && (now - lastPollMs_) < kPollIntervalMs) return;
  lastPollMs_ = now;
  readNow();
}

int8_t BoardPower::percentForUi() const {
  if (!r_.ok || !r_.batteryPresent) return -1;
  // 首选库仑计（0xA4）。AXP2101 只在电池在位时这个寄存器才有意义，
  // 而且未标定的批次会返回 0/255 之类的垃圾 —— 所以加范围校验，
  // 不合格就退回"按电压线性估算"（单节锂电 3.30V~4.15V）。
  if (r_.socRaw >= 0 && r_.socRaw <= 100) {
    return static_cast<int8_t>(r_.socRaw);
  }
  if (r_.vbatMv == 0) return -1;
  int p = (static_cast<int>(r_.vbatMv) - 3300) * 100 / (4150 - 3300);
  if (p < 0) p = 0;
  if (p > 100) p = 100;
  return static_cast<int8_t>(p);
}

uint8_t BoardPower::pollPek() {
  if (!ok_) return 0;
  const int32_t v = readReg(kRegPek);
  if (v < 0) return 0;
  const uint8_t bits = static_cast<uint8_t>(v & 0x0F);
  if (bits) writeReg(kRegPek, bits);  // 写 1 清标记
  return bits;
}
