// SPDX-License-Identifier: MIT
// 见 BoardKeys.h 的说明。本文件对 TCA9554 只读，绝不写任何寄存器。

#include "BoardKeys.h"

#include <Wire.h>

int32_t BoardKeys::readReg(uint8_t addr, uint8_t reg) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;  // 只发寄存器号，不释放总线
  if (Wire.requestFrom(static_cast<int>(addr), 1) != 1) return -1;
  return static_cast<int32_t>(Wire.read());
}

bool BoardKeys::begin() {
  foundCount_ = 0;

  // 先把 0x20..0x27 全扫一遍再判断：既拿到"到底有没有应答"，也留下现场，
  // 万一地址不是 0x20（A2A1A0 被拉高过）也能从日志看出来。
  for (uint8_t a = kAddrFirst; a <= kAddrLast; ++a) {
    if (readReg(a, kRegConfig) < 0) continue;
    if (foundCount_ < sizeof(found_)) found_[foundCount_] = a;
    ++foundCount_;
  }

  Serial.printf("[keys] 扩展 IO 扫描 0x%02X..0x%02X 应答:", kAddrFirst,
                kAddrLast);
  if (foundCount_ == 0) {
    Serial.println(" (无)");
    ok_ = false;
    return false;
  }
  for (uint8_t i = 0; i < foundCount_ && i < 8; ++i) {
    Serial.printf(" 0x%02X", found_[i]);
  }
  Serial.println();

  addr_ = found_[0];
  // 注意：readReg 失败返回 -1，别直接 & 0xFF —— 那会变成 0xFF 这个"看起来
  // 很正常"的值（指南 8.21 里那条"十进制/十六进制读错"是同一类坑）。
  const int32_t cfg = readReg(addr_, kRegConfig);
  const int32_t pol = readReg(addr_, kRegPolar);
  const int32_t o1 = readReg(addr_, kRegInput1);
  const int32_t i0 = readReg(addr_, kRegInput0);
  config_ = cfg < 0 ? 0 : static_cast<uint8_t>(cfg);
  polar_ = pol < 0 ? 0 : static_cast<uint8_t>(pol);
  last1_ = o1 < 0 ? 0 : static_cast<uint8_t>(o1);
  last0_ = i0 < 0 ? 0 : static_cast<uint8_t>(i0);
  idle0_ = last0_;

  // 0x03 全 1 = 8 个脚全是输入（TCA9554 出厂默认），此时读 0x00 得到的就是
  // 引脚真实电平。有哪一位是 0，说明那个脚是输出，读到的不是外部按键。
  Serial.printf(
      "[keys] 用 0x%02X: config(0x03)=0x%02X%s polar(0x02)=0x%02X "
      "port0=0x%02X port1=0x%02X -> EXIO%d=%d（静止电平）\n",
      addr_, config_, config_ == 0xFF ? "(全输入)" : "(有输出脚)", polar_,
      last0_, last1_, kPwrBit, (last0_ >> kPwrBit) & 1);

  ok_ = true;
  return true;
}

bool BoardKeys::readPorts(uint8_t* p0, uint8_t* p1) {
  if (!ok_) return false;
  // 0x00 起连读两字节：0x00=输入端口、0x01=输出端口锁存值。
  Wire.beginTransmission(addr_);
  Wire.write(kRegInput0);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(static_cast<int>(addr_), 2) != 2) return false;
  const uint8_t a = static_cast<uint8_t>(Wire.read());
  const uint8_t b = static_cast<uint8_t>(Wire.read());
  last0_ = a;
  last1_ = b;
  if (p0 != nullptr) *p0 = a;
  if (p1 != nullptr) *p1 = b;
  return true;
}

bool BoardKeys::pressed() {
  if (!ok_) return false;
  return ((last0_ >> kPwrBit) & 1) != ((idle0_ >> kPwrBit) & 1);
}
