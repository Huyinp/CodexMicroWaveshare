// SPDX-License-Identifier: MIT
// 板级按键层：板载 TCA9554 扩展 IO（原理图 U7 "I/O Extensions"，EXIO0..EXIO7）。
//
// 【为什么需要它】
// step5 一开始只按"PWR 键接在 AXP2101 PWRON 上"来做（PocketOracle 的 config.h
// 也是这么写的），但实测按下去 0x49(INTSTS2) 毫无反应 —— 连 bit1(PWRON 负沿)
// 都不置位。查 1.8 官方原理图发现板上除 AXP2101 外还挂了一颗 **TCA9554PWR**，
// 8 个引脚直接就叫 EXIO0..EXIO7；PocketOracle 的 config.h 里那条后备注释
// （`PWR_EXIO_ADDR 0x20 / PWR_EXIO_BIT 4`，高电平=按下）说的就是它。
// 所以 PWR 键的真实通道有两条候选，这一步把第二条也接上：
//   · PMU 通道   0x34 寄存器 0x49 bit0/bit1 —— 见 BoardPower
//   · 扩展 IO    0x20~0x27 输入端口 bit4 —— 本文件
// 两个都读，谁先动就是谁（`x` 命令的监控窗口能把这件事一次看清）。
//
// 【硬约束：只读】
// TCA9554 的 0x03(CONFIG) 决定每个脚是输入还是输出。板子出厂时这些脚要么
// 全输入、要么被别处（比如屏幕/功放使能）当输出用。本文件**一个字节都不写**：
// 只读 0x00(输入端口)/0x01(输出端口)/0x02(极性)/0x03(配置) 供判别。
// 若某天确实要改成输出，必须先读到原配置再按"读-改-写"处理，并确认那个脚
// 没被别的东西用着 —— 直接写 0 会把板子上的某个使能脚拉到地。
//
// ⚠️ 与 BoardPower 一样，必须在 BoardTouch::begin()（Wire.begin）之后调用。

#pragma once

#include <Arduino.h>

class BoardKeys {
 public:
  static constexpr int8_t kSda = 15;
  static constexpr int8_t kScl = 14;

  // TCA9554 的地址由 A2A1A0 三个脚决定，区间就是 0x20..0x27。
  static constexpr uint8_t kAddrFirst = 0x20;
  static constexpr uint8_t kAddrLast = 0x27;

  static constexpr uint8_t kRegInput0 = 0x00;  // 输入端口（读电平）
  static constexpr uint8_t kRegInput1 = 0x01;  // 输出端口（读回锁存值）
  static constexpr uint8_t kRegPolar = 0x02;   // 极性反转（只读，用于判别）
  static constexpr uint8_t kRegConfig = 0x03;  // 1 = 输入，0 = 输出

  // ✅ 2026-09-24 实测确认：EXIO4 = PWR 键，**高电平 = 按下**。
  // 一次真实短按的事件环（`xl`）里成对出现：
  //   EXIO4:0->1（按下） …157~263ms… EXIO4:1->0（抬起）
  // 而同一时刻 AXP2101 的 0x49 也给出 PWR_NEG / PWR_POS|PWR_SHORT ⇒
  // **同一颗键被引到了两处**（PMU 的 PWRON + 扩展 IO 的 EXIO4），
  // 不是"二者之一"。本工程的动作走 PMU 那条（它能区分短按/长按），
  // 这条只读留着做交叉验证、以及 PMU 失联时的兜底。
  //
  // 另：EXIO5 与按键无关 —— 它在"充电完成"那一刻 1->0 并持续保持低
  // （与 0x4A 的 BAT_CHG_DONE 同一时刻），是一条状态线。
  static constexpr uint8_t kPwrBit = 4;

  // 探针：扫 0x20..0x27，挑第一个能读通配置寄存器的当作扩展 IO。
  // 必须在 BoardTouch::begin() 之后调用。
  bool begin();

  bool ready() const { return ok_; }
  uint8_t addr() const { return addr_; }
  uint8_t config() const { return config_; }
  uint8_t polar() const { return polar_; }
  uint8_t idle0() const { return idle0_; }
  uint8_t port0() const { return last0_; }
  uint8_t port1() const { return last1_; }

  // 读一次输入端口 0/1（0x00 起连读两字节）。失败时返回 false 且不改缓存值。
  bool readPorts(uint8_t* p0, uint8_t* p1);

  // PWR 键是否按下：把 EXIO4 与"开机那一刻的静止电平"比 —— 这样高有效/
  // 低有效搞反了也不会把判据带偏（本板待实测，先不做假设）。
  bool pressed();

  // 找到的所有应答地址（0 表示没找到）。begin() 之后有效。
  uint8_t foundCount() const { return foundCount_; }
  uint8_t foundAddr(uint8_t i) const {
    return i < foundCount_ ? found_[i] : 0;
  }

 private:
  int32_t readReg(uint8_t addr, uint8_t reg);

  bool ok_ = false;
  uint8_t addr_ = 0;
  uint8_t config_ = 0;
  uint8_t polar_ = 0;
  uint8_t idle0_ = 0;
  uint8_t last0_ = 0;
  uint8_t last1_ = 0;
  uint8_t found_[8] = {};
  uint8_t foundCount_ = 0;
};
