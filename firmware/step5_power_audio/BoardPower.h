// SPDX-License-Identifier: MIT
// 板级电源层：Waveshare ESP32-S3-Touch-AMOLED-1.8 (V2) 上的 AXP2101 PMIC
// （I2C 0x34，走触摸那条 SDA=15 / SCL=14 总线）。
//
// 第 ⑤ 步的定位：**只读，不动任何电源轨**。
// 也就是不碰 DCDC/LDO 的使能与电压、不改充放电策略 —— 那些改错了会让板子
// 直接掉电或充不进电，而本步只需要"把电量显示对"。全文件只有三处写寄存器：
//   ① 0x30 打开 vbat / vbus 两路 ADC（默认可能是关的，不开就读到 0）
//   ② 0x68 打开电池在位检测（同上，不开 STATUS1.bit3 可能恒 0）
//   ③ 0x49 写 1 清 PWR 键的中断标记（读后要清，否则一直重复报同一次按键）
// 三处都"先读、只改需要的位、写完回读"，写不进去就退回原值并如实上报。
//
// 【为什么不用 XPowersLib】
// 与 PocketOracle 的结论一致（那份是这块板实测过的）：库的 begin() 先读 0x03
// 校验 chip ID，型号/版本对不上就整体失效，而我们要的只是读几个状态位 //
// 库的 isCharging() 之类前面还多一道 INTEN 使能位的门。裸 I2C 把两道门都绕开了。
//
// 【为什么能共用 Wire】
// 触摸（BoardTouch）、ES8311（BoardAudio）、板载 RTC/IMU 全在同一条 I2C 上，
// 都走 Arduino 的 Wire。本文件不做 ISR 内的 I2C，只在 loop() 里顺序访问，
// 所以不需要第二路 I2C master，也不存在重入。
// ⚠️ 前提：**必须在 BoardTouch::begin() 之后调用**（Wire 由它 Wire.begin(15,14) 起来）。

#pragma once

#include <Arduino.h>

// 一次完整读数。所有字段都是"芯片当场答的"，没有任何推算（percent 除外，见下）。
struct BoardPowerReading {
  bool ok = false;              // 这一次 I2C 读通了没有
  uint8_t icType = 0;           // 0x03，AXP2101 = 0x4A
  uint8_t status1 = 0;          // 0x00
  uint8_t status2 = 0;          // 0x01
  uint8_t adcCtrl = 0;          // 0x30
  uint8_t chgState = 0;         // 0x01[2:0] 充电状态机
  uint8_t pekRaw = 0;           // 0x49[3:0] PWR 键事件（读后已被清掉）
  uint16_t vbatMv = 0;          // 0x34/0x35，14 位，单位 mV
  uint16_t vbusMv = 0;          // 0x38/0x39
  int16_t socRaw = -1;          // 0xA4 原始字节，-1 = 没读到
  bool batteryPresent = false;  // 0x00.bit3
  bool charging = false;        // 0x01[6:5] == 1
  bool vbusGood = false;        // 0x00.bit5
  bool vbusIn = false;          // 0x01.bit3 == 0 且 vbusGood
};

class BoardPower {
 public:
  // 与 BoardTouch 同一条总线（这两个数是实测值，不是 variant 里写的那组）
  static constexpr int8_t kSda = 15;
  static constexpr int8_t kScl = 14;
  static constexpr uint8_t kAddr = 0x34;

  static constexpr uint8_t kRegStatus1 = 0x00;
  static constexpr uint8_t kRegStatus2 = 0x01;
  static constexpr uint8_t kRegIcType = 0x03;
  static constexpr uint8_t kRegAdcCtrl = 0x30;
  static constexpr uint8_t kRegVbat = 0x34;  // 高 5 位；低 8 位 = +1
  static constexpr uint8_t kRegVbus = 0x38;  // 高 6 位；低 8 位 = +1
  static constexpr uint8_t kRegInten1 = 0x40;
  static constexpr uint8_t kRegInten2 = 0x41;  // PWR 键中断使能
  static constexpr uint8_t kRegInten3 = 0x42;
  static constexpr uint8_t kRegIntsts1 = 0x48;  // 中断状态（RW1C：读不清，写 1 清）
  static constexpr uint8_t kRegPek = 0x49;      // INTSTS2：PWR 键事件
  static constexpr uint8_t kRegIntsts3 = 0x4A;
  // PWRON 按键级别/时间配置（IRQLEVEL/OFFLEVEL/ONLEVEL），排查用，只读。
  static constexpr uint8_t kRegPwronCfg = 0x27;
  static constexpr uint8_t kRegBatDet = 0x68;  // 电池在位检测使能
  static constexpr uint8_t kRegSoc = 0xA4;     // BAT_PERCENT_DATA 0..100

  // PWR 键事件位（0x49 = INTSTS2）。位定义取自 XPowersLib 的
  // XPowersParams.hpp → xpowers_axp2101_irq_t（使能寄存器 0x41 的低 8 位）：
  //   bit0 PKEY_POSITIVE(抬手) bit1 PKEY_NEGATIVE(按下)
  //   bit2 PKEY_LONG          bit3 PKEY_SHORT
  //   bit4 BAT_REMOVE bit5 BAT_INSERT bit6 VBUS_REMOVE bit7 VBUS_INSERT
  //
  // ✅ 2026-09-24 实测确认：短按一次 PWR，0x49 依次给出
  //   `0x02`(PWR_NEG，按下负沿) → `0x09`(PWR_POS|PWR_SHORT，抬手正沿 + 短按)
  // ⇒ **PWR 键确实接在 AXP2101 的 PWRON 上**，PEK 通道可用；0x41(INTEN2)=0xFF
  // 的出厂使能就是它能报出来的前提。
  // ⚠️ "一次按键 = 两条事件"：SHORT 只在抬手那条里出现一次，所以拿它当
  // "点击"不会重复触发（主固件的 pollPowerButton 就是这么用的）。
  // 长按交给 AXP2101 硬件（约 6s 关机），软件不接管。
  static constexpr uint8_t kPekPos = 0x01;    // 正边沿 = 抬手
  static constexpr uint8_t kPekNeg = 0x02;    // 负边沿 = 按下
  static constexpr uint8_t kPekLong = 0x04;   // 长按
  static constexpr uint8_t kPekShort = 0x08;  // 短按

  // 轮询周期。电量是缓变量，2 秒足够。
  static constexpr uint32_t kPollIntervalMs = 2000;

  // 探针 + 打开遥测。返回 true = 0x34 有应答且 0x03 是 AXP2101 的 ID。
  // ⚠️ 必须在 BoardTouch::begin() 之后调用（见文件头）。
  bool begin();

  bool ready() const { return ok_; }
  int32_t deviceId() const { return deviceId_; }

  void poll();     // loop 里调一次；内部按 kPollIntervalMs 节流
  void readNow();  // 立刻读一遍（不等节流）

  const BoardPowerReading& reading() const { return r_; }

  // ---- 给 UI / BLE 的两个值 ----
  // 电量：-1 = 未知（读不到芯片，或没接电池）。
  // 不接电池时**故意返回 -1**，让屏上显示 "--%" 而不是编一个 100% 出来 ——
  // 上游那种"没电池就当满电"的兜底在插着 USB 时会让人分不清
  // "真充满"还是"根本没有电池"。
  int8_t percentForUi() const;
  bool chargingForUi() const { return r_.ok && r_.charging; }
  bool vbusPresent() const { return r_.ok && r_.vbusIn; }

  // ---- 原始寄存器通道（排查用） ----
  bool readRegs(uint8_t reg, uint8_t* out, uint8_t len);
  int32_t readReg(uint8_t reg);  // -1 = 无应答
  bool writeReg(uint8_t reg, uint8_t value);

  // PWR 键：读一次 0x49 并把事件位写回清零，返回本次的事件位（0 = 无事件）。
  uint8_t pollPek();

 private:
  bool ok_ = false;
  int32_t deviceId_ = -1;
  uint32_t lastPollMs_ = 0;
  uint32_t readCount_ = 0;
  BoardPowerReading r_;
};
