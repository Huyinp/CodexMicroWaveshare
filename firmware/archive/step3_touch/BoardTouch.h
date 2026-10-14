// SPDX-License-Identifier: MIT
// 板级触摸层：Waveshare ESP32-S3-Touch-AMOLED-1.8 (V2) 上的 CST820
// （CST816x 系列，I2C 0x15，INT=GPIO21，SDA=15 / SCL=14）
//
// 第 ③ 步的定位：**只做原始坐标采样，不做任何变换**。
//
// 上游 1.75C 的 CST9217 需要 (232-raw)*2 这套"镜像 + 缩放"，那是 232 量程的
// 特定产物。本板是同尺寸不同后缀的芯片，量程/镜像方向**不能互推** ——
// 所以在写任何变换公式之前，先用这一版把三件事钉死：
//   1) 有效量程是多少（靠 min/max 实测，不靠猜）
//   2) 有没有镜像（看物理左上角对应 raw 的哪个角）
//   3) 中断引脚到底有没有在动（irqCount 一直不涨 = INT 没接对/模式没配）
// 变换与校准留到拿到这三个答案之后再做。

#pragma once

#include <Arduino.h>
#include <Arduino_DriveBus_Library.h>

#include <memory>

// 一次采样的结果。x/y 是**未经任何变换**的芯片原始坐标。
struct BoardTouchSample {
  bool down = false;   // 采样时刻手指是否在屏上
  uint8_t fingers = 0; // 芯片报的手指数
  int16_t x = -1;
  int16_t y = -1;
};

class BoardTouch {
 public:
  static constexpr int8_t kSda = 15;
  static constexpr int8_t kScl = 14;
  static constexpr int8_t kIntPin = 21;
  static constexpr uint8_t kI2cAddr = 0x15;

  // 坐标变化超过这个像素数才算"移动"，避免抖动把串口刷爆。
  static constexpr int16_t kMoveDeltaPx = 2;
  // 中断不触发时的兜底轮询周期。双路并存：中断管实时，轮询管兜底。
  static constexpr uint32_t kPollFallbackMs = 100;

  bool begin();
  void poll();  // 每个 loop 调一次

  bool ready() const { return ok_; }
  int32_t deviceId() const { return deviceId_; }  // 读 0xA7；-1 = 读不到
  uint32_t irqCount() const;                      // 中断次数，证明 INT 在动
  uint32_t samples() const { return samples_; }   // 有效样本数

  // ---------------- 诊断通道（绕过驱动解码） ----------------
  //
  // 为什么需要：驱动的 `IIC_Read_Device_Value()` 只认 {0,1,2} 三个返回值，
  // 其它一律折成 -1（= 按"没手指"处理）。于是"真的没人点"和"点了但芯片
  // 报了个驱动不认识的值"在串口上长得一模一样（irq=0 smp=0）。
  // 这一组接口直接把 0x15 的原始寄存器读出来，把这两种情况分开。
  //
  // 读 addr=0x15 的 [reg, reg+len)，一次 I2C 事务。false = 连 ACK 都没有。
  bool readRegs(uint8_t reg, uint8_t* out, uint8_t len);
  bool writeReg(uint8_t reg, uint8_t value);
  // INT 引脚当前电平。1 = 空闲（上拉），0 = 芯片正在拉低报事件。
  uint8_t intLevel() const;
  // 驱动那条路的解码结果，仅用于和原始值对照。
  double driverValue(uint32_t information);

  // 自上次调用以来状态有变化（按下 / 抬起 / 移动 >= kMoveDeltaPx）。
  bool takeSample(BoardTouchSample& out);

  // 当前状态（给屏幕用）
  bool pressed() const { return pressed_; }
  int16_t x() const { return x_; }
  int16_t y() const { return y_; }
  uint8_t fingers() const { return fingers_; }

  // 实测量程。没采到过样本时返回 0。
  int16_t minX() const { return minX_; }
  int16_t maxX() const { return maxX_; }
  int16_t minY() const { return minY_; }
  int16_t maxY() const { return maxY_; }
  void resetRange();

 private:
  std::shared_ptr<Arduino_IIC_DriveBus> bus_;
  std::unique_ptr<Arduino_IIC> chip_;

  bool ok_ = false;
  int32_t deviceId_ = -1;

  uint32_t lastPollMs_ = 0;
  uint32_t samples_ = 0;

  bool pressed_ = false;
  uint8_t fingers_ = 0;
  int16_t x_ = 0;
  int16_t y_ = 0;

  int16_t minX_ = 0;
  int16_t maxX_ = 0;
  int16_t minY_ = 0;
  int16_t maxY_ = 0;

  bool pending_ = false;
  BoardTouchSample pendingSample_;
};
