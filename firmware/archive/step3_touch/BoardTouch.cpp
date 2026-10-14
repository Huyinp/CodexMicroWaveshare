// SPDX-License-Identifier: MIT
// 见 BoardTouch.h 的说明：本文件只做原始坐标采样，不做任何变换。

#include "BoardTouch.h"

#include <Wire.h>

namespace {

// 中断计数器放在文件级：ISR 里只碰这两个 volatile 变量，不碰对象，
// 省掉"ISR 通过实例指针访问成员"那套 volatile / 竞态讲究。
volatile uint32_t g_irqCount = 0;
volatile bool g_irqFlag = false;

// 下降沿触发（Arduino_IIC::begin() 里 attachInterrupt(..., FALLING)）。
void boardTouchIsr() {
  ++g_irqCount;
  g_irqFlag = true;
}

}  // namespace

bool BoardTouch::begin() {
  // 和 PocketOracle 同款接法：显式 begin 一次 Wire，再把同一个 Wire 交给
  // Arduino_HWIIC。总线是触摸 / PMU / RTC / IMU 共用的那条。
  Wire.begin(kSda, kScl);

  bus_ = std::make_shared<Arduino_HWIIC>(kSda, kScl, &Wire);
  // rst 传 DRIVEBUS_DEFAULT_VALUE = 没有复位脚（本板触摸没有独立 RST）。
  chip_.reset(new Arduino_CST816x(bus_, kI2cAddr, DRIVEBUS_DEFAULT_VALUE, kIntPin,
                                  boardTouchIsr));

  if (chip_->begin()) {
    // 默认初始化里写的是「检测到手势才发脉冲」（MOTION），按住不动就不再报。
    // 改成 PERIODIC（检测到触摸就周期性发低脉冲），按住期间才有连续坐标流。
    chip_->IIC_Write_Device_State(
        Arduino_IIC_Touch::Device::TOUCH_DEVICE_INTERRUPT_MODE,
        Arduino_IIC_Touch::Device_Mode::TOUCH_DEVICE_INTERRUPT_PERIODIC);
    ok_ = true;
  } else {
    ok_ = false;
  }

  // 不管成没成，都试着读一次芯片 ID（寄存器 0xA7）。CST816x/CST820 这里
  // 通常返回 0xB4/B5/B6 之类；读不到就是 -1。这是"总线通不通"的最快判据。
  deviceId_ = chip_->IIC_Read_Device_ID();

  resetRange();
  return ok_;
}

uint32_t BoardTouch::irqCount() const { return g_irqCount; }

uint8_t BoardTouch::intLevel() const { return digitalRead(kIntPin) == HIGH ? 1 : 0; }

double BoardTouch::driverValue(uint32_t information) {
  if (!chip_) return -1;
  Arduino_CST816x* t = static_cast<Arduino_CST816x*>(chip_.get());
  return t->IIC_Read_Device_Value(information);
}

// 直接走 Wire 读原始寄存器，不经过驱动的 switch/解码。
// CST816x/CST820 的寄存器图：0x01 GestureID / 0x02 FingerNum /
// 0x03-0x04 Xpos / 0x05-0x06 Ypos。凡是这几格里有非零值，就说明芯片
// 确实在报触摸 —— 跟驱动认不认这个值无关。
bool BoardTouch::readRegs(uint8_t reg, uint8_t* out, uint8_t len) {
  if (len == 0) return false;
  Wire.beginTransmission(kI2cAddr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;  // 只发地址+寄存器号，不释放总线
  const uint8_t got = Wire.requestFrom(static_cast<int>(kI2cAddr),
                                       static_cast<int>(len));
  if (got != len) return false;
  for (uint8_t i = 0; i < len; ++i) out[i] = static_cast<uint8_t>(Wire.read());
  return true;
}

bool BoardTouch::writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(kI2cAddr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

void BoardTouch::resetRange() {
  minX_ = maxX_ = minY_ = maxY_ = 0;
  samples_ = 0;
}

bool BoardTouch::takeSample(BoardTouchSample& out) {
  if (!pending_) return false;
  pending_ = false;
  out = pendingSample_;
  return true;
}

void BoardTouch::poll() {
  if (!ok_) return;

  const uint32_t now = millis();
  const bool irq = g_irqFlag;
  g_irqFlag = false;

  // 双路：中断触发就立刻读；否则每 kPollFallbackMs 兜底读一次。
  // 兜底这一路很关键 —— 一旦 INT 没接对或中断模式没配成功，
  // 光靠中断会表现为"完全没有数据"，而有了兜底至少还能读到坐标。
  if (!irq && (now - lastPollMs_) < kPollFallbackMs) return;
  lastPollMs_ = now;

  Arduino_CST816x* t = static_cast<Arduino_CST816x*>(chip_.get());

  const int32_t n = static_cast<int32_t>(t->IIC_Read_Device_Value(
      Arduino_IIC_Touch::Value_Information::TOUCH_FINGER_NUMBER));

  if (n <= 0) {
    // 没手指 = 抬起。注意这里**不能**直接 return，还要让上层知道状态变了。
    if (pressed_) {
      pressed_ = false;
      fingers_ = 0;
      pendingSample_ = {false, 0, x_, y_};  // 保留最后坐标，抬起点也看得见
      pending_ = true;
    }
    return;
  }

  const int32_t rx = static_cast<int32_t>(t->IIC_Read_Device_Value(
      Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X));
  const int32_t ry = static_cast<int32_t>(t->IIC_Read_Device_Value(
      Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y));
  if (rx < 0 || ry < 0) return;  // 读失败，下次再来

  const bool wasDown = pressed_;
  const int16_t prevX = x_;
  const int16_t prevY = y_;

  x_ = static_cast<int16_t>(rx);
  y_ = static_cast<int16_t>(ry);
  fingers_ = static_cast<uint8_t>(n > 255 ? 255 : n);
  pressed_ = true;

  // 实测量程：老胡把四个角点一遍，min/max 就是这块屏的真实量程。
  if (samples_ == 0) {
    minX_ = maxX_ = x_;
    minY_ = maxY_ = y_;
  } else {
    if (x_ < minX_) minX_ = x_;
    if (x_ > maxX_) maxX_ = x_;
    if (y_ < minY_) minY_ = y_;
    if (y_ > maxY_) maxY_ = y_;
  }
  ++samples_;

  const int16_t dx = static_cast<int16_t>(abs(x_ - prevX));
  const int16_t dy = static_cast<int16_t>(abs(y_ - prevY));
  if (!wasDown || dx >= kMoveDeltaPx || dy >= kMoveDeltaPx) {
    pendingSample_ = {true, fingers_, x_, y_};
    pending_ = true;
  }
}
