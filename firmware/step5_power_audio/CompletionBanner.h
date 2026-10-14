// SPDX-License-Identifier: MIT
// 上游 include/CompletionBanner.h 原样搬（与硬件无关）。
// 一个"某个 Agent 刚跑完"的横幅计时器：show() 记下 agent 和到期时刻，
// visible() 看还在不在窗口内，expire() 由 loop 每轮调一次负责收尾。

#pragma once

#include <cstdint>

namespace completion_banner {

struct Timer {
  std::int8_t agent = -1;
  std::uint32_t until_ms = 0;

  void show(std::int8_t completed_agent, std::uint32_t now,
            std::uint32_t duration_ms) {
    agent = completed_agent;
    until_ms = now + duration_ms;
  }

  bool visible(std::uint32_t now) const {
    return agent >= 0 && static_cast<std::int32_t>(until_ms - now) > 0;
  }

  bool expire(std::uint32_t now) {
    if (agent < 0 || visible(now)) return false;
    agent = -1;
    until_ms = 0;
    return true;
  }
};

}  // namespace completion_banner
