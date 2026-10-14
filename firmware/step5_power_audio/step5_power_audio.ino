// SPDX-License-Identifier: MIT
// 第 ⑤ 步：AXP2101 电源遥测 + ES8311 完成提示音。
// （本文件由 step4_dashboard.ino 原样复制而来，新增的是 BoardPower /
//   BoardAudio 两块板级层与它们的接线，看板/触摸部分一字未改。）
//
// 这一版把上游的五层接起来：
//   ① CodexMicroBle（协议，与 step1 逐字相同）
//   ② DashboardUi   （版式，从上游 466x466 重排到 368x448）
//   ③ BoardTouch    （CST820 原始坐标 -> 直接当屏幕坐标用）
//   ④ BoardPower    （AXP2101：真电量/充电/VBUS + PWR 键事件）★ 本步新增
//   ⑤ BoardAudio    （ES8311 + I2S：Agent 跑完响一声）★ 本步新增
//   ⑥ BoardKeys     （TCA9554 扩展 IO：PWR 键的第二条候选通道）★ 本步新增
//
// 【PWR 键为什么要接两条通道】
// 一开始按"PWR 键在 AXP2101 的 PWRON 上"实现（PocketOracle 的 config.h
// 也是这么写的），但实测按下去 AXP2101 的 0x49(INTSTS2) **一位都不动**：
// 连 bit1(PWRON 负沿) 都不置位，而中断状态位是不受 INTEN 使能位影响的
// （使能位只决定 IRQ 引脚输不输出），所以"没置位"= 这条线上真的没信号。
// 翻 1.8 官方原理图：板上除 AXP2101 外还有一颗 **TCA9554PWR**（U7
// "I/O Extensions"，引脚就叫 EXIO0..EXIO7），EXT 侧还有 10K 上拉。
// 于是两条通道都读，用 `x` 命令的嗅探窗口让"按下这一下"自己指认是谁。
//
// 关于 ③ 的结论（step3 实测 + ④ 的六点校准，见指南 8.12 / 8.18 节）：
//   raw 量程 0..367 x 0..447，**无镜像、无旋转**，与屏幕接近 1:1；
//   但**数字仪增益比屏大 14%(x) / 20%(y)**，四向朝外偏、屏心不偏
//   （点屏幕四角时偏差可达 40px）。所以命中测试之前必须过一层
//   raw -> screen 的线性修正（calKx/calCx/calKy/calCy，见下）。
//   修正前六键偏差 24~40px，修正后残差 rms≈7.8px。
//
// 触摸交互照搬上游 main.cpp 的状态机（begin -> update -> finish）：
//   begin : 命中 Agent 键 -> 待提交；命中表盘 -> 待提交 Send
//   update: 位移超过阈值 -> 改成滑动手势（发摇杆），并取消按键候选
//   finish: 抬起；候选存活够久才提交（滤掉 CST816x 的幻触）
//
// 第 ⑤ 步（本文件）在 ④ 的基础上再接两块板级模块：
//   ④ BoardPower —— AXP2101 的真实电量 / 充电 / VBUS（④ 里电量是写死的 100%）
//   ⑤ BoardAudio —— ES8311 + I2S，某个 Agent 跑完时响一声（上游的完成提示音）
//
// 上游那套 haptic / 深度休眠 / 息屏 / PWR 键动作没有搬进来，属于后续步骤。
// 这里只把 PWR 键的**事件读出来打日志**，不绑定任何动作 —— 先确认这条线通不通。

#include <BLEDevice.h>
#include <Preferences.h>

#include "BoardAudio.h"
#include "BoardDisplay.h"
#include "BoardPower.h"
#include "BoardTouch.h"
#include "CodexMicroBle.h"
#include "BoardKeys.h"
#include "CompletionBanner.h"
#include "DashboardUi.h"

BoardDisplay display;
BoardTouch touch;
BoardPower power;
BoardKeys keys;
BoardAudio audio;
CodexMicroBle ble;

// ---------------- 交互常量 ----------------
//
// 这一组数字**全部对齐上游**，因为它们是踩过坑之后定下来的，不能按屏幕尺寸
// 等比缩放 —— 抖动是数字化仪的电气特性，跟屏上有多少像素无关。
//
// 上一版我把三个值都改小了（swipe 80 / same-spot 30 / lockout 350），理由是
// "368 比 466 窄，按比例折一下"。实测证明这个推理是错的：
//   · 点表盘被偷成滑动（swipe 阈值比上游的 100 还低 20%）
//   · 一次点按提交两次（same-spot 只有上游的 30%，幻触落在 41px 外就放行了）
constexpr uint32_t kTouchKeyPulseMs = 55;  // 按下→抬起之间的保持时长
constexpr uint32_t kMinTapHoldMs = 40;     // 短于这个算幻触；上游 20，本板取 40
// 提交后的幻触抑制窗口。1.75C 的 CST9217 一次物理点按会报好几轮
// press/release，每轮都提交一次按键，于是 ChatGPT 窗口被反复触发。
constexpr uint32_t kCommitLockoutMs = 600;
// "同一个位置"的判据：只有落在上一次提交点周围**这个半径**内的 press
// 才当幻触吞掉；落到别处（另一个键、表盘）算真正的新操作，立刻放行。
// 100 是上游实测值，本项目实测复核有效：真幻触全在 40px 内。
// 注意必须按**欧氏半径**比，不能逐轴比（见 beginGesture 里的注释）。
constexpr int kPhantomSameSpotPx = 100;
// 滑动阈值。上游从 52 抬到 100 的原因就写在它的注释里：
// "1.75C 的触摸数字化仪抖动到足以让表盘上的一次点按漂过 52px，
//  被滑动分类器偷成摇杆手势、把 Send 取消掉。100 仍能识别刻意的滑动，
//  同时让中心点按稳定。"
constexpr int kSwipeThresholdPx = 100;
// 连发抑制：2 秒内提交满 4 次就当成"表放在桌上被幻触刷屏"，静默 3 秒。
constexpr uint32_t kNoiseBurstWindowMs = 2000;
constexpr int kNoiseBurstThreshold = 4;
constexpr uint32_t kNoiseSuppressMs = 3000;

constexpr uint32_t kStatusIntervalMs = 5000;
constexpr uint32_t kMinRedrawGapMs = 120;
constexpr uint32_t kPeriodicRedrawMs = 2000;
constexpr uint32_t kQuotaStaleAfterMs = 180000;

const char* kAgentKeys[] = {"AG00", "AG01", "AG02", "AG03", "AG04", "AG05"};
constexpr char kSendKey[] = "ACT12";

// ---------------- ⑤ 物理按键（照上游 1.75C 的定义搬） ----------------
// 上游 `_src/src__main.cpp` 的 updateBoardButtons() 里：
//   BOOT（GPIO0，低有效）= 按住说话（Push to talk），发 ACT10 的按下/抬起
//   PWR （AXP2101 PEK）  = 短按切"语音聊天"，发一颗 70ms 脉冲
//   PWR 长按             = 上游是 travel power off；本板**软件不接管**，
//                          因为 AXP2101 硬件约 6 秒就会自己关机，插手反而抢时序。
//
// ⚠️ **键名只是"插槽编号"，插槽里装什么由 ChatGPT 桌面端的键位配置决定。**
//    实测踩坑：上游注释的 ACT09（第 4 号 Command Key）在**官方出厂默认**布局里绑的是
//    "分支键帽 = 在新对话中继续当前对话" —— 老胡短按 PWR 直接把对话分叉了。
//    官方默认布局（learn.chatgpt.com/docs/features/codex-micro，中文镜像 codex-docs.com）：
//      快速模式 / 批准 / 拒绝 / **分支(在新对话中继续)** / 麦克风(按住说话) / Codex(发送)
//    正解：桌面端 → 设置 → Codex Micro → 打开「**使用独立麦克风按键**」，
//    那颗宽麦克风键底下的两个开关就能分别映射；把其中一个设成「**语音聊天**」，
//    本固件 PWR 键发它对应的编号即对上（BOOT 仍发 ACT10 = 按住说话）。
//    哪个开关对应哪个 ACT 编号只能靠实测 ⇒ 所以这里**既要默认值、也要 `vc` 命令**：
//    宁可先发一个没人认领的 ACT11（无副作用），也绝不再发 ACT09（按下去会分叉对话）。
constexpr int kBootButtonPin = 0;
constexpr uint32_t kBootButtonDebounceMs = 40;  // 上游同值
constexpr char kPttKey[] = "ACT10";             // 上游 kLeftMicSwitch
char gVoiceChatKey[8] = "ACT11";  // 运行时可改：`vc ACTxx`；默认避开被"分支"占用的 ACT09
constexpr uint32_t kVoiceClickPulseMs = 70;     // 上游 kVoiceClickPulseMs
constexpr uint32_t kSimPttHoldMs = 150;         // `n` 模拟按住说话的时长

// 按键的状态与函数都必须先于 handleSerial 可见 —— 命令 `n`/`y` 要用它们。
// ⚠️ **变量也要放这里**：函数有原型可以先声明，变量没有原型，
// 放到后面就是编译错 `'xxx' was not declared in this scope`（踩过一次）。
bool pttDown = false;             // 已发 ACT10=1、还没发 0（真按键与 `n` 模拟共用）
bool bootRawPressed = false;      // BOOT 脚消抖后的物理状态（只在真按键时变）
uint32_t bootChangedAtMs = 0;
uint32_t pttAutoReleaseAtMs = 0;  // 非 0 = 到点自动抬（`n` 模拟用）
bool voiceClickReleasePending = false;  // ACT09 那颗 70ms 脉冲还没收尾
uint32_t voiceClickReleaseAtMs = 0;

void pttPress();
void pttRelease();
void beginVoiceChatClick();
void updateVoiceChatClick();
void pollBootButton();
void pollSimPtt();

// 上游 touch_gesture::normalizedAngle() 的四个值，按本文件的 dir 编号排：
// 0=上 1=右 2=下 3=左
constexpr float kSwipeAngles[4] = {0.75f, 0.00f, 0.25f, 0.50f};
const char* kSwipeNames[4] = {"UP", "RIGHT", "DOWN", "LEFT"};

// ---------------- 交互状态 ----------------
bool touchTracking = false;
bool wasDown = false;
bool agentPressed = false;
bool sendPressed = false;
int8_t activeAgent = -1;
int16_t touchStartX = 0;
int16_t touchStartY = 0;
uint32_t touchBeganAtMs = 0;
uint32_t lockoutUntilMs = 0;
int16_t lastCommitX = -1;
int16_t lastCommitY = -1;
int8_t activeSwipe = -1;  // -1 = 无，0..3 = 上/右/下/左

// 连发抑制状态（上游 suppressNoiseBurst 的三个变量，外加"同区域"判定）
uint32_t lastTouchCommitAtMs = 0;
int touchCommitCount = 0;
uint32_t touchSuppressUntilMs = 0;
// 上一次提交落在哪个区域：-1 无 / 0..5 = A1..A6 / 6 = 表盘。
// 用途见 suppressNoiseBurst()：只有**同一个区域**反复提交才算噪点。
int8_t lastNoiseRegion = -1;

// v1 打开：串口打每一个移动样本（含 dx/dy/距离），屏上画触点十字。
// 用途是回答两个只能靠数据回答的问题：
//   1) 命中测试对不对齐 —— 屏上十字画在芯片报的位置，跟手指按的位置一比就知道
//   2) 滑动阈值该取多少 —— 看"点一下"实际能漂多少像素，而不是靠猜
bool touchDebug = false;

// ---------------- 触摸校准（串口 c / z0 / z1） ----------------
// 起因：v1 的十字"跟着手指走，但有偏移" —— 说明芯片报的坐标与屏幕坐标之间
// 存在一层**系统性偏差**。这既不是命中半径的问题，也不是"手指落点不准"，
// 得先把它量出来才能决定怎么修。
//
// 量法：屏幕上依次高亮 A1..A6 的圆心，老胡逐个点它们的**正中心**；
// 每个键心于是贡献一个"已知屏幕坐标 -> 芯片原始坐标"的对应点，
// 6 个点足够最小二乘拟合出 raw -> screen 的一次变换（平移 + 缩放 + 剪切/旋转）。
//
// 为什么拿六个键心当靶点，而不是画一堆标定图标：
// 拟合出的修正在**真正要点的那些位置上**误差最小。
// 【为什么是 4 参数模型，而不是完整仿射】
// 第一版用了 6 参数完整仿射（含剪切/旋转），6 个点 6 个未知数 —— 结果 rms=60px，
// 比"不修"还差：参数太多 + 单点噪声大到 ±30px = 纯粹过拟合。
// 实测数据反而指向一个**物理上很干净**的模型：raw = a*screen + b，**每个轴各一组**
// （无旋转、无剪切）。证据是四个方向**都往外偏**、中心几乎不偏：
//   A1 顶部 raw_y 比屏幕 y 小 38（更往上）   A4 底部 y 大 22（更往下）
//   A3 右侧 x 大 24                        A5/A6 左侧 x 小 25
// ⇒ 数字仪的增益比屏大（坐标被放大约 15~20%）。
// 4 个未知数用 6 个点拟合，过拟合风险大幅下降；而且模型可逆、可解释。
constexpr int kCalPoints = 6;
constexpr int kCalRepeats = 3;  // 每个键点 3 次：取中位数抗单点失手，并量出重复性
struct CalPoint {
  int16_t tx, ty;  // 屏幕上的靶点（该键的圆心）
  int16_t rx[kCalRepeats];
  int16_t ry[kCalRepeats];
  int n;
};
CalPoint calPts[kCalPoints];
int8_t calIndex = -1;  // -1 = 不在校准；0..5 = 正在等第 N 个键
int calRep = 0;        // 当前键已点了几次

// raw -> screen 的 4 参数模型：x = (rx - calCx) / calKx ，y = (ry - calCy) / calKy
// （calKx/calKy = 数字仪相对屏的增益；1.0 = 完全一致）
//
// 【出厂默认值 = 本板实测】2026-09-24，六键各点 3 次取中位后做最小二乘：
//   raw_x = 1.1374*sx - 17.3     raw_y = 1.1974*sy - 42.1
//   未修正时六键偏差 24~40px；修正后残差 rms=7.84px / worst=9.82px。
// 物理含义：CST820 数字仪的坐标增益比屏大 14%(x) / 20%(y)，**四个方向都朝外偏、
// 屏心几乎不偏** —— 这是矢量缩放而不是平移，所以调命中半径糊不过去。
// ⚠️ 这几个数字是**这块 1.8 板**的实测值。换屏/换板之后要么重新 `c` 校准
// （成功后自动写 NVS，掉电不丢），要么直接把这里四个常量改掉。
constexpr double kCalDefKx = 1.1374;
constexpr double kCalDefCx = -17.3;
constexpr double kCalDefKy = 1.1974;
constexpr double kCalDefCy = -42.1;

double calKx = kCalDefKx, calCx = kCalDefCx, calKy = kCalDefKy, calCy = kCalDefCy;
bool calFitOk = true;     // 出厂默认值本身就是一套可用拟合，故开机即为 true
bool calApplied = true;   // 开机即应用：省掉"每次上电都得手工 z1"这一步
bool calFromNvs = false;  // 当前常量是否来自 NVS（= 曾经 c 校准过）

// ---------------- 校准常量的持久化（NVS） ----------------
// 目的：**校准一次就永久生效**，不用每次上电都重新点 18 下。
// 出厂默认值只作兜底 —— 换板后重新 `c` 一次，NVS 里的值会盖掉它。
// 命名空间上限 15 字符，"touchcal" 安全。
constexpr const char* kCalNvsNs = "touchcal";

void saveCalToNvs() {
  Preferences p;
  if (!p.begin(kCalNvsNs, /*readOnly=*/false)) {
    Serial.println("[cal] NVS 打开失败，本次校准只保留在内存里");
    return;
  }
  p.putDouble("kx", calKx);
  p.putDouble("cx", calCx);
  p.putDouble("ky", calKy);
  p.putDouble("cy", calCy);
  p.end();
  Serial.println("[cal] 已写入 NVS，掉电后仍然生效");
}

void loadCalFromNvs() {
  Preferences p;
  if (!p.begin(kCalNvsNs, /*readOnly=*/true)) return;
  if (!p.isKey("kx")) {  // 从没校准过：保持出厂默认
    p.end();
    return;
  }
  calKx = p.getDouble("kx", kCalDefKx);
  calCx = p.getDouble("cx", kCalDefCx);
  calKy = p.getDouble("ky", kCalDefKy);
  calCy = p.getDouble("cy", kCalDefCy);
  p.end();
  calFromNvs = true;
}

// z2：丢弃 NVS 里的校准，回到出厂默认值。
void resetCalToDefault() {
  Preferences p;
  if (p.begin(kCalNvsNs, /*readOnly=*/false)) {
    p.clear();
    p.end();
  }
  calKx = kCalDefKx;
  calCx = kCalDefCx;
  calKy = kCalDefKy;
  calCy = kCalDefCy;
  calFromNvs = false;
  calFitOk = true;
  calApplied = true;
}

// 最近一次按下实际使用的屏幕坐标（未修正 or 修正后）。v1 的十字画在这里 ——
// 于是"关修正 / 开修正"的差别是**肉眼直接可见**的：修正对了，十字就落在手指下。
int16_t uiTouchX = 0;
int16_t uiTouchY = 0;

// 取某一轴三次读数的中位数（抗单次失手；三次取中位比取均值稳）。
int16_t median3(const int16_t* v) {
  int16_t a = v[0], b = v[1], c = v[2];
  if (a > b) {
    const int16_t t = a;
    a = b;
    b = t;
  }
  if (b > c) {
    const int16_t t = b;
    b = c;
    c = t;
  }
  if (a > b) {
    const int16_t t = a;
    a = b;
    b = t;
  }
  return b;
}

// 单轴最小二乘：raw = k*screen + c（对 n 个 (screen, raw) 点）
void fitAxis(const int16_t* screen, const int16_t* raw, int n, double* k,
             double* c) {
  double ss = 0, sr = 0, s = 0, r = 0;
  for (int i = 0; i < n; ++i) {
    ss += static_cast<double>(screen[i]) * screen[i];
    sr += static_cast<double>(screen[i]) * raw[i];
    s += screen[i];
    r += raw[i];
  }
  const double den = n * ss - s * s;
  if (fabs(den) < 1e-6) {  // 所有点重合，退化成纯平移
    *k = 1.0;
    *c = (r - s) / n;
    return;
  }
  *k = (n * sr - s * r) / den;
  *c = (r - *k * s) / n;
}

void calFit() {
  int16_t sx[kCalPoints], sy[kCalPoints], rmx[kCalPoints], rmy[kCalPoints];
  int n = 0;
  double sumDx = 0, sumDy = 0;
  for (int i = 0; i < kCalPoints; ++i) {
    if (calPts[i].n < kCalRepeats) continue;
    sx[n] = calPts[i].tx;
    sy[n] = calPts[i].ty;
    rmx[n] = median3(calPts[i].rx);
    rmy[n] = median3(calPts[i].ry);
    sumDx += (rmx[n] - calPts[i].tx);
    sumDy += (rmy[n] - calPts[i].ty);
    ++n;
  }
  if (n < 2) {
    Serial.printf("[cal] 有效点不足 n=%d，无法拟合\n", n);
    calFitOk = false;
    return;
  }

  fitAxis(sx, rmx, n, &calKx, &calCx);
  fitAxis(sy, rmy, n, &calKy, &calCy);

  // 合理性检查：增益偏离 1 太远说明取点有问题（点错了键/手滑），
  // 这种拟合结果宁可不用，也不要把修正常量灌进去。
  if (calKx < 0.6 || calKx > 1.6 || calKy < 0.6 || calKy > 1.6) {
    Serial.printf("[cal] 拟合结果不合理（增益 %.3f / %.3f 超出 0.6~1.6），"
                  "已丢弃。请重跑 c 并尽量点键的正中心\n",
                  calKx, calKy);
    calFitOk = false;
    return;
  }
  calFitOk = true;
  calApplied = true;   // 拟合成功就立刻启用，省掉手工 z1 这一步
  calFromNvs = false;
  saveCalToNvs();      // 写 NVS —— 让这次校准掉电后依然生效

  Serial.printf("[cal] 六个键的中位偏差均值 = (%+.1f, %+.1f) px\n", sumDx / n,
                sumDy / n);

  // 逐点残差：用拟合出来的模型去预测每个键心，与真实键心的距离。
  double rms = 0, worst = 0;
  char res[96];
  int off = 0;
  for (int i = 0; i < n; ++i) {
    const double px = (rmx[i] - calCx) / calKx;
    const double py = (rmy[i] - calCy) / calKy;
    const double ex = px - sx[i], ey = py - sy[i];
    const double e = sqrt(ex * ex + ey * ey);
    rms += e * e;
    if (e > worst) worst = e;
    off += snprintf(res + off, sizeof(res) - off, "%.1f ", e);
  }
  rms = sqrt(rms / n);

  Serial.printf("[cal] model  raw_x = %.4f*sx %+.1f   raw_y = %.4f*sy %+.1f\n",
                calKx, calCx, calKy, calCy);
  Serial.printf("[cal] 数字仪增益 = %.3f (x) / %.3f (y)  ← 1.000 = 与屏完全一致\n",
                calKx, calKy);
  Serial.printf("[cal] 拟合残差 rms=%.2fpx worst=%.2fpx  每点: %s\n", rms, worst,
                res);
  Serial.println(
      "[cal] z1 应用 / z0 关掉；v1 开着时十字画在修正后的位置（青色）");
}

// ---------------- 电量（⑤：改成从 AXP2101 真读） ----------------
// ④ 里 batteryPercent 写死 100。现在由 BoardPower 每 2 秒读一次真值：
//   · 接电池 -> 0..100
//   · 没接电池 / 读不到芯片 -> -1（屏上显示 "--%"）—— 故意不兜底成 100，
//     否则"插着 USB 的裸板"和"电池真充满"在屏上完全一样。
// batteryOverride 只给串口 `b` 用（想看不同电量下的版式，或对比 BLE 上报）。
int8_t batteryPercent = -1;
bool charging = false;
bool docked = false;
int8_t batteryOverride = -1;
// 上一次"报给主机"的电量。只在变了的时候才 setBattery —— 遥测函数每个
// loop（5ms）都被调一次，无条件上报等于往 BLE 里灌毫无意义的流量。
int8_t lastReportedBattery = -2;
bool lastReportedCharging = false;

// 完成横幅 + 提示音。上游 kCompletionBannerMs = 5000。
constexpr uint32_t kCompletionBannerMs = 5000;
completion_banner::Timer completionBanner;

// Agent 状态跃迁检测（移植上游 detectAgentTransitions）。
// 关键是"上一次不是 Complete、这一次是 Complete"才算**刚**完成 ——
// 主机大约 1 分钟轮询一次 device.status，若只看当前状态，
// 一个一直停在 Complete 的 Agent 会被反复当成刚跑完，提示音响个不停。
dashboard::AgentStatus lastAgentStatus[6];
bool agentStatusSeen[6] = {false, false, false, false, false, false};

uint32_t lastStatusAtMs = 0;
uint32_t lastRedrawAtMs = 0;
bool redrawPending = true;
uint32_t touchCommits = 0;

// ---------------- 屏自检（驻留循环版，同 step2/3） ----------------
constexpr uint32_t kSelfTestFrameMs = 1500;
constexpr int kSelfTestFrames = 4;
constexpr int kSelfTestRounds = 2;
constexpr uint32_t kSelfTestTotalMs =
    kSelfTestFrameMs * kSelfTestFrames * kSelfTestRounds;

uint32_t selfTestEndAtMs = 0;
uint32_t selfTestNextFrameAtMs = 0;
int selfTestFrameIndex = 0;

void drawSelfTestFrame(int index) {
  Arduino_GFX* g = display.gfx();
  switch (index) {
    case 0:
      Serial.println("[screen] frame 1/4 PURE RED");
      g->fillScreen(RGB565_RED);
      break;
    case 1:
      Serial.println("[screen] frame 2/4 PURE GREEN");
      g->fillScreen(RGB565_GREEN);
      break;
    case 2:
      Serial.println("[screen] frame 3/4 PURE BLUE");
      g->fillScreen(RGB565_BLUE);
      break;
    default: {
      Serial.println("[screen] frame 4/4 BARS top->bottom "
                     "R,G,B,WHITE,BLACK,YELLOW,CYAN,MAGENTA");
      const uint16_t bars[] = {RGB565_RED,   RGB565_GREEN, RGB565_BLUE,
                               RGB565_WHITE, RGB565_BLACK, RGB565_YELLOW,
                               RGB565_CYAN,  RGB565_MAGENTA};
      const int16_t h = BoardDisplay::kHeight / 8;
      for (int i = 0; i < 8; ++i) {
        g->fillRect(0, i * h, BoardDisplay::kWidth, h, bars[i]);
      }
      g->drawRect(0, 0, BoardDisplay::kWidth, BoardDisplay::kHeight,
                  RGB565_WHITE);
      break;
    }
  }
  display.flush();
}

void beginSelfTest() {
  Serial.printf("[screen] self test start: %d frames x %lums x %d rounds "
                "(~%lus)\n",
                kSelfTestFrames, static_cast<unsigned long>(kSelfTestFrameMs),
                kSelfTestRounds,
                static_cast<unsigned long>(kSelfTestTotalMs / 1000));
  selfTestFrameIndex = 0;
  const uint32_t now = millis();
  selfTestEndAtMs = now + kSelfTestTotalMs;
  selfTestNextFrameAtMs = now;
}

void pollSelfTest() {
  if (selfTestEndAtMs == 0) return;
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - selfTestEndAtMs) >= 0) {
    selfTestEndAtMs = 0;
    Serial.println("[screen] self test done -> back to dashboard");
    redrawPending = true;
    return;
  }
  if (static_cast<int32_t>(now - selfTestNextFrameAtMs) >= 0) {
    drawSelfTestFrame(selfTestFrameIndex);
    selfTestFrameIndex = (selfTestFrameIndex + 1) % kSelfTestFrames;
    selfTestNextFrameAtMs = now + kSelfTestFrameMs;
  }
}

// ---------------- UI 状态组装 ----------------
uint32_t quotaResetRemaining(const QuotaState& quota) {
  if (!quota.available) return 0;
  const uint32_t elapsedSeconds = (millis() - quota.receivedAtMs) / 1000;
  return quota.resetInSeconds > elapsedSeconds
             ? quota.resetInSeconds - elapsedSeconds
             : 0;
}

dashboard::State uiState() {
  const CodexMicroState st = ble.snapshot();

  dashboard::State ui;
  ui.linkHealth = st.hostRpcObserved
                      ? dashboard::LinkHealth::CodexLive
                      : (st.connected ? dashboard::LinkHealth::BleOnly
                                      : dashboard::LinkHealth::Offline);
  ui.batteryPercent = batteryPercent;
  ui.charging = charging;
  ui.docked = docked;
  ui.quotaAvailable = st.quota.available;
  ui.quotaStale =
      st.quota.available && (millis() - st.quota.receivedAtMs) > kQuotaStaleAfterMs;
  ui.remainingPercent = st.quota.remainingPercent;
  ui.resetInSeconds = quotaResetRemaining(st.quota);
  ui.sendPressed = sendPressed;
  ui.activeTouchAgent = agentPressed ? activeAgent : -1;
  ui.swipeDirection = activeSwipe;
  // ⑤：完成横幅由 CompletionBanner 的时间窗决定（5 秒，上游同值）
  ui.completedAgent =
      completionBanner.visible(millis()) ? completionBanner.agent : -1;
  // PWR 键的长按关机流程还没做（本步只"读"PWR 键，不绑动作）
  ui.powerOverlay = dashboard::PowerOverlay::None;
  for (int i = 0; i < 6; ++i) {
    ui.threads[i].color = st.threads[i].color;
    ui.threads[i].brightness = st.threads[i].brightness;
    ui.threads[i].focused = false;
  }
  return ui;
}

void drawScreen() {
  if (selfTestEndAtMs != 0) return;  // 自检期间不抢屏
  dashboard::render(display.gfx(), uiState());
  if (touchDebug) {
    // 把"系统认为你按在哪"画在屏上。这是"命中测试对不对齐"唯一的直接证据：
    // 手指按在某个键的中心，而十字画在别处，就说明坐标有偏移（不是命中半径的问题）。
    //   · 黄色 = 未修正（画的芯片原始坐标）
    //   · 青色 = 已应用 c 命令拟合出的修正
    // 于是在同一个窗口里就能肉眼 A/B：修正对了，十字会落回手指下面。
    Arduino_GFX* g = display.gfx();
    const int16_t px = uiTouchX;
    const int16_t py = uiTouchY;
    const uint16_t c = !touch.pressed()      ? RGB565_MAGENTA
                       : (calApplied ? RGB565_CYAN : RGB565_YELLOW);
    g->drawCircle(px, py, 7, c);
    g->drawFastHLine(px - 13, py, 27, c);
    g->drawFastVLine(px, py - 13, 27, c);
  }
  // 校准模式：高亮当前该点的那个键，并在左上角标进度。
  if (calIndex >= 0 && calIndex < kCalPoints) {
    Arduino_GFX* g = display.gfx();
    const int16_t tx = dashboard::kAgentCenters[calIndex].x;
    const int16_t ty = dashboard::kAgentCenters[calIndex].y;
    g->drawCircle(tx, ty, 52, RGB565_WHITE);
    g->drawCircle(tx, ty, 49, RGB565_WHITE);
    g->drawFastHLine(tx - 14, ty, 29, RGB565_WHITE);
    g->drawFastVLine(tx, ty - 14, 29, RGB565_WHITE);
    char buf[40];
    snprintf(buf, sizeof(buf), "CAL A%d  %d/%d", calIndex + 1, calRep + 1,
             kCalRepeats);
    g->setTextSize(2);
    g->setTextColor(RGB565_WHITE);
    g->setCursor(8, 6);
    g->print(buf);
  }
  display.flush();
}

// ---------------- 触摸状态机 ----------------
void clearCandidates() {
  agentPressed = false;
  sendPressed = false;
  activeAgent = -1;
}

// 上游 suppressNoiseBurst() 的移植。返回 true = 这次提交当噪点丢掉。
// 触发条件：短时间内（2s）**同一个区域**提交满 4 次 —— 正常使用不可能这么密，
// 只可能是表放在桌上、屏被人/物压着持续产生幻触。命中后静默 3 秒。
//
// 与上游的两处刻意差异（都是实测逼出来的）：
//   1) 上游在**每次抬手**都计数；我们只在"真有候选、真够长、真要去提交"时计数 ——
//      空点一下/划一道不算 commit，按上游注释 "4 commits within 2s" 的字面语义实现。
//   2) 上游是"2 秒 4 次"不分位置，实测会把正常操作也吃掉：老胡连点
//      A1→A1→A1→表盘→表盘，第 4 次起被静默 3 秒，紧接着真正的 A1、A6 点按
//      （按住 100~120ms，够长）连丢两个。而上游注释写的是"**同一个键**被幻触刷屏"，
//      所以按**同区域**计数才是它想表达的意思：桌面压屏 → 同一个键反复提交；
//      用户换着键点 → 区域在变，那是正常操作，不该算噪声。
bool suppressNoiseBurst(int8_t region) {
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - touchSuppressUntilMs) < 0) return true;
  if (region != lastNoiseRegion ||
      now - lastTouchCommitAtMs > kNoiseBurstWindowMs) {
    touchCommitCount = 0;
  }
  lastNoiseRegion = region;
  ++touchCommitCount;
  lastTouchCommitAtMs = now;
  if (touchCommitCount >= kNoiseBurstThreshold) {
    touchSuppressUntilMs = now + kNoiseSuppressMs;
    touchCommitCount = 0;
    Serial.printf("[touch] noise burst on region %d -> suppress 3s\n", region);
    return true;
  }
  return false;
}

// 手势当前点是否还落在「按下时命中的那个区域」之内。
//
// 这是一条上游没有的**结构性护栏**，专门堵住「点表盘被偷成滑动」：
// 只要这次手势是从某个键/表盘上按下去的，就必须先"离开"这个区域，才有资格
// 被当成滑动。真滑动必然离开（从表盘出发划 100px 一定出半径 95 的圆），
// 而被偷走的那次点按不会。这样"点按被偷"在结构上就不可能发生，
// 而不是靠把阈值调高来降低概率。
//
// 已知代价：从表盘边缘朝圆心方向拖 100px 会停在圆内，此时仍算点按。
// 但"从表盘往里划"本来就不是一个有意义的手势（要划也是往外划），可接受。
bool insideStartCandidate(int16_t x, int16_t y) {
  if (agentPressed && activeAgent >= 0) {
    return dashboard::agentAtPoint(x, y) == activeAgent;
  }
  if (sendPressed) return dashboard::sendAtPoint(x, y);
  return false;
}

// x/y = 已经过 raw->screen 修正的屏幕坐标（命中测试 / 滑动判断都用它）；
// rawX/rawY = 芯片原始坐标，**只有校准取点会用到**。
//
// 【为什么非要单独传 raw】校准的目标是拟合 raw -> screen 的映射。如果取点
// 取了"已经修正过"的坐标，那么再校准一次就会把同一层修正**叠乘第二遍**
// （增益变成 1.1374² ≈ 1.294），越校越歪。这个坑在加出厂默认值之后才会
// 真正暴露：默认值开机即 applied，于是第一次 c 就会踩到。
void beginGesture(int16_t x, int16_t y, int16_t rawX, int16_t rawY) {
  const uint32_t now = millis();

  // 校准模式：这次按下只用来取点，完全不参与按键逻辑
  // （也不受 lockout / 消噪干扰 —— 取点必须每次都算数）。
  if (calIndex >= 0 && calIndex < kCalPoints) {
    CalPoint& p = calPts[calIndex];
    p.tx = dashboard::kAgentCenters[calIndex].x;
    p.ty = dashboard::kAgentCenters[calIndex].y;
    if (calRep < kCalRepeats) {
      p.rx[calRep] = rawX;  // ← 存 raw，不是修正后的 x（见函数头注释）
      p.ry[calRep] = rawY;
      ++calRep;
    }
    p.n = calRep;
    const int ddx = rawX - p.tx;
    const int ddy = rawY - p.ty;
    Serial.printf(
        "[cal] A%d 第%d/%d次 target=(%d,%d) raw=(%d,%d) d=(%+d,%+d) |d|=%.1f\n",
        calIndex + 1, calRep, kCalRepeats, p.tx, p.ty, rawX, rawY, ddx, ddy,
        static_cast<double>(sqrtf(static_cast<float>(ddx * ddx + ddy * ddy))));
    if (calRep >= kCalRepeats) {
      // 该键收满三次：打中位数与重复性（max-min）。
      // 重复性这一栏很关键 —— 它决定"偏差里有多少是手抖、有多少是系统性的"。
      const int16_t mx = median3(p.rx);
      const int16_t my = median3(p.ry);
      int16_t minX = p.rx[0], maxX = p.rx[0], minY = p.ry[0], maxY = p.ry[0];
      for (int i = 1; i < kCalRepeats; ++i) {
        if (p.rx[i] < minX) minX = p.rx[i];
        if (p.rx[i] > maxX) maxX = p.rx[i];
        if (p.ry[i] < minY) minY = p.ry[i];
        if (p.ry[i] > maxY) maxY = p.ry[i];
      }
      Serial.printf(
          "[cal] A%d 中位 raw=(%d,%d) 偏=(%+d,%+d)  重复性 max-min=(%d,%d)\n",
          calIndex + 1, mx, my, mx - p.tx, my - p.ty, maxX - minX, maxY - minY);
      ++calIndex;
      calRep = 0;
      if (calIndex >= kCalPoints) {
        calIndex = -1;
        calFit();
      }
    }
    redrawPending = true;
    return;
  }

  // 幻触抑制：CST816x 在按下后可能连着报几轮 press/release。提交后的
  // 锁定期里，**落在同一位置**的按下当残留吞掉；落到别处（另一个键、
  // 表盘）算真正的新操作，放行。持续吞掉的同时延长锁定期，
  // 免得一串慢速幻触在两个窗口之间漏出一次按键。
  if (static_cast<int32_t>(now - lockoutUntilMs) < 0) {
    // 判据用**欧氏距离**，不能逐轴比较（Chebyshev）。
    // 2026-09-24 实测抓到一次真实误吞：A6(66,130) 提交后紧接着点表盘，
    // 落点 (164,224) 被吞 —— 逐轴差 98/94 都在 100 以内，实际相距 136px，
    // 是"另一个目标"却被判成同一位置。而所有真幻触的欧氏距离都 <40px。
    // 换成欧氏距离后两类干净分开，且不需要改 100px 这个上游来的常量。
    const long ddx = static_cast<long>(x) - lastCommitX;
    const long ddy = static_cast<long>(y) - lastCommitY;
    const long d2 = ddx * ddx + ddy * ddy;
    const bool sameSpot =
        lastCommitX >= 0 &&
        d2 <= static_cast<long>(kPhantomSameSpotPx) * kPhantomSameSpotPx;
    if (sameSpot) {
      lockoutUntilMs = now + kCommitLockoutMs;
      Serial.printf("[touch] locked out x=%d y=%d d=%.0f\n", x, y,
                    static_cast<double>(sqrtf(static_cast<float>(d2))));
      return;
    }
    lockoutUntilMs = 0;
  }

  touchTracking = true;
  touchStartX = x;
  touchStartY = y;
  touchBeganAtMs = now;
  activeSwipe = -1;

  const int agent = dashboard::agentAtPoint(x, y);
  if (agent >= 0) {
    agentPressed = true;
    activeAgent = static_cast<int8_t>(agent);
  } else if (dashboard::sendAtPoint(x, y)) {
    sendPressed = true;
  }

  Serial.printf("[touch] begin x=%d y=%d agent=%d send=%d\n", x, y, agent,
                sendPressed ? 1 : 0);
  redrawPending = true;
}

void updateGesture(int16_t x, int16_t y) {
  if (!touchTracking || activeSwipe >= 0) return;

  const int dx = x - touchStartX;
  const int dy = y - touchStartY;

  if (touchDebug) {
    Serial.printf("[tmove] x=%d y=%d dx=%d dy=%d d=%d cand=%d\n", x, y, dx, dy,
                  static_cast<int>(sqrtf(static_cast<float>(dx * dx + dy * dy)) +
                                   0.5f),
                  insideStartCandidate(x, y) ? 1 : 0);
  }

  if (dx * dx + dy * dy < kSwipeThresholdPx * kSwipeThresholdPx) return;

  // 还待在起始区域里 -> 继续当按键候选，别偷成滑动（见 insideStartCandidate）。
  if (insideStartCandidate(x, y)) return;

  const int ax = dx < 0 ? -dx : dx;
  const int ay = dy < 0 ? -dy : dy;
  const int dir = (ax >= ay) ? (dx >= 0 ? 1 : 3)   // 右 / 左
                             : (dy >= 0 ? 2 : 0);  // 下 / 上

  clearCandidates();
  activeSwipe = static_cast<int8_t>(dir);
  ble.sendJoystick(kSwipeAngles[dir], 1.0f);
  Serial.printf("[touch] swipe %s angle=%.2f press (dx=%d dy=%d)\n",
                kSwipeNames[dir], kSwipeAngles[dir], dx, dy);
  redrawPending = true;
}

void finishGesture() {
  const uint32_t now = millis();

  if (activeSwipe >= 0) {
    ble.sendJoystick(kSwipeAngles[activeSwipe], 0.0f);
    Serial.printf("[touch] swipe %s release\n", kSwipeNames[activeSwipe]);
    activeSwipe = -1;
    clearCandidates();
    redrawPending = true;
    return;
  }

  // 幻触抑制：只有按住时间够长才算一次真按键。
  // （上游叫 suppressNoiseBurst，这里时长判据 + 连发抑制两道。）
  const bool heldLongEnough = (now - touchBeganAtMs) >= kMinTapHoldMs;
  // 这次手势落在哪个区域：-1 无 / 0..5 = A1..A6 / 6 = 表盘。
  int8_t region = -1;
  if (agentPressed && activeAgent >= 0) {
    region = activeAgent;
  } else if (sendPressed) {
    region = 6;
  }
  const bool willCommit = heldLongEnough && region >= 0;
  const bool noiseSuppressed = willCommit && suppressNoiseBurst(region);
  // 丢掉的两种原因要分开打 —— 混在一句 "too short" 里，会让"被静默吃掉"
  // 看起来像"按太短"，实测就是这么误判过一次。
  const char* dropReason =
      heldLongEnough ? "noise suppression" : "held too short";

  if (agentPressed && activeAgent >= 0) {
    if (heldLongEnough && !noiseSuppressed) {
      lockoutUntilMs = now + kCommitLockoutMs;
      lastCommitX = touchStartX;
      lastCommitY = touchStartY;
      ble.sendKey(kAgentKeys[activeAgent], 1, activeAgent);
      delay(kTouchKeyPulseMs);
      ble.sendKey(kAgentKeys[activeAgent], 0, activeAgent);
      ++touchCommits;
      Serial.printf("[touch] agent A%d -> %s pulse (%lu commits)\n",
                    activeAgent + 1, kAgentKeys[activeAgent],
                    static_cast<unsigned long>(touchCommits));
    } else if (!noiseSuppressed) {
      Serial.printf("[touch] agent A%d dropped (%s)\n", activeAgent + 1,
                    dropReason);
    }
  } else if (sendPressed) {
    if (heldLongEnough && !noiseSuppressed) {
      lockoutUntilMs = now + kCommitLockoutMs;
      lastCommitX = touchStartX;
      lastCommitY = touchStartY;
      ble.sendKey(kSendKey, 1);
      delay(kTouchKeyPulseMs);
      ble.sendKey(kSendKey, 0);
      ++touchCommits;
      Serial.printf("[touch] send -> %s pulse (%lu commits)\n", kSendKey,
                    static_cast<unsigned long>(touchCommits));
    } else if (!noiseSuppressed) {
      Serial.printf("[touch] send dropped (%s)\n", dropReason);
    }
  }

  clearCandidates();
  redrawPending = true;
}

void pollTouch() {
  touch.poll();
  BoardTouchSample sample;
  while (touch.takeSample(sample)) {
    // 唯一一处"原始坐标 -> 屏幕坐标"的转换点。校准关掉时它就是恒等变换，
    // 所以 8.12 那条"raw 与屏幕 1:1、无镜像、无缩放"的结论依然成立 ——
    // 这里加的只是那层**残余的系统偏差**。
    int16_t sx = sample.x;
    int16_t sy = sample.y;
    if (calApplied && calFitOk) {
      sx = static_cast<int16_t>(lround((sample.x - calCx) / calKx));
      sy = static_cast<int16_t>(lround((sample.y - calCy) / calKy));
    }
    if (sample.down) {
      uiTouchX = sx;
      uiTouchY = sy;
    }
    if (sample.down && !wasDown) {
      beginGesture(sx, sy, sample.x, sample.y);
    } else if (sample.down && wasDown) {
      updateGesture(sx, sy);
    } else if (!sample.down && wasDown) {
      finishGesture();
    }
    wasDown = sample.down;
    // debug 模式下每个样本都重绘一次，屏上的十字才会跟着手指走
    // （否则只有按键/滑动那几个状态点才重绘，十字看起来像卡住）。
    if (touchDebug) redrawPending = true;
  }
}

// ---------------- NimBLE 兜底：断开后协议栈不一定可靠地恢复广播 ----------------
void ensureAdvertising() {
  BLEServer* server = BLEDevice::getServer();
  if (server == nullptr) return;
  if (server->getConnectedCount() > 0) return;
  if (BLEDevice::getAdvertising()->isAdvertising()) return;
  Serial.println("[ble] no peer and not advertising -> restart advertising");
  BLEDevice::startAdvertising();
}

// ---------------- PWR 键三通道嗅探（排查用，不参与业务） ----------------
//
// 背景：本步按"PWR 键接在 AXP2101 PWRON 上"实现了 PMU 通道，实测按下去
// 0x49(INTSTS2) 一位都不动 —— 连 bit1(PWRON 负沿) 都不置位。翻 1.8 官方
// 原理图发现板上除 AXP2101 外还挂了一颗 **TCA9554PWR**（U7 "I/O
// Extensions"，引脚直接叫 EXIO0..EXIO7），PocketOracle 里那条
// "PWR 键可能挂在 EXIO4 上"的后备通道指的就是它。
//
// 与其猜是"PMU 没配好"还是"压根不在这条线上"，不如把三条候选通道同时
// 挂上监听，让**按下这一下**自己说明它是谁：
//   ① AXP2101 0x48/0x49/0x4A（PMU 中断状态，RW1C：读不清、写 1 才清）
//   ② TCA9554 0x00/0x01（扩展 IO 输入端口）
//   ③ ESP32 上本工程没用到的那些 GPIO（万一还有第三种走线）
//
// 关键设计：把 **GPIO0(BOOT)** 也放进 ③ 当**对照组**。按 BOOT 应该在 ③ 里
// 看到 GPIO0 跳变 —— 看不见就说明 ③ 这个测法本身有问题，而不是"信号不存在"。
// 用对照组把"测法失效"和"真没信号"分开，比事后隔空争论省事得多。
void i2cScanBus() {
  Serial.print("[i2c] 总线扫描:");
  int n = 0;
  for (uint8_t a = 0x08; a < 0x78; ++a) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" 0x%02X", a);
      ++n;
    }
  }
  Serial.printf("   共 %d 个\n", n);
}

// ---------------- 事件环：把"时机"从排查里彻底拿掉 ----------------
// ④ 的教训：给时间窗口 = 把时机变成变量 = 排查变成猜（当时给了 900 秒窗口，
// 窗口里没人点屏，于是拿到 900 秒零事件，白排查好几轮）。
// "常驻实时打印"只解决了一半问题 —— 另一半是：**没人正在抓串口时，那行日志
// 就永远丢了**。所以每条事件同时进一个环形缓冲：老胡什么时候按都行，按完不必
// 配合我，之后任何时刻敲 `xl` 都能把开机以来的历史整条倒出来。
constexpr uint8_t kEvtMax = 24;
char evtLog[kEvtMax][120];
uint8_t evtHead = 0;    // 下一条写到哪
uint8_t evtCount = 0;   // 有效条数（最多 kEvtMax）

void evtPush(const char* line) {
  snprintf(evtLog[evtHead], sizeof(evtLog[0]), "%s", line);
  evtHead = static_cast<uint8_t>((evtHead + 1) % kEvtMax);
  if (evtCount < kEvtMax) ++evtCount;
  Serial.println(line);  // 有人在实时抓的时候照样实时可见
}

// AXP2101 中断状态位的中文名。位定义来自 XPowersLib 的
// XPowersParams.hpp → xpowers_axp2101_irq_t：使能寄存器 0x40/0x41/0x42 的低 8 位
// 一一对应状态寄存器 0x48/0x49/0x4A 的 8 位。
//
// 【为什么必须打名字，不能只打十六进制】
// 光看 `0x4A=0x10` 完全看不出是什么；而查表最容易踩的坑是**把 0x4A 当成
// "PWR 键相关"** —— 0x4A bit4 其实是 **BAT_CHG_DONE（充电完成）**。
// 2026-09-24 就此误判过一轮：一条 `0x4A=0x10` + 扩展 IO 同时跳变，看起来
// 像"按到了某个键"，实际是电池充饱了（随后 chg 也由 1 变 0）。
const char* const kIrq1Names[8] = {"BAT_UT",   "BAT_OT",  "CHG_UT",  "CHG_OT",
                                   "SOC_NEW",  "WDT_TO",  "SOC_WL1", "SOC_WL2"};
const char* const kIrq2Names[8] = {"PWR_POS",  "PWR_NEG", "PWR_LONG", "PWR_SHORT",
                                   "BAT_OUT",  "BAT_IN",  "VBUS_OUT", "VBUS_IN"};
const char* const kIrq3Names[8] = {"BAT_OVP",  "CHG_TIMER", "DIE_OT", "CHG_START",
                                   "CHG_DONE", "BATFET_OCP", "LDO_OCP", "WDT_EXP"};

// 把 0x48/0x49/0x4A 里所有置位的位翻译成 "0x4A:CHG_DONE" 这种形式。
void irqNames(const uint8_t* sts, char* out, size_t n) {
  const char* const* tbl[3] = {kIrq1Names, kIrq2Names, kIrq3Names};
  size_t used = 0;
  out[0] = '\0';
  for (uint8_t r = 0; r < 3; ++r) {
    for (uint8_t b = 0; b < 8; ++b) {
      if (((sts[r] >> b) & 1) == 0) continue;
      const int w = snprintf(out + used, n - used, "%s0x%02X:%s",
                             used == 0 ? "" : " ",
                             static_cast<unsigned>(BoardPower::kRegIntsts1 + r),
                             tbl[r][b]);
      if (w < 0 || static_cast<size_t>(w) >= n - used) return;
      used += static_cast<size_t>(w);
    }
  }
}

// GPIO 嗅探名单：**只放本工程确实没用到的脚**。排除理由（碰这些会出事）：
//   26..37              闪存 / 八线 PSRAM（碰了直接崩）
//   19,20               原生 USB（串口就死在这上面）
//   43,44               UART0 控制台
//   4,5,6,7,11,12       屏 QSPI
//   14,15               I2C（触摸/PMU/音频都在这条线上）
//   21                  触摸 INT
//   8,9,10,16,45,46     I2S / 功放使能
// 名单里只剩下面这些（0 号留着做对照组，只读不写）。
const uint8_t kSniffPins[] = {0, 1, 2, 3, 13, 17, 18, 38, 39, 40, 41, 42, 47};
constexpr uint8_t kSniffCount = sizeof(kSniffPins) / sizeof(kSniffPins[0]);

uint32_t monTickMs = 0;
uint32_t monStartMs = 0;
uint8_t monPort0 = 0;
uint8_t monPort1 = 0;
uint8_t monSts[3] = {};
uint32_t monGpio = 0;    // 位图：第 i 位 = kSniffPins[i] 读到低电平
bool monPrimed = false;

void sniffSampleGpio(uint32_t* mask) {
  uint32_t m = 0;
  for (uint8_t i = 0; i < kSniffCount; ++i) {
    if (digitalRead(kSniffPins[i]) == LOW) m |= (1u << i);
  }
  *mask = m;
}

// 嗅探是**常驻**的：开机就听，只在"电平真的变了"时才打印。
// 为什么不给时间窗口：④ 的触摸调试已经踩过一次"给了 900 秒窗口、窗口里
// 没人点屏、于是得到 900 秒零事件"的坑 —— 让时机变成变量，排查就变成猜。
// 常驻之后，老胡什么时候按都行，按了就有日志。
bool pwrWatch = true;
uint32_t monEvents = 0;   // 一共看到过几次跳变（用来判断"在听但没动静"）
uint32_t monBeatMs = 0;   // 心跳计时：串口里能分清"在听"和"根本没在听"

void sniffBeginGpio() {
  for (uint8_t i = 0; i < kSniffCount; ++i) {
    pinMode(kSniffPins[i], INPUT_PULLUP);  // 只读；上拉是为了让"没接"也有确定电平
  }
  monStartMs = millis();
  monTickMs = 0;
  monPrimed = false;
  monBeatMs = monStartMs;
}

void setPwrWatch(bool on) {
  pwrWatch = on;
  monPrimed = false;  // 重新开时把当前电平当新基线，避免把开关瞬间当按键
  Serial.printf("[x] PWR 键嗅探 = %s（PMU=%d 扩展IO=%d GPIO %u 个脚）\n", on ? "开" : "关",
                power.ready() ? 1 : 0, keys.ready() ? 1 : 0,
                static_cast<unsigned>(kSniffCount));
  if (on) {
    Serial.println("[x] 现在按一下 PWR 键；再按一下 BOOT 键当对照（应当在 G0 上看到跳变）");
  }
}

void pollPwrMonitor() {
  if (!pwrWatch) return;
  const uint32_t now = millis();
  if (monTickMs != 0 && (now - monTickMs) < 50) return;
  monTickMs = now;
  const uint32_t rel = now - monStartMs;

  uint8_t p0 = 0;
  uint8_t p1 = 0;
  const bool haveKeys = keys.ready() && keys.readPorts(&p0, &p1);
  uint8_t sts[3] = {};
  const bool haveSts = power.readRegs(BoardPower::kRegIntsts1, sts, sizeof(sts));
  uint32_t gpio = 0;
  sniffSampleGpio(&gpio);

  if (!monPrimed) {
    // 第一拍只记基线，不报"变化" —— 否则开场就是一堆假跳变。
    monPrimed = true;
    monPort0 = p0;
    monPort1 = p1;
    monGpio = gpio;
    memcpy(monSts, sts, sizeof(sts));
    Serial.printf(
        "[x] 监听中（常驻+留痕）。基线：扩展IO port0=0x%02X port1=0x%02X | PMU %02X/%02X/%02X | GPIO=%04lX\n",
        p0, p1, sts[0], sts[1], sts[2], static_cast<unsigned long>(gpio));
    Serial.println(
        "[x] 按 PWR / BOOT 键**不需要配合我**（0.2~0.5s，别按住 6s 以上会关机）；"
        "随时敲 xl 倒出历史。");
    return;
  }

  if (haveKeys && (p0 != monPort0 || p1 != monPort1)) {
    char line[120];
    int n = snprintf(line, sizeof(line),
                     "[x] +%lums [扩展IO 0x%02X] port0=0x%02X port1=0x%02X",
                     static_cast<unsigned long>(rel), keys.addr(), p0, p1);
    for (uint8_t b = 0; b < 8; ++b) {
      const uint8_t was = (monPort0 >> b) & 1;
      const uint8_t is = (p0 >> b) & 1;
      if (was == is) continue;
      if (n < 0 || n >= static_cast<int>(sizeof(line)) - 16) break;
      n += snprintf(line + n, sizeof(line) - n, "  EXIO%u:%u->%u", b, was, is);
    }
    evtPush(line);
    monPort0 = p0;
    monPort1 = p1;
    ++monEvents;
    monBeatMs = now;
  }

  if (haveSts && memcmp(sts, monSts, sizeof(sts)) != 0) {
    // ⚠️ 0x49 这一格通常恒为 0 —— 不是"没事件"，而是本循环里
    //    pollPowerButton() 排在 pollPwrMonitor() **之前**，它读 0x49 时会
    //    顺手把位清掉（RW1C）。PEK 事件走的是 `[pmu] PWR 键事件` 那条线，
    //    那条线现在也进事件环，所以不会丢。
    char names[64] = {};
    irqNames(sts, names, sizeof(names));
    char line[160];
    snprintf(line, sizeof(line), "[x] +%lums [PMU] 0x48=0x%02X 0x49=0x%02X 0x4A=0x%02X%s%s",
             static_cast<unsigned long>(rel), sts[0], sts[1], sts[2],
             names[0] == '\0' ? "" : "  ", names);
    evtPush(line);
    memcpy(monSts, sts, sizeof(sts));
    ++monEvents;
    monBeatMs = now;
  }

  if (gpio != monGpio) {
    char line[160];
    int n = snprintf(line, sizeof(line), "[x] +%lums [GPIO]",
                     static_cast<unsigned long>(rel));
    for (uint8_t i = 0; i < kSniffCount; ++i) {
      const uint32_t bit = 1u << i;
      if (((gpio ^ monGpio) & bit) == 0) continue;
      if (n < 0 || n >= static_cast<int>(sizeof(line)) - 16) break;
      // 位图里的 1 = 读到低电平，打印成"电平"更直观（0=低 1=高）
      n += snprintf(line + n, sizeof(line) - n, " G%u:%u->%u", kSniffPins[i],
                    (monGpio & bit) ? 0 : 1, (gpio & bit) ? 0 : 1);
    }
    evtPush(line);
    monGpio = gpio;
    ++monEvents;
    monBeatMs = now;
  }

  // 心跳：60 秒没有事件就报一次"还在听"，并把这四条线**当前的电平**一起打。
  // 为什么连电平一起打：光有"跳变历史"分不清"按完又抬起"和"一直压着"；
  // 而光有"当前电平"又分不清"从没动过"和"动过又回来了"。两个一起看才闭合。
  // ⚠️ 心跳**只打串口、不进事件环** —— 否则长跑时心跳会把真正的按键事件挤出去。
  if (now - monBeatMs >= 60000) {
    monBeatMs = now;
    uint8_t b0 = 0;
    uint8_t b1 = 0;
    const bool haveK = keys.ready() && keys.readPorts(&b0, &b1);
    uint8_t bSts[3] = {};
    const bool haveS = power.readRegs(BoardPower::kRegIntsts1, bSts, sizeof(bSts));
    uint32_t bGpio = 0;
    sniffSampleGpio(&bGpio);
    Serial.printf(
        "[x] 监听中 +%lus 已见 %lu 次事件 | 当场电平 扩展IO %s0x%02X(4=%u 5=%u) "
        "PMU %s%02X/%02X/%02X GPIO=%04lX | 敲 xl 看历史\n",
        static_cast<unsigned long>(rel / 1000),
        static_cast<unsigned long>(monEvents), haveK ? "" : "?", b0,
        (b0 >> 4) & 1, (b0 >> 5) & 1, haveS ? "" : "?", bSts[0], bSts[1], bSts[2],
        static_cast<unsigned long>(bGpio));
  }
}

// `xl`：把事件环整条倒出来，再补一张"当场电平"快照。
void dumpEventLog() {
  Serial.printf("[x] 事件历史 %u/%u 条（最早的在前，相对于开机）\n", evtCount,
                static_cast<unsigned>(kEvtMax));
  if (evtCount == 0) {
    Serial.println(
        "[x] 一条都没有 = 这段时间三条通道都没动过（不是没在听，心跳一直在）");
  }
  const uint8_t first =
      static_cast<uint8_t>((evtHead + kEvtMax - evtCount) % kEvtMax);
  for (uint8_t i = 0; i < evtCount; ++i) {
    Serial.printf("[x]  #%u %s\n", static_cast<unsigned>(i + 1),
                  evtLog[(first + i) % kEvtMax]);
  }

  uint8_t p0 = 0;
  uint8_t p1 = 0;
  if (keys.ready() && keys.readPorts(&p0, &p1)) {
    Serial.printf("[x] 当场 扩展IO 0x%02X config=0x%02X(0xFF=全输入) port1=0x%02X:",
                  keys.addr(), keys.config(), p1);
    for (uint8_t b = 0; b < 8; ++b) Serial.printf(" EXIO%u=%u", b, (p0 >> b) & 1);
    Serial.println();
  } else {
    Serial.println("[x] 当场 扩展IO 读不到（0x20..0x27 无应答）");
  }

  uint8_t irq[3] = {};
  if (power.readRegs(BoardPower::kRegIntsts1, irq, sizeof(irq))) {
    char names[64] = {};
    irqNames(irq, names, sizeof(names));
    Serial.printf("[x] 当场 PMU 0x48=0x%02X 0x49=0x%02X 0x4A=0x%02X%s%s\n", irq[0],
                  irq[1], irq[2], names[0] == '\0' ? "" : "  ", names);
  } else {
    Serial.println("[x] 当场 PMU 读不到（0x34 无应答）");
  }

  uint32_t gpio = 0;
  sniffSampleGpio(&gpio);
  Serial.print("[x] 当场 GPIO(电平，1=高):");
  for (uint8_t i = 0; i < kSniffCount; ++i) {
    Serial.printf(" G%u=%u", kSniffPins[i], ((gpio >> i) & 1) ? 0 : 1);
  }
  Serial.println();
}

// ---------------- 串口 ----------------
void printHelp() {
  Serial.println();
  Serial.println("k <KEY> [act] [agent]  发按键，例如: k ACT10 1 / k AG00 1 0");
  Serial.println("n                      模拟按住说话：走真按键同一条路（ACT10 按下 150ms 后抬起）");
  Serial.println("y                      模拟 PWR 短按：一颗 70ms 脉冲（发 `vc` 设定的键名）");
  Serial.println("vc [ACTxx]             看 / 改 PWR 键发的键名（默认 ACT11；配合桌面端键位）");
  Serial.println("j <angle> <distance>   发摇杆，例如: j 0.75 1.0");
  Serial.println("b <0..100>             临时覆盖电量显示（b -1 = 恢复 PMU 真值）");
  Serial.println("p                      AXP2101 现场：电量/充电/VBUS + 中断（带位名）+ 扩展IO");
  Serial.println("i                      I2C 总线扫描（板上有哪些设备应答）");
  Serial.println("x                      PWR 键嗅探开关（常驻，变化即打印+留痕）：PMU/扩展IO/GPIO");
  Serial.println("xl                     倒出事件历史（开机以来所有跳变，含按键；随时可敲）");
  Serial.println("e                      ES8311 关键寄存器现场");
  Serial.println("g                      播放完成提示音（A4->C#5, 270ms）");
  Serial.println("m                      播放测试音（880Hz 300ms）");
  Serial.println("w0 / w1                关 / 开功放 PA（GPIO46）");
  Serial.println("s                      打印状态");
  Serial.println("t                      屏自检（红/绿/蓝/彩条，驻留 12s）");
  Serial.println("d                      转储触摸原始寄存器");
  Serial.println("r                      清零触摸量程统计");
  Serial.println("u                      打印当前 UI 状态（版式/六键/颜色）");
  Serial.println("a                      查询触摸自动休眠状态（寄存器 0xFE）");
  Serial.println("a0 / a1                关闭 / 恢复自动休眠（关 = 一直 Dynamic）");
  Serial.println("q                      中断自检：A)强制拉低 GPIO21 看 ISR  B)0xFA=0x80 芯片自发中断");
  Serial.println("v0 / v1                关 / 开触摸调试：屏上画触点十字 + 串口打 [tmove]");
  Serial.println("c                      六点触摸校准：依次点 A1..A6 的圆心，自动拟合 raw->screen");
  Serial.println("z0 / z1 / z2           关 / 应用 / 恢复出厂校准（v1 时十字：黄=关 青=开）");
  Serial.println("h                      帮助");
}

void printStatus() {
  const CodexMicroState st = ble.snapshot();
  BLEServer* server = BLEDevice::getServer();
  const int peers = server == nullptr ? 0 : server->getConnectedCount();
  const int advertising =
      BLEDevice::getAdvertising()->isAdvertising() ? 1 : 0;
  Serial.printf(
      "[status] connected=%d peers=%d adv=%d rpc=%d quota=%s rem=%.1f "
      "heap=%u canvas=%d tp=%d irq=%lu smp=%lu pol=%lu as=%d raw=(%d,%d) down=%d "
      "ui=%d,%d swipe=%d commits=%lu bat=%d chg=%d dock=%d pmu=%d "
      "ptt=%d boot=%d vc=%d vck=%s\n",
      st.connected ? 1 : 0, peers, advertising, st.hostRpcObserved ? 1 : 0,
      st.quota.available ? "yes" : "no", st.quota.remainingPercent,
      static_cast<unsigned>(ESP.getFreeHeap()),
      display.doubleBuffered() ? 1 : 0, touch.ready() ? 1 : 0,
      static_cast<unsigned long>(touch.irqCount()),
      static_cast<unsigned long>(touch.samples()),
      static_cast<unsigned long>(touch.pollCount()),
      touch.autoSleepDisabled() ? 1 : 0, touch.x(), touch.y(),
      touch.pressed() ? 1 : 0, touchStartX, touchStartY,
      static_cast<int>(activeSwipe),
      static_cast<unsigned long>(touchCommits),
      static_cast<int>(batteryPercent), charging ? 1 : 0, docked ? 1 : 0,
      power.ready() ? 1 : 0, pttDown ? 1 : 0, bootRawPressed ? 1 : 0,
      voiceClickReleasePending ? 1 : 0, gVoiceChatKey);
}

// AXP2101 / ES8311 的完整现场。两个 `p` / `e` 命令共用同一个风格：
// 先打原始寄存器（事实），再打解码结果（解释）—— 混在一起就分不清
// "芯片报了什么"和"我怎么理解它"了。
void dumpPower() {
  const bool ready = power.ready();
  const int32_t idNow = power.readReg(BoardPower::kRegIcType);
  Serial.printf(
      "[pmu] ready=%d(开机探针) 现在读 0x03=0x%02X  0x34 %s\n", ready ? 1 : 0,
      idNow < 0 ? 0 : static_cast<int>(idNow),
      idNow < 0 ? "无应答" : "有应答");
  power.readNow();
  const BoardPowerReading& r = power.reading();
  Serial.printf("[pmu] 0x00=0x%02X 0x01=0x%02X 0x30=0x%02X ok=%d\n", r.status1,
                r.status2, r.adcCtrl, r.ok ? 1 : 0);
  Serial.printf(
      "[pmu] 电池在位=%d 充电中=%d 充电状态机=%d VBUS好=%d VBUS_IN=%d\n",
      r.batteryPresent ? 1 : 0, r.charging ? 1 : 0, static_cast<int>(r.chgState),
      r.vbusGood ? 1 : 0, r.vbusIn ? 1 : 0);
  Serial.printf(
      "[pmu] vbat=%umV vbus=%umV SOC(0xA4)=%d -> 屏上电量=%d%%（-1 = 没接电池/读不到）\n",
      r.vbatMv, r.vbusMv, static_cast<int>(r.socRaw),
      static_cast<int>(power.percentForUi()));
  Serial.printf("[pmu] PWR键 0x49=0x%02X%s%s%s%s  （RW1C：读不清、写 1 才清）\n",
                r.pekRaw, (r.pekRaw & BoardPower::kPekNeg) ? " PRESS" : "",
                (r.pekRaw & BoardPower::kPekPos) ? " RELEASE" : "",
                (r.pekRaw & BoardPower::kPekShort) ? " SHORT" : "",
                (r.pekRaw & BoardPower::kPekLong) ? " LONG" : "");
  // 原样（不掩码）打出中断相关的全部寄存器。位定义一旦搞错，掩码会把真相
  // 一起盖掉 —— 该看的那几位被掩掉时，日志看起来就像"什么都没发生"。
  uint8_t irq[3] = {};
  if (power.readRegs(BoardPower::kRegIntsts1, irq, sizeof(irq))) {
    char names[64] = {};
    irqNames(irq, names, sizeof(names));
    Serial.printf(
        "[pmu] INTSTS 0x48=0x%02X 0x49=0x%02X 0x4A=0x%02X  ← 置位的位：%s\n",
        irq[0], irq[1], irq[2], names[0] == '\0' ? "(全 0)" : names);
    Serial.println(
        "[pmu]   （0x49: bit0=PWR_POS 抬手 bit1=PWR_NEG 按下 bit3=PWR_SHORT；"
        "0x4A bit4=CHG_DONE 充电完成 —— 别把它当按键）");
  } else {
    Serial.println("[pmu] INTSTS 读取失败（0x34 无应答）");
  }
  const uint8_t cfgRegs[] = {BoardPower::kRegInten1, BoardPower::kRegInten2,
                             BoardPower::kRegInten3, BoardPower::kRegPwronCfg};
  const char* cfgNames[] = {"INTEN1", "INTEN2", "INTEN3", "按键时间/电平"};
  for (size_t i = 0; i < sizeof(cfgRegs) / sizeof(cfgRegs[0]); ++i) {
    const int32_t v = power.readReg(cfgRegs[i]);
    Serial.printf("[pmu] %-12s 0x%02X = %s0x%02X\n", cfgNames[i], cfgRegs[i],
                  v < 0 ? "ERR " : "", v < 0 ? 0 : static_cast<int>(v));
  }
  // 另一条候选通道：板载 TCA9554 扩展 IO。两条都打出来，谁动就是谁。
  if (keys.ready()) {
    uint8_t p0 = 0;
    uint8_t p1 = 0;
    keys.readPorts(&p0, &p1);
    Serial.printf(
        "[keys] 扩展IO 0x%02X config=0x%02X port0=0x%02X port1=0x%02X "
        "EXIO%d=%d（静止=%d 按下=%d）\n",
        keys.addr(), keys.config(), p0, p1, BoardKeys::kPwrBit,
        (p0 >> BoardKeys::kPwrBit) & 1, (keys.idle0() >> BoardKeys::kPwrBit) & 1,
        !((keys.idle0() >> BoardKeys::kPwrBit) & 1));
  } else {
    Serial.println("[keys] 扩展IO 不可用（0x20..0x27 无应答）");
  }
  Serial.printf("[audio] codec(0x18)=%d i2s=%d PA=%d(GPIO%d) @%luHz\n",
                audio.codecReady() ? 1 : 0, audio.i2sReady() ? 1 : 0,
                audio.ampOn() ? 1 : 0, BoardAudio::kAmpPin,
                static_cast<unsigned long>(BoardAudio::kSampleRate));
}

void dumpCodec() {
  // ES8311 的关键寄存器。0x00 上电后 bit7 应为 1（power on）、
  // 0x32 是 DAC 音量、0x06/0x07/0x08 是 BCLK/LRCK 分频。
  const uint8_t regs[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
                          0x06, 0x07, 0x08, 0x09, 0x0A, 0x0D,
                          0x0E, 0x12, 0x13, 0x1C, 0x32, 0x37};
  Serial.printf("[audio] ES8311 @0x%02X 寄存器现场\n", BoardAudio::kCodecAddr);
  for (size_t i = 0; i < sizeof(regs); ++i) {
    const int32_t v = audio.readCodecReg(regs[i]);
    Serial.printf("[audio]   0x%02X = %s%02X\n", regs[i], v < 0 ? "ERR " : "",
                  v < 0 ? 0 : static_cast<int>(v));
  }
}

void dumpTouchRegs() {
  uint8_t regs[6] = {};
  const bool ok = touch.readRegs(0x01, regs, sizeof(regs));
  if (!ok) {
    Serial.println("[tdump] raw read FAILED (I2C 0x15 无 ACK)");
    return;
  }
  Serial.printf("[tdump] 01=%02X 02=%02X 03=%02X 04=%02X 05=%02X 06=%02X | "
                "gpio21=%u irq=%lu smp=%lu pol=%lu\n",
                regs[0], regs[1], regs[2], regs[3], regs[4], regs[5],
                static_cast<unsigned>(touch.intLevel()),
                static_cast<unsigned long>(touch.irqCount()),
                static_cast<unsigned long>(touch.samples()),
                static_cast<unsigned long>(touch.pollCount()));
  // 0xFE 单独读：这是「自动休眠到底关掉没有」的唯一现场证据。
  uint8_t autoSleep = 0;
  const bool asOk = touch.readRegs(BoardTouch::kAutoSleepReg, &autoSleep, 1);
  Serial.printf("[tdump] 0xFE=%d read=%s disabled=%d\n",
                asOk ? static_cast<int>(autoSleep) : -1, asOk ? "ok" : "FAIL",
                touch.autoSleepDisabled() ? 1 : 0);
  char line[96];
  // 注意顺序：Value_Information 枚举是 FINGER_NUMBER(0) / COORDINATE_X(1) /
  // COORDINATE_Y(2)，所以 driverValue 的 0/1/2 是 finger/x/y，不是 x/y/f。
  snprintf(line, sizeof(line), "[tdump] driver finger=%.0f x=%.0f y=%.0f",
           touch.driverValue(0), touch.driverValue(1), touch.driverValue(2));
  Serial.println(line);
}

// 把"屏幕此刻应该长什么样"打成文本，用来跟老胡肉眼看到的东西逐项对。
// 版式是算出来的，只有文字描述和实物对上，才能确认没有算错。
void dumpUi() {
  const dashboard::State ui = uiState();
  char reset[24];
  dashboard::formatReset(ui.resetInSeconds, reset, sizeof(reset));
  const char* resetLabel = ui.quotaAvailable
                               ? reset
                               : (ui.quotaStale ? "CHECK COMPANION"
                                                : "NO QUOTA DATA");
  Serial.printf(
      "[ui] %dx%d center=%d,%d orbit=%d,%d keyR=%d dial=%d/%d sendR=%d\n",
      dashboard::kWidth, dashboard::kHeight, dashboard::kCenterX,
      dashboard::kQuotaCenterY, dashboard::kOrbitX, dashboard::kOrbitY,
      dashboard::kAgentRadius, dashboard::kDialOuterRadius,
      dashboard::kDialInnerRadius, dashboard::kSendRadius);
  Serial.printf(
      "[ui] link=%s dial=%s quota=%s reset=%s batt=%d%% charging=%d\n",
      dashboard::linkHealthLabel(ui.linkHealth),
      ui.quotaStale ? "SYNC STALE"
                    : (ui.linkHealth == dashboard::LinkHealth::Offline
                           ? "WAITING CODEX"
                           : "LEFT"),
      ui.quotaAvailable ? "yes" : "no", resetLabel,
      static_cast<int>(ui.batteryPercent), ui.charging ? 1 : 0);
  for (int i = 0; i < 6; ++i) {
    const dashboard::AgentStatus status = dashboard::classify(ui.threads[i]);
    const std::uint32_t rgb = dashboard::statusFillRgb(status, ui.threads[i]);
    const std::uint16_t fill =
        dashboard::rgb565(rgb, ui.threads[i].brightness);
    Serial.printf(
        "[ui] A%d (%3d,%3d) status=%-11s src=#%06lX fill=#%04X b=%.2f\n", i + 1,
        dashboard::kAgentCenters[i].x, dashboard::kAgentCenters[i].y,
        dashboard::agentStatusName(status),
        static_cast<unsigned long>(ui.threads[i].color),
        static_cast<unsigned>(fill),
        static_cast<double>(ui.threads[i].brightness));
  }
}

void handleCommand(char* line) {
  char* head = strtok(line, " ");
  if (head == nullptr) return;

  if (strcmp(head, "h") == 0 || strcmp(head, "help") == 0) {
    printHelp();
    return;
  }
  if (strcmp(head, "s") == 0) {
    printStatus();
    return;
  }
  if (strcmp(head, "t") == 0) {
    beginSelfTest();
    return;
  }
  if (strcmp(head, "d") == 0) {
    dumpTouchRegs();
    return;
  }
  if (strcmp(head, "q") == 0) {
    // 两级 INT 自检，从**最不依赖触摸芯片**的那一级开始：
    //
    //  A) 电学自检 —— 直接把 GPIO21 拉低 5ms（等价于芯片自己拉低 INT），再看 ISR
    //     涨不涨。这一步完全不经过触摸芯片，测的就是"GPIO21 + attachInterrupt +
    //     ISR"这条链子本身。涨 ⇒ 我们这端是好的，此后任何 irq=0 都只能解释成
    //     "芯片没有拉低 INT"；不涨 ⇒ 问题在我们这端，之前所有 irq=0 都不作数。
    //     （先例：上一版只用 B，结果 irq 没动，但分不清是芯片不产中断还是 ISR 死。）
    //  B) 芯片自检 —— 0xFA=0x80（EnTest）让芯片按扫描周期自发产生中断 1.2s。
    //     CST820 这一位未必支持（id=0xB7 与文档里的 CST816S 有差异），
    //     所以 B 只当旁证，判据以 A 为准。
    const uint32_t irq0 = touch.irqCount();
    pinMode(BoardTouch::kIntPin, OUTPUT);
    digitalWrite(BoardTouch::kIntPin, LOW);
    delay(5);
    const uint32_t irqA = touch.irqCount();
    pinMode(BoardTouch::kIntPin, INPUT_PULLUP);
    delay(2);
    Serial.printf(
        "[tp] ISR self-test force-LOW gpio21: irq=%lu->%lu (+%lu) intLevel=%u\n",
        static_cast<unsigned long>(irq0), static_cast<unsigned long>(irqA),
        static_cast<unsigned long>(irqA - irq0),
        static_cast<unsigned>(touch.intLevel()));

    const uint32_t b0 = touch.irqCount();
    const bool wok = touch.writeReg(0xFA, 0x80);
    const uint32_t t0 = millis();
    while (millis() - t0 < 1200) delay(5);  // ISR 在后台累加，不被 poll 抢
    uint8_t inTest = 0;
    const bool testOk = touch.readRegs(0xFA, &inTest, 1);
    const uint32_t b1 = touch.irqCount();
    touch.writeReg(0xFA, 0x40);  // 恢复 PERIODIC（检测到触摸就周期报）
    uint8_t back = 0;
    const bool backOk = touch.readRegs(0xFA, &back, 1);
    Serial.printf(
        "[tp] chip self-test write=%d 0xFA@test=%s%02X irq=%lu->%lu (+%lu/1.2s) "
        "0xFA@restore=%s%02X intLevel=%u\n",
        wok ? 1 : 0, testOk ? "" : "ERR", testOk ? static_cast<int>(inTest) : 0,
        static_cast<unsigned long>(b0), static_cast<unsigned long>(b1),
        static_cast<unsigned long>(b1 - b0), backOk ? "" : "ERR",
        backOk ? static_cast<int>(back) : 0,
        static_cast<unsigned>(touch.intLevel()));
    return;
  }
  if (strcmp(head, "r") == 0) {
    touch.resetRange();
    touchCommits = 0;
    Serial.println("[tp] range reset / commit counter cleared");
    return;
  }
  if (strcmp(head, "u") == 0) {
    dumpUi();
    return;
  }
  if (strcmp(head, "a") == 0) {
    Serial.printf("[tp] autoSleep 0xFE=%d disabled=%d\n",
                  static_cast<int>(touch.autoSleepReg()),
                  touch.autoSleepDisabled() ? 1 : 0);
    return;
  }
  if (strcmp(head, "a0") == 0) {
    const bool ok = touch.setAutoSleep(false);
    Serial.printf("[tp] autoSleep OFF ok=%d 0xFE=%d disabled=%d\n", ok ? 1 : 0,
                  static_cast<int>(touch.autoSleepReg()),
                  touch.autoSleepDisabled() ? 1 : 0);
    return;
  }
  if (strcmp(head, "a1") == 0) {
    const bool ok = touch.setAutoSleep(true);
    Serial.printf("[tp] autoSleep ON  ok=%d 0xFE=%d disabled=%d\n", ok ? 1 : 0,
                  static_cast<int>(touch.autoSleepReg()),
                  touch.autoSleepDisabled() ? 1 : 0);
    return;
  }
  if (strcmp(head, "v0") == 0 || strcmp(head, "v1") == 0) {
    touchDebug = (head[1] == '1');
    redrawPending = true;
    Serial.printf("[tp] touchDebug=%d（屏上画触点十字；串口打 [tmove] 每个移动样本）\n",
                  touchDebug ? 1 : 0);
    return;
  }
  if (strcmp(head, "c") == 0) {
    calIndex = 0;
    calRep = 0;
    // 注意：**不要**在这里清 calApplied / calFitOk。
    // 校准取的本来就是 raw（见 beginGesture 头注释），跟"当前有没有应用修正"
    // 无关；保留旧修正的好处是：万一这次校准中途放弃，板子仍停在上一套可用
    // 的修正上，而不是莫名掉回未修正状态。拟合成功后才覆盖常量。
    for (int i = 0; i < kCalPoints; ++i) calPts[i].n = 0;
    touchDebug = true;  // 校准必须能看见十字，否则没法判断自己点没点正
    Serial.printf(
        "[cal] 六点校准（每键 %d 次，共 %d 次）：屏幕上已高亮 A1\n",
        kCalRepeats, kCalPoints * kCalRepeats);
    Serial.println(
        "[cal] 顺序 A1->A2->A3->A4->A5->A6，每个键**连点 3 次它的正中心**，"
        "点满 3 次自动跳到下一个键");
    Serial.println(
        "[cal] 单纯取点：不会发任何按键。六个键都收满后自动拟合，"
        "打印数字仪增益与残差");
    redrawPending = true;
    return;
  }
  if (strcmp(head, "z0") == 0 || strcmp(head, "z1") == 0 ||
      strcmp(head, "z2") == 0) {
    if (head[1] == '2') {  // z2 = 丢弃 NVS 里的校准，回到出厂默认值
      resetCalToDefault();
      redrawPending = true;
      Serial.printf(
          "[cal] z2 已弃用 NVS 校准，回到出厂默认："
          "kx=%.4f cx=%+.1f ky=%.4f cy=%+.1f  apply=1\n",
          calKx, calCx, calKy, calCy);
      return;
    }
    const bool want = (head[1] == '1');
    if (want && !calFitOk) {
      Serial.println("[cal] 还没有可用的拟合结果，先跑一次 c（或等 6 个点点完）");
      return;
    }
    calApplied = want;
    redrawPending = true;
    Serial.printf("[cal] apply=%d  十字颜色=%s\n", calApplied ? 1 : 0,
                  calApplied ? "青(修正后)" : "黄(原始)");
    // 顺手报一下当前用的是哪一套常量、从哪里来 —— 排查"越校越歪"时最关键。
    Serial.printf("[cal] 常量 kx=%.4f cx=%+.1f ky=%.4f cy=%+.1f 来源=%s\n", calKx,
                  calCx, calKy, calCy,
                  calFromNvs ? "NVS(曾被 c 校准过)" : "出厂默认(本板实测)");
    return;
  }
  if (strcmp(head, "p") == 0) {
    dumpPower();
    return;
  }
  if (strcmp(head, "i") == 0) {
    i2cScanBus();
    return;
  }
  if (strcmp(head, "xl") == 0) {
    dumpEventLog();
    return;
  }
  if (strcmp(head, "x") == 0) {
    // 常驻监听的开关：无参数 = 翻转；`x 0` / `x 1` = 明确关/开。
    const char* arg = strtok(nullptr, " ");
    setPwrWatch(arg == nullptr ? !pwrWatch : atoi(arg) != 0);
    return;
  }
  if (strcmp(head, "e") == 0) {
    dumpCodec();
    return;
  }
  if (strcmp(head, "g") == 0) {
    // 模拟"某个 Agent 刚跑完"：横幅 + 提示音一起走，这样同一条命令能同时
    // 验证"检测到完成之后的动作"这一整段。真正触发它的是 thstatus 下发的
    // RGB 变绿（见 detectAgentTransitions()）；单独测音频用 m。
    // 这里不用 startCompletionChime()：那个函数定义在文件后面，
    // 直接内联两行比依赖 .ino 的自动原型生成更稳。
    completionBanner.show(0, millis(), kCompletionBannerMs);
    redrawPending = true;
    audio.playChime();
    Serial.printf("[audio] 模拟 A1 完成 -> 横幅 A1 DONE %lums + 提示音\n",
                  static_cast<unsigned long>(kCompletionBannerMs));
    return;
  }
  if (strcmp(head, "m") == 0) {
    // 单频测试音：用来把"喇叭/功放不行"和"提示音生成不对"分开。
    audio.playTone(880.0f, 300, 0.20f);
    return;
  }
  if (strcmp(head, "w0") == 0 || strcmp(head, "w1") == 0) {
    const bool on = (head[1] == '1');
    audio.setAmp(on);
    Serial.printf("[audio] 功放 PA(GPIO%d) = %d\n", BoardAudio::kAmpPin,
                  on ? 1 : 0);
    return;
  }
  if (strcmp(head, "k") == 0) {
    char* key = strtok(nullptr, " ");
    if (key == nullptr) {
      Serial.println("[cmd] usage: k <KEY> [act] [agent]");
      return;
    }
    char* actArg = strtok(nullptr, " ");
    char* agentArg = strtok(nullptr, " ");
    const uint8_t action = actArg == nullptr ? 1 : atoi(actArg);
    const int8_t agent =
        agentArg == nullptr ? -1 : static_cast<int8_t>(atoi(agentArg));
    ble.sendKey(key, action, agent);
    Serial.printf("[cmd] sendKey %s action=%u agent=%d\n", key, action, agent);
    return;
  }
  // 模拟两颗物理按键（不用真按）：走的是和真按键**同一条代码路径**
  // （pttPress / beginVoiceChatClick），所以验的是真实行为而不是旁路。
  if (strcmp(head, "n") == 0) {  // 模拟按住说话（BOOT/PTT）
    pttPress();
    pttAutoReleaseAtMs = millis() + kSimPttHoldMs;
    return;
  }
  if (strcmp(head, "y") == 0) {  // 模拟 PWR 短按（语音聊天）
    beginVoiceChatClick();
    return;
  }
  // 改 PWR 键发哪个键名。桌面端改了键位（例如把"独立麦克风按键"里的一个开关
  // 设成"语音聊天"）之后，先 `vc` 看当前值，再 `vc ACTxx` 就地换绑 —— 不重编译。
  if (strcmp(head, "vc") == 0) {
    char* keyArg = strtok(nullptr, " ");
    if (keyArg == nullptr) {
      Serial.printf("[cmd] PWR 键当前发 %s（改法：vc ACT11）\n", gVoiceChatKey);
    } else {
      strncpy(gVoiceChatKey, keyArg, sizeof(gVoiceChatKey) - 1);
      gVoiceChatKey[sizeof(gVoiceChatKey) - 1] = '\0';
      Serial.printf("[cmd] PWR 键改发 %s（按一下 PWR 或敲 y 验证）\n", gVoiceChatKey);
    }
    return;
  }
  if (strcmp(head, "j") == 0) {
    char* angleArg = strtok(nullptr, " ");
    char* distArg = strtok(nullptr, " ");
    const float angle = angleArg == nullptr ? 0.0f : atof(angleArg);
    const float distance = distArg == nullptr ? 1.0f : atof(distArg);
    ble.sendJoystick(angle, distance);
    Serial.printf("[cmd] sendJoystick angle=%.2f distance=%.2f\n", angle,
                  distance);
    return;
  }
  if (strcmp(head, "b") == 0) {
    char* value = strtok(nullptr, " ");
    if (value == nullptr) {
      Serial.println("[cmd] usage: b <0..100>");
      return;
    }
    const int percent = atoi(value);
    if (percent < 0) {
      batteryOverride = -1;
      lastReportedBattery = -2;  // 强制下一轮重新上报
      Serial.println("[cmd] 电量改回 AXP2101 的真值");
    } else {
      batteryOverride = static_cast<int8_t>(percent > 100 ? 100 : percent);
      batteryPercent = batteryOverride;
      lastReportedBattery = -2;
      Serial.printf("[cmd] 电量临时覆盖 = %d%%（b -1 恢复真值）\n",
                    batteryOverride);
    }
    redrawPending = true;
    return;
  }

  Serial.printf("[cmd] unknown: %s (h = help)\n", head);
}

void handleSerial() {
  static char buffer[96];
  static size_t length = 0;
  while (Serial.available()) {
    const int c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n' || length >= sizeof(buffer) - 1) {
      buffer[length] = '\0';
      if (length > 0) handleCommand(buffer);
      length = 0;
      continue;
    }
    buffer[length++] = static_cast<char>(c);
  }
}

// ---------------- ⑤ 电源 / 音频 / 完成提示 ----------------

void refreshPowerTelemetry() {
  power.poll();  // 内部按 2s 节流

  const int8_t next = batteryOverride >= 0 ? batteryOverride
                                           : power.percentForUi();
  const bool nextCharging = power.chargingForUi();
  const bool nextDocked = power.vbusPresent();
  if (next != batteryPercent || nextCharging != charging ||
      nextDocked != docked) {
    batteryPercent = next;
    charging = nextCharging;
    docked = nextDocked;
    redrawPending = true;
  }

  // device.status 的 battery 字段只有 0..100 才有意义（-1 = 没接电池/读不到），
  // 所以这种情况**不上报**，让主机看到的是"没数据"而不是"0%"。
  if (batteryPercent >= 0 && (batteryPercent != lastReportedBattery ||
                              charging != lastReportedCharging)) {
    lastReportedBattery = batteryPercent;
    lastReportedCharging = charging;
    ble.setBattery(static_cast<uint8_t>(batteryPercent), charging);
  }
}

void startCompletionChime(int8_t agent) {
  completionBanner.show(agent, millis(), kCompletionBannerMs);
  // playChime() 是阻塞写（约 270ms 的 PCM）。step2 实测过 500ms 的全屏
  // QSPI flush 都不影响 BLE 收发，这里短得多，可以放心阻塞。
  audio.playChime();
  Serial.printf("[audio] agent A%d 完成 -> 横幅 %lums + 提示音\n", agent + 1,
                static_cast<unsigned long>(kCompletionBannerMs));
}

// 移植上游 detectAgentTransitions()。判据是"上一次不是 Complete、
// 这一次是 Complete"，见文件头注释里为什么不能只看当前状态。
void detectAgentTransitions() {
  // 50ms 一次就够（主机轮询是分钟级的），不必每个 loop 都去拿一次快照。
  static uint32_t lastMs = 0;
  const uint32_t nowMs = millis();
  if (lastMs != 0 && (nowMs - lastMs) < 50) return;
  lastMs = nowMs;

  const CodexMicroState st = ble.snapshot();
  for (int i = 0; i < 6; ++i) {
    dashboard::ThreadVisual light;
    light.color = st.threads[i].color;
    light.brightness = st.threads[i].brightness;
    light.focused = false;
    const dashboard::AgentStatus next = dashboard::classify(light);
    if (agentStatusSeen[i] &&
        lastAgentStatus[i] != dashboard::AgentStatus::Complete &&
        next == dashboard::AgentStatus::Complete) {
      startCompletionChime(static_cast<int8_t>(i));
    }
    lastAgentStatus[i] = next;
    agentStatusSeen[i] = true;
  }
}

// ================= 物理按键：BOOT=按住说话 / PWR=语音聊天 =================
// 用到的状态位（pttDown / bootRawPressed / voiceClickRelease…）**定义在文件前部**，
// 因为命令 `n`/`y` 要用到它们，而变量必须在 handleSerial 之前就可见。
void pttPress() {
  if (pttDown) return;  // 真按键撞上模拟时不重复发
  pttDown = true;
  ble.sendKey(kPttKey, 1);
  evtPush("[btn] PTT 按下 -> ACT10 1");
}

void pttRelease() {
  if (!pttDown) return;
  pttDown = false;
  pttAutoReleaseAtMs = 0;
  ble.sendKey(kPttKey, 0);
  evtPush("[btn] PTT 抬起 -> ACT10 0");
}

// 上游 beginVoiceChatClick()：先补上一次没收尾的抬手，再发按下，70ms 后收尾。
// 这样连按两次不会让主机看到"两次按下中间没有抬起"。
void beginVoiceChatClick() {
  if (voiceClickReleasePending) ble.sendKey(gVoiceChatKey, 0);
  ble.sendKey(gVoiceChatKey, 1);
  voiceClickReleasePending = true;
  voiceClickReleaseAtMs = millis() + kVoiceClickPulseMs;
  char line[72];
  snprintf(line, sizeof(line), "[btn] 语音聊天 -> %s 一颗 70ms 脉冲", gVoiceChatKey);
  evtPush(line);
}

void updateVoiceChatClick() {
  if (!voiceClickReleasePending) return;
  if (static_cast<int32_t>(millis() - voiceClickReleaseAtMs) < 0) return;
  ble.sendKey(gVoiceChatKey, 0);
  voiceClickReleasePending = false;
}

// BOOT 键：GPIO0 低有效，40ms 消抖，**电平触发**（照上游写法：
// "电平跟上次不同、且这个不同已经稳定超过 40ms"才算数）。
// 与 esptool 不冲突：下载工具是在复位瞬间把 GPIO0 拉低，跟运行中的轮询无关。
void pollBootButton() {
  const uint32_t now = millis();
  const bool raw = digitalRead(kBootButtonPin) == LOW;
  if (raw != bootRawPressed) {
    if (now - bootChangedAtMs >= kBootButtonDebounceMs) {
      bootRawPressed = raw;
      bootChangedAtMs = now;
      if (bootRawPressed) {
        pttPress();
      } else {
        pttRelease();
      }
    }
  } else {
    bootChangedAtMs = now;  // 电平没变就不断刷新基准，消抖窗口始终从"最后一次变化"起算
  }
}

// `t` 模拟用：按下后到点自动抬起。
void pollSimPtt() {
  if (pttAutoReleaseAtMs == 0) return;
  if (static_cast<int32_t>(millis() - pttAutoReleaseAtMs) < 0) return;
  pttAutoReleaseAtMs = 0;
  pttRelease();
}

// PWR 键：50ms 轮询。按键是人手操作，用不着更快；但比电量那个 2s 的周期
// 快得多，因为边沿事件抢的是"有没有看到"，慢了可能被下一次清标记覆盖掉。
//
// 动作只绑在 SHORT 上：一次短按会来**两条**事件 —— 按下瞬间 `0x02`(只有负沿)，
// 抬手瞬间 `0x09`(正沿+SHORT)。SHORT 只出现一次 ⇒ 一次按键一次动作，不会重复。
// 长按**不接管**：AXP2101 硬件约 6 秒自己关机，软件插手只会抢时序。
void pollPowerButton() {
  static uint32_t lastMs = 0;
  const uint32_t now = millis();
  if (lastMs != 0 && (now - lastMs) < 50) return;
  lastMs = now;

  const uint8_t p = power.pollPek();
  if (p == 0) return;
  char names[64] = {};
  {
    uint8_t one[3] = {0, p, 0};
    irqNames(one, names, sizeof(names));
  }
  char line[160];
  snprintf(line, sizeof(line), "[pmu] PWR 键事件 0x%02X%s%s%s%s%s%s", p,
           (p & BoardPower::kPekNeg) ? " PRESS(负沿)" : "",
           (p & BoardPower::kPekPos) ? " RELEASE(正沿)" : "",
           (p & BoardPower::kPekShort) ? " SHORT" : "",
           (p & BoardPower::kPekLong) ? " LONG" : "",
           names[0] == '\0' ? "" : "  ", names);
  evtPush(line);  // 也进事件环：没人在抓串口时这一下不能白按

  if (p & BoardPower::kPekShort) beginVoiceChatClick();
  if (p & BoardPower::kPekLong) {
    evtPush("[btn] PWR 长按 -> 交给 AXP2101 硬件（软件不接管）");
  }
}

// ---------------- 生命周期 ----------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("Codex Micro / Waveshare port / step5 power+audio");
  Serial.printf("chip=%s rev=%d cores=%d flash=%dKB psram=%dKB\n",
                ESP.getChipModel(), ESP.getChipRevision(),
                ESP.getChipCores(), ESP.getFlashChipSize() / 1024,
                ESP.getPsramSize() / 1024);

  // 成功也要出声。这版早期只在失败时打印，结果"屏/触摸起来了没有"只能从
  // status 行的 canvas=/tp= 反推 —— 而 `tp=1` 并不能区分"CST820 认出来了"
  // 和"恰好 begin() 没报错"。ID 那一位才是芯片身份的硬证据。
  const uint32_t screenStartMs = millis();
  if (!display.begin()) {
    Serial.println("[screen] CO5300 begin FAILED");
  } else {
    Serial.printf("[screen] CO5300 就绪 %dx%d canvas=%d 耗时=%lums\n",
                  BoardDisplay::kWidth, BoardDisplay::kHeight,
                  display.doubleBuffered() ? 1 : 0,
                  static_cast<unsigned long>(millis() - screenStartMs));
  }

  const uint32_t touchStartMs = millis();
  if (!touch.begin()) {
    Serial.println("[touch] CST820 begin FAILED");
  } else {
    Serial.printf("[touch] CST820 ready id=0x%02X int=GPIO%d sda=%d scl=%d "
                  "addr=0x%02X 耗时=%lums\n",
                  touch.deviceId() < 0 ? 0 : touch.deviceId(),
                  BoardTouch::kIntPin, BoardTouch::kSda, BoardTouch::kScl,
                  BoardTouch::kI2cAddr,
                  static_cast<unsigned long>(millis() - touchStartMs));
    // 0xFE 是「关掉自动休眠」有没有真的写进去的硬证据。
    // disabled=0 说明芯片还停在出厂的低功耗 Standby 逻辑上。
    Serial.printf("[touch] autoSleep reg0xFE=%d disabled=%d\n",
                  static_cast<int>(touch.autoSleepReg()),
                  touch.autoSleepDisabled() ? 1 : 0);
  }

  // 触摸坐标修正（raw -> screen）开机即生效：
  //   · NVS 里有上次 c 校准的结果 -> 用它（换板后校准一次就永久有效）
  //   · 没有 -> 用出厂默认（本板实测的增益/截距）
  // 两种情况下 calApplied 都是 true，所以 v1 的十字一开机就是青色。
  loadCalFromNvs();
  Serial.printf("[touch] 坐标修正 apply=1 kx=%.4f cx=%+.1f ky=%.4f cy=%+.1f 来源=%s\n",
                calKx, calCx, calKy, calCy,
                calFromNvs ? "NVS(上次 c 校准)" : "出厂默认(本板实测)");

  // ⑤ 电源与音频。两者都共用触摸那条 Wire，所以必须排在 touch.begin() 之后。
  power.begin();
  keys.begin();  // TCA9554 扩展 IO（PWR 键的第二条候选通道），只读
  audio.begin();
  sniffBeginGpio();  // PWR 键嗅探：三通道常驻监听（loop 里的 pollPwrMonitor）

  // BOOT 键（GPIO0）当 push-to-talk。INPUT_PULLUP 既是这块板按键的标准接法
  // （按键另一端接 GND、按下拉低），也是 esptool 进下载模式需要的那个上拉；
  // 下载工具是在**复位瞬间**拉低 GPIO0，与运行中的轮询互不干扰。
  pinMode(kBootButtonPin, INPUT_PULLUP);
  bootChangedAtMs = millis();
  // 整条总线的现场：0x15 触摸 / 0x18 ES8311 / 0x20~27 扩展IO(若有) /
  // 0x34 AXP2101 / 0x51 RTC / 0x6B IMU。少哪个一眼就能看出来。
  i2cScanBus();

  ble.begin();
  // 电量/充电/VBUS 由 refreshPowerTelemetry() 从 AXP2101 读真值再上报。
  // （④ 里这里是 ble.setBattery(100, false) —— 那是"编一个满电出来"，
  //   插着 USB 的裸板也会显示 100%，看不出到底有没有接电池。）
  refreshPowerTelemetry();
  Serial.printf("[pmu] 首次读数 电量=%d%% 充电=%d VBUS=%d vbat=%umV vbus=%umV\n",
                static_cast<int>(batteryPercent), charging ? 1 : 0,
                docked ? 1 : 0, power.reading().vbatMv, power.reading().vbusMv);
  Serial.printf("[audio] 自检 codec=%d i2s=%d（敲 g 听一声提示音）\n",
                audio.codecReady() ? 1 : 0, audio.i2sReady() ? 1 : 0);

  drawScreen();
  printHelp();
}

void loop() {
  ble.poll();
  pollTouch();
  handleSerial();
  ensureAdvertising();

  const uint32_t now = millis();

  // ⑤：电源遥测（2s 节流）、PWR 键（50ms）、BOOT 键（40ms 消抖）、
  // ACT09 脉冲收尾、完成跃迁（50ms）、横幅到期、功放到点关闭。
  // 都在这里，与屏/BLE 无耦合。
  refreshPowerTelemetry();
  pollPowerButton();
  pollBootButton();
  pollSimPtt();
  updateVoiceChatClick();
  pollPwrMonitor();
  detectAgentTransitions();
  if (completionBanner.expire(now)) redrawPending = true;
  audio.poll();

  pollSelfTest();

  if (selfTestEndAtMs == 0) {
    const bool periodic = (now - lastRedrawAtMs) >= kPeriodicRedrawMs;
    if ((redrawPending || periodic) &&
        (now - lastRedrawAtMs) >= kMinRedrawGapMs) {
      redrawPending = false;
      lastRedrawAtMs = now;
      drawScreen();
    }
  }

  if (now - lastStatusAtMs >= kStatusIntervalMs) {
    lastStatusAtMs = now;
    printStatus();
  }

  delay(5);
}
