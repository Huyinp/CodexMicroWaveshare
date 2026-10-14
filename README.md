# Codex Micro — Waveshare 1.8 移植版

把 [Douiuiuiz/codex-micro-waveshare](https://github.com/Douiuiuiz/codex-micro-waveshare)
（Waveshare ESP32-S3-Touch-AMOLED-**1.75C** 版）复刻到 **Waveshare ESP32-S3-Touch-AMOLED-1.8**
开发板：让 ESP32-S3 在 BLE 上伪装成 "Codex Micro"（ChatGPT Desktop 的 BLE HID 外设），
并附带一个用量（配额）看板。

协议层（`CodexMicroBle.*` 等）与上游**逐字相同**，只重写了板级驱动和看板版式。

> ⚠️ 这是一个**非官方**的兼容性项目，与 OpenAI / Work Louder / M5Stack 无隶属关系。
> 商标、协议兼容性、无担保等声明见 [`NOTICE.md`](NOTICE.md)。

---

## 1. 硬件平台

| 项目 | 本板（1.8） | 上游（1.75C） |
|---|---|---|
| 主控 | ESP32-S3（带 PSRAM 8MB） | ESP32-S3 |
| 屏 | CO5300 AMOLED **368×448**（QSPI） | CO5300 466×466 |
| 触摸 | **CST820**（I²C，INT=GPIO21） | CST9217 |
| 电源管理 | AXP2101 @0x34 | AXP2101 |
| 音频编解码 | ES8311 @0x18（提示音输出） | ES8311 |

> 注意：资料里曾误传为 SH8601 + FT3168，实际以本板为准（CO5300 + CST820）。

---

## 2. 功能与两条独立通道

设备对外暴露两条互不干扰的 BLE 通道：

1. **控制面 — 标准 BLE HID**（`0x1812`，VID `303A` / PID `8360`，vendor usage `0xFF00`，
   report ID 6，63 字节 body）。`json + "\n"` 按 61 字节切片，每片 `report[0]=2, report[1]=len`，
   片间约 4ms。
   - 设备→主机：`v.oai.hid{k,act,ag}`（按键）、`v.oai.rad{a,d}`（模拟摇杆/方向键）
   - 主机→设备 RPC：`sys.version` / `device.status` / `v.oai.thstatus{id,c,b,e,s}` /
     `v.oai.rgbcfg` / `lights.preview` / `host.focused_app`
   - Agent 状态**只能**靠 `thstatus` 的 RGB 阈值反推（绿 > 红×1.12 = Complete）
2. **配额面 — 私有 GATT** `7f0d4e66-…-5c01`（写 `5c02`）。Windows 伴进程读取本机配额后
   写入 `{remaining_percent, reset_in_seconds}`，**写完立即断开**（Windows 基本只维持一条 LE 连接）。

**键名只是"插槽编号"，动作由桌面端决定**：设备发 `ACT09` 只表示"第 4 号命令键被按了"，
具体干什么在 ChatGPT 桌面端 **设置 → Codex Micro** 里绑定。默认映射：

| 控件 | 出厂默认动作 | 键名 |
|---|---|---|
| Command Key 4 | 新开分支（Continue in a new chat） | `ACT09` |
| Mic 开关 1（BOOT 键） | 按住说话 Push to talk | `ACT10` |
| Mic 开关 2（PWR 键） | 语音聊天 Voice chat | `ACT11`* |
| Codex 键（表盘） | Send | `ACT12` |
| 6 个 Agent 键 | 跟随最近 6 个会话 | `AG00`..`AG05` |

\* 出厂 `ACT09` 误触即分叉对话，固件默认已避开，把 PWR 键指向 `ACT11`
（宽麦克风键"使用独立麦克风按键"拆出的第二个开关）；用串口命令 **`vc [ACTxx]`** 可零重编译改键名。
**录音走电脑麦克风**：设备只发"开始/停止"控制信号，音频不经过 BLE。

---

## 3. 目录结构

```
.
├── firmware/
│   ├── step5_power_audio/   ← 最终固件（要烧的就是这个）
│   └── archive/             ← step1~4 复刻过程存档（每步独立可编译，仅作参考）
├── tools/                   ← 串口 / 编译烧录 工具
│   ├── build.bat            ← 一键编译（双击即可，无需 Arduino IDE）
│   ├── flash.bat            ← 编译 + 烧录（默认 COM11，可 flash.bat COM6）
│   ├── tap_peek.py          ← 不复位读串口（看事件环 / 配额等 RAM 状态）
│   ├── tap_reset.py         ← 复位并抓 boot 日志
│   ├── tap_send.py          ← 复位→重连→下发串口命令
│   ├── tap_touch.py         ← 触摸校准/验证采集窗
│   └── probe_ports.py       ← 排查"COM 口被进程独占"
├── companion/               ← ⑥ 配额伴进程（Windows / Python + bleak）
│   ├── codex_quota_bridge.py
│   └── run_quota_bridge.bat
├── scripts/
│   └── patch_nimble_duplicate_char.py   ← NimBLE 补丁（见 §5 已知坑①）
├── docs/
│   ├── 复刻指南.md           ← 完整复刻/排障笔记（含 NimBLE 根因、校准、键位）
│   ├── 编译与烧录说明.md      ← 编译参数、串口命令表、报错对照
│   └── ws18_schematic.pdf    ← 板子原理图
├── LICENSE
├── NOTICE.md
└── README.md
```

---

## 4. 编译与烧录

### 依赖
- **Arduino-ESP32 `3.3.7-cn`**（默认 **NimBLE** 后端，Bluedroid 未启用）
- 库：`GFX_Library_for_Arduino`（自带 `Arduino_CO5300`）、`Arduino_CST816x`
- **不要升级**到 latest 3.3.11；不要换 Bluedroid。

### 方式 A：一键脚本（推荐）
```bat
tools\build.bat        # 仅编译，产物在 build\step5_pa
tools\flash.bat        # 编译 + 烧录（默认 COM11；其他口 flash.bat COM6）
```
> 脚本路径/FQBN/库目录已写死，全 ASCII，双击即用。

### 方式 B：Arduino IDE / arduino-cli
```
FQBN: esp32:esp32:waveshare_esp32_s3_touch_amoled_18:CDCOnBoot=cdc,PSRAM=enabled
目标: firmware/step5_power_audio
```
详细参数与串口命令表见 [`docs/编译与烧录说明.md`](docs/编译与烧录说明.md)。

---

## 5. 使用与已知坑

### ① NimBLE 必须打补丁（第一次编译前）
本地 core 的 `BLEService::addCharacteristic()` 在 NimBLE 下**按 UUID 去重**，
会把 HOGP 必需的第二个 `0x2A4D`（output report）吃掉，导致主机 `hid_write` 恒报
`0x57 (ERROR_INVALID_PARAMETER)`。
```bat
python scripts\patch_nimble_duplicate_char.py      # 幂等；--revert 可还原
```
打完补丁后串口应打印 `hid chr handle input=29 output=33 battery=37`（output≠65535 即成功）。

### ② 改完 GATT / 第一次配对必须"删设备 → 重配对 → 重启 ChatGPT"
Windows 会缓存配对时的 GATT 表，旧缓存不会自动刷新。出现 `WRITE_FAILED 0x57` 时先这样清缓存。

### ③ 配额伴进程（Windows）
```bat
companion\run_quota_bridge.bat     # 或 python companion\codex_quota_bridge.py
```
- `remaining_percent` **必须是 float**（固件要求），脚本已处理。
- 写完**立即断开**，否则会挤掉 ChatGPT 的 HID 连接。
- 若身处带代理的沙箱环境，可能需要 `--proxy <url>` 覆盖（普通机器不用带）。

### ④ 语音聊天要在桌面端配置
PWR 键默认发 `ACT11`；需在 ChatGPT 桌面端 **设置 → Codex Micro → 使用独立麦克风按键**，
并把**第二个开关的下拉**选成"语音聊天"（仅拆开关不算绑定，每个开关都要各自选动作）。

### ⑤ 观察技巧
- 看按键到底"设备没发"还是"桌面端没绑"：不复位打开串口（`tools/tap_peek.py`），
  敲 `xl` 看事件环、`s` 看配额等 RAM 状态 —— 都不用卡时机。

更完整的排障笔记见 [`docs/复刻指南.md`](docs/复刻指南.md)。

---

## 6. 来源与许可

- 上游：`Douiuiuiz/codex-micro-waveshare`（MIT，Copyright imliubo / codex-micro-4-stopwatch contributors）
  → 源自 `digitsisyph/codex-micro-stopwatch` → `imliubo/codex-micro-4-core2`
- 本移植修改：Copyright (c) 2026 yp.hu，MIT（见 [`LICENSE`](LICENSE)）
- 署名、商标与无担保声明见 [`NOTICE.md`](NOTICE.md)
