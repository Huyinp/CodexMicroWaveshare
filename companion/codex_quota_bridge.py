#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Codex Micro 配额伴进程（Windows / BLE）—— 复刻步骤 ⑥

把 ChatGPT 的 Codex 剩余额度推给 Waveshare ESP32-S3-Touch-AMOLED-1.8。

链路（每轮）:
    读 %CODEX_HOME%/auth.json 的 tokens.access_token
      -> GET https://chatgpt.com/backend-api/wham/usage
      -> 取 rate_limit.primary_window.{used_percent, reset_after_seconds}
      -> 换算成 {"remaining_percent": <float>, "reset_in_seconds": <int>}
      -> BLE 连板子（GATT 7f0d4e66-...-5c02 写）
      -> 写完**立即断开**，把唯一的 Windows LE 链路还给 HID 驱动
      -> 等 interval 秒，再来一轮

为什么必须"写完即断"：
    Windows 基本只维持一条 LE 连接。伴进程占着连接时，ChatGPT 的 HID 会话
    会被挤掉 —— 表现就是按住说话/语音聊天失灵。上游同样这么做。

用法:
    py codex_quota_bridge.py --dry-run --once      # 只读配额，不碰 BLE（最小自检）
    py codex_quota_bridge.py --once --verbose      # 连板子写一次就退
    py codex_quota_bridge.py --interval 120        # 常驻，每 120s 一轮
    py codex_quota_bridge.py --address XX:XX:...   # 跳过扫描直连（更快）

依赖: pip install bleak   （本机已装）
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import pathlib
import sys
import time
import urllib.error
import urllib.request

# --- 与固件 CodexMicroBle.cpp / QuotaPayload.h 的契约（改这里 == 改协议）---
QUOTA_SERVICE_UUID = "7f0d4e66-2ac2-4a71-bfbe-4ef61a0e5c01"
QUOTA_WRITE_UUID = "7f0d4e66-2ac2-4a71-bfbe-4ef61a0e5c02"
MAX_PAYLOAD = 512  # 固件 onQuotaWrite() 的长度上限
DEFAULT_DEVICE_NAME = "Codex Micro"  # 固件 kDeviceName[]

USAGE_URL = "https://chatgpt.com/backend-api/wham/usage"
USER_AGENT = "codex-quota-bridge/1.0"

# Windows 控制台默认可能是 GBK，中文会炸。强制 UTF-8 + 容错。
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass


class BridgeError(Exception):
    """可预期的失败（凭据缺失/接口变更/板子没广播），不必打堆栈。"""


def ts() -> str:
    return time.strftime("%H:%M:%S")


def log(msg: str) -> None:
    print(f"[{ts()}] {msg}", flush=True)


def err(msg: str) -> None:
    print(f"[{ts()}] ERROR {msg}", file=sys.stderr, flush=True)


# ---------------------------------------------------------------------------
# 1. 凭据 + 配额（纯 HTTP，不碰 BLE）
# ---------------------------------------------------------------------------

def codex_home() -> pathlib.Path:
    """CODEX_HOME 优先，否则 ~/.codex（与上游一致）。"""
    env = os.environ.get("CODEX_HOME")
    if env:
        return pathlib.Path(env)
    return pathlib.Path(os.path.expanduser("~")) / ".codex"


def load_access_token() -> str:
    path = codex_home() / "auth.json"
    if not path.exists():
        raise BridgeError(f"no auth.json at {path}; run: codex login")
    with open(path, encoding="utf-8") as handle:
        auth = json.load(handle)
    token = (auth.get("tokens") or {}).get("access_token")
    if not token:
        raise BridgeError(f"auth.json has no tokens.access_token; run: codex login")
    return token


def read_quota(token: str, proxy: str | None = None) -> dict:
    """返回固件能吃的快照（字段名就是 GATT 上的字段名）。

    proxy=None 时走环境变量/系统代理。显式给 --proxy 可以覆盖 —— 有些终端
    会注入自己的 HTTP(S)_PROXY（例如沙箱隧道），那些代理未必放行 chatgpt.com。
    """
    request = urllib.request.Request(
        USAGE_URL,
        headers={
            "Authorization": "Bearer " + token,
            "User-Agent": USER_AGENT,
            "Accept": "application/json",
        },
    )
    if proxy:
        opener = urllib.request.build_opener(
            urllib.request.ProxyHandler({"http": proxy, "https": proxy}))
        urlopen = opener.open
    else:
        urlopen = urllib.request.urlopen
    try:
        with urlopen(request, timeout=20) as response:
            data = json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        if exc.code in (401, 403):
            raise BridgeError(
                f"HTTP {exc.code} from usage endpoint: token expired/invalid. "
                "Run `codex login` or launch Codex CLI once to refresh auth.json."
            ) from exc
        raise BridgeError(f"HTTP {exc.code} from usage endpoint: {exc.reason}") from exc
    except urllib.error.URLError as exc:
        raise BridgeError(f"cannot reach {USAGE_URL}: {exc.reason}") from exc

    window = (data.get("rate_limit") or {}).get("primary_window")
    if not isinstance(window, dict):
        raise BridgeError("no rate_limit.primary_window in wham/usage response")

    used = window.get("used_percent")
    reset_in = window.get("reset_after_seconds")
    if used is None or reset_in is None:
        raise BridgeError("primary_window missing used_percent / reset_after_seconds")

    used_f = max(0.0, min(100.0, float(used)))
    # remaining_percent 必须是 **float**：固件 QuotaPayload::parse() 要求
    # value.is<float>()，整数 JSON 在部分 ArduinoJson 版本下过不了这一关，
    # 会被判成 "Quota update rejected: invalid fields"。带小数点才稳。
    remaining = round(100.0 - used_f, 1)
    return {
        "remaining_percent": remaining,
        "reset_in_seconds": max(0, int(float(reset_in))),
    }


# ---------------------------------------------------------------------------
# 2. BLE 写入（连一次、写一次、立刻断）
# ---------------------------------------------------------------------------

async def resolve_target(name: str, address: str | None, verbose: bool):
    """--address 直连；否则扫描到叫 name 的设备。"""
    if address:
        return address

    from bleak import BleakScanner

    if verbose:
        log(f"[ble] scanning for '{name}' ...")
    box: dict = {"dev": None}

    def on_detect(device, _adv):
        if box["dev"] is None and (device.name or "").strip() == name:
            box["dev"] = device

    scanner = BleakScanner(detection_callback=on_detect)
    await scanner.start()
    try:
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline and box["dev"] is None:
            await asyncio.sleep(0.3)
    finally:
        await scanner.stop()

    device = box["dev"]
    if device is None:
        raise BridgeError(
            f"'{name}' not advertising. Power on the board; note that ChatGPT "
            "owning the LE link can hide it from scans."
        )
    if verbose:
        log(f"[ble] found {device.name} [{device.address}]")
    return device


async def push_once(snapshot: dict, name: str, address: str | None,
                    verbose: bool) -> str:
    from bleak import BleakClient

    target = await resolve_target(name, address, verbose)
    payload = json.dumps(snapshot, separators=(",", ":")).encode("utf-8")
    if len(payload) > MAX_PAYLOAD:
        raise BridgeError(f"payload too large ({len(payload)}B > {MAX_PAYLOAD}B)")

    kwargs = {}
    if os.name == "nt":
        # 绕过 OS 服务缓存：刚改过 GATT 表的固件会被旧缓存挡住。
        kwargs["winrt"] = {"use_cached_services": False}

    async with BleakClient(target, timeout=20.0, **kwargs) as client:
        if verbose:
            log(f"[ble] connected {client.address}")
        await client.write_gatt_char(QUOTA_WRITE_UUID, payload, response=False)
        if verbose:
            log(f"[ble] wrote {payload.decode('utf-8')}")
    # 退出 async-with ⇒ 立即断开，链路还给 HID 驱动。
    if verbose:
        log("[ble] disconnected")
    return payload.decode("utf-8")


# ---------------------------------------------------------------------------
# 3. 主循环
# ---------------------------------------------------------------------------

async def run(args) -> int:
    if args.verbose:
        log(f"codex home: {codex_home()}")
        log(f"quota chars: {QUOTA_SERVICE_UUID} / {QUOTA_WRITE_UUID}")
        log(f"proxy: {args.proxy or '(env/system default)'}")
        log(f"mode: {'dry-run (no BLE)' if args.dry_run else 'push to watch'}"
            f"{' / once' if args.once else ''}")

    failures = 0
    while True:
        started = time.monotonic()
        try:
            token = load_access_token()
            snapshot = read_quota(token, args.proxy)
            log(f"quota remaining={snapshot['remaining_percent']}% "
                f"reset_in={snapshot['reset_in_seconds']}s")
            if args.dry_run:
                log("dry-run: BLE untouched")
            else:
                await push_once(snapshot, args.device_name, args.address,
                                args.verbose)
                log("pushed to watch")
            failures = 0
        except BridgeError as exc:
            err(str(exc))
            failures += 1
        except asyncio.CancelledError:
            raise
        except Exception as exc:  # noqa: BLE001 - 常驻进程不该被未知异常打死
            err(f"{type(exc).__name__}: {exc}")
            failures += 1

        if args.once:
            return 1 if failures else 0

        delay = max(5.0, args.interval - (time.monotonic() - started))
        await asyncio.sleep(delay)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Push Codex quota to Codex Micro over BLE")
    parser.add_argument("--device-name", default=DEFAULT_DEVICE_NAME,
                        help=f"BLE name to scan for (default: {DEFAULT_DEVICE_NAME})")
    parser.add_argument("--address", default=None,
                        help="connect directly to this BLE address (skips scan)")
    parser.add_argument("--interval", type=int, default=120,
                        help="seconds between rounds (default: 120)")
    parser.add_argument("--once", action="store_true",
                        help="run one round and exit")
    parser.add_argument("--dry-run", action="store_true",
                        help="read quota only; do not touch BLE")
    parser.add_argument("--proxy", default=None,
                        help="HTTP(S) proxy URL override, e.g. http://127.0.0.1:10808 "
                             "(default: HTTP(S)_PROXY env / system settings)")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        print("\nstopped", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
