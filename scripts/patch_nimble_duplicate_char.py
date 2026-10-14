#!/usr/bin/env python3
"""Make Arduino-ESP32's BLE library tolerate duplicate characteristic UUIDs on NimBLE.

Why this is needed
------------------
HID over GATT (0x1812) requires several Report characteristics that all share
the UUID 0x2A4D; input / output / feature reports are told apart only by their
0x2908 Report Reference descriptor.  Arduino-ESP32's BLEService::addCharacteristic()
drops the second characteristic with the same UUID:

    #if defined(CONFIG_NIMBLE_ENABLED)
      if (pExisting != nullptr) {
        pExisting->m_removed = 0;
      } else
    #endif
      {
        m_characteristicMap.setByUUID(pCharacteristic, pCharacteristic->getUUID());
      }

On NimBLE the characteristic map is keyed by BLECharacteristic* (not by UUID),
so being able to hold duplicates is fine -- the getByUUID() lookup is only a
convenience accessor.  Without the output Report characteristic the host can
enumerate the HID device (Windows still creates a HIDClass node) but every
hid_write() fails with ERROR_INVALID_PARAMETER (0x57).

What this script does
---------------------
Rewrites the NimBLE branch so the characteristic is always inserted into the
map.  Idempotent: running it twice is a no-op.  Pass --revert to restore the
stock code.

Usage:
  python patch_nimble_duplicate_char.py [--core-root <path>] [--revert]
"""
from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

DEFAULT_CORE_ROOT = Path(
    r"C:\Users\yp.hu\AppData\Local\Arduino15\packages\esp32\hardware\esp32\3.3.7-cn"
)

STOCK = """#if defined(CONFIG_NIMBLE_ENABLED)
  if (pExisting != nullptr) {
    pExisting->m_removed = 0;
  } else
#endif
  {
    // Remember this characteristic in our map of characteristics.  At this point, we can lookup by UUID
    // but not by handle.  The handle is allocated to us on the ESP_GATTS_ADD_CHAR_EVT.
    m_characteristicMap.setByUUID(pCharacteristic, pCharacteristic->getUUID());
  }"""

PATCHED = """#if defined(CONFIG_NIMBLE_ENABLED)
  // NimBLE 的 m_uuidMap 以 BLECharacteristic* 为 key，同 UUID 的多个 characteristic
  // 完全可以共存（HOGP 的 HID 服务就是这样：几个 Report characteristic 全用 0x2A4D，
  // 靠 0x2908 Report Reference 区分 input/output/feature）。
  // 原版在这里一旦 getByUUID() 命中就跳过 setByUUID()，第二个同 UUID 的
  // characteristic 永远进不了 GATT 表 —— 于是 output report 消失、主机 hid_write 恒 87。
  if (pExisting != nullptr) {
    pExisting->m_removed = 0;
  }
#endif
  {
    // Remember this characteristic in our map of characteristics.  At this point, we can lookup by UUID
    // but not by handle.  The handle is allocated to us on the ESP_GATTS_ADD_CHAR_EVT.
    m_characteristicMap.setByUUID(pCharacteristic, pCharacteristic->getUUID());
  }"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--core-root", type=Path, default=DEFAULT_CORE_ROOT)
    parser.add_argument("--revert", action="store_true")
    args = parser.parse_args()

    target = args.core_root / "libraries" / "BLE" / "src" / "BLEService.cpp"
    if not target.is_file():
        print(f"BLEService.cpp not found at {target}", file=sys.stderr)
        return 1

    source = target.read_text(encoding="utf-8")
    want, other = (STOCK, PATCHED) if args.revert else (PATCHED, STOCK)

    if want in source:
        print(f"already {'reverted' if args.revert else 'patched'} - nothing to do")
        return 0
    if other not in source:
        print("anchor block not found; the core source may have changed", file=sys.stderr)
        return 1

    backup = target.with_suffix(".cpp.orig")
    if not backup.exists():
        shutil.copy2(target, backup)
        print(f"backup written: {backup}")

    target.write_text(source.replace(other, want), encoding="utf-8")
    print(f"{'reverted' if args.revert else 'patched'}: {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
