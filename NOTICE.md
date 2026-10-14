# Notices and Disclaimers

## Source and copyright

This repository is a port of
[Douiuiuiz/codex-micro-waveshare](https://github.com/Douiuiuiz/codex-micro-waveshare)
(the Waveshare ESP32-S3-Touch-AMOLED-**1.75C** build) to the Waveshare
ESP32-S3-Touch-AMOLED-**1.8** board, reusing its protocol layer verbatim and
rewriting only the board-specific drivers and the dashboard layout.

- **Douiuiuiz/codex-micro-waveshare** is itself a port of
  `digitsisyph/codex-micro-stopwatch` (M5Stack StopWatch C152 + macOS), which
  in turn derives from `imliubo/codex-micro-4-core2`.
- Original source in that lineage is **Copyright (c) 2026 imliubo**; StopWatch
  port changes **Copyright (c) 2026 codex-micro-4-stopwatch contributors**, and
  is distributed under the **MIT License**.
- Modifications for the 1.8 board are **Copyright (c) 2026 yp.hu**, also under
  the MIT License (see `LICENSE`).

## Independent project and trademarks

This is an independent, unofficial compatibility project. It is not affiliated
with, authorized by, endorsed by, sponsored by, or supported by OpenAI, Work
Louder, M5Stack, or any of their affiliates.

OpenAI, ChatGPT, and Codex are trademarks or registered trademarks of OpenAI.
Work Louder and Codex Micro product branding belong to their respective owners.
M5Stack, Core2, and StopWatch are trademarks or product names of M5Stack. All
other names and marks are the property of their respective owners.

Names and marks are used only to identify compatibility and the intended use of
the software. Their use does not imply an official relationship.

## Protocol and compatibility disclaimer

This firmware implements behavior observed from an undocumented vendor HID
interface. The BLE device name, manufacturer string, vendor ID, product ID,
usage page, report ID, key IDs, and RPC method names are used only so a
compatible host can recognize and communicate with the device. These
identifiers are not assigned to this project and must not be interpreted as
proof of certification, ownership, authenticity, or official-device status.

No representation is made that the implementation is complete, accurate,
secure, permanently compatible, or suitable for production use. OpenAI, Work
Louder, ChatGPT Desktop, Windows, ESP32 libraries, or the board vendor may
change at any time and may cause partial or complete loss of functionality.

## Hardware, security, and data disclaimer

Flashing third-party firmware can erase existing software, settings, or data
and can leave hardware temporarily unusable. Verify the target board and keep a
known-good recovery path before flashing. You assume all risk for hardware
damage, data loss, radio behavior, battery use, security, and regulatory
compliance.

BLE pairing uses a "Just Works" flow without passkey authentication. Use the
device only in a trusted environment and remove stale pairings from both the
host and device when appropriate.

This firmware does **not** connect directly to OpenAI. Audio for voice chat is
captured by the **computer's microphone** and transcribed by the desktop app;
the device only sends "start/stop" control signals over BLE and never carries
audio bytes. The device sends button events, directional actions, and device
status to the paired computer; the host OS and applications control how local
input is processed.

## No warranty

THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS", WITHOUT WARRANTY OF ANY
KIND. TO THE MAXIMUM EXTENT PERMITTED BY LAW, THE AUTHORS AND COPYRIGHT HOLDERS
ARE NOT LIABLE FOR ANY CLAIM, DAMAGE, LOSS, OR OTHER LIABILITY ARISING FROM USE,
MODIFICATION, FLASHING, PAIRING, DISTRIBUTION, OR INABILITY TO USE THIS PROJECT.
