@echo off
REM === Codex Micro quota bridge launcher (step 6) ===
REM Pushes the Codex remaining quota to the watch every 120 seconds.
REM It writes ONE snapshot then disconnects immediately, so ChatGPT keeps
REM the single Windows LE link for its HID (push-to-talk / voice chat).
REM
REM Keep this window open while you want the quota to stay fresh.
REM Ctrl+C or closing the window stops it.

setlocal

REM Prefer the local scoop python that has bleak installed; fall back to PATH.
set "PYEXE=D:\DevTool\scoop\apps\python\current\python.exe"
if not exist "%PYEXE%" set "PYEXE=python"

"%PYEXE%" "%~dp0codex_quota_bridge.py" --interval 120 --verbose

echo.
echo (quota bridge stopped)
pause
