@echo off
setlocal

rem ============================================================
rem  CodexMicroWaveshare - compile only (no upload)
rem  Usage: double-click, or run from anywhere.
rem ============================================================

set "CLI=D:\DevTool\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
set "CFG=%LOCALAPPDATA%\Arduino15\arduino-cli.yaml"
set "LIBS=D:\DevTool\Arduino IDE\Project\libraries"
set "FQBN=esp32:esp32:waveshare_esp32_s3_touch_amoled_18:CDCOnBoot=cdc,PSRAM=enabled"

set "ROOT=%~dp0.."
set "SKETCH=%ROOT%\firmware\step5_power_audio"
set "BUILD=%ROOT%\build\step5_pa"

echo.
echo [build] sketch : %SKETCH%
echo [build] fqbn   : %FQBN%
echo [build] output : %BUILD%
echo.

"%CLI%" compile --fqbn "%FQBN%" --config-file "%CFG%" --libraries "%LIBS%" --build-path "%BUILD%" "%SKETCH%"

if errorlevel 1 (
    echo.
    echo ============================================
    echo  [FAIL] compile error - scroll up for "error:"
    echo ============================================
    exit /b 1
)

echo.
echo ============================================
echo  [OK] compiled.  Run flash.bat to upload.
echo ============================================
endlocal
