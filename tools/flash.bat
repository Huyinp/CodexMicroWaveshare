@echo off
setlocal

rem ============================================================
rem  CodexMicroWaveshare - compile then upload
rem  Usage: flash.bat            (defaults to COM11)
rem         flash.bat COM6       (explicit port)
rem  NOTE: compile MUST succeed first - otherwise the old
rem        firmware in build\ would be flashed silently.
rem ============================================================

set "CLI=D:\DevTool\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
set "CFG=%LOCALAPPDATA%\Arduino15\arduino-cli.yaml"
set "LIBS=D:\DevTool\Arduino IDE\Project\libraries"
set "FQBN=esp32:esp32:waveshare_esp32_s3_touch_amoled_18:CDCOnBoot=cdc,PSRAM=enabled"

set "ROOT=%~dp0.."
set "SKETCH=%ROOT%\firmware\step5_power_audio"
set "BUILD=%ROOT%\build\step5_pa"

set "PORT=%~1"
if "%PORT%"=="" set "PORT=COM11"

echo.
echo [flash] port   : %PORT%
echo [flash] sketch : %SKETCH%
echo.

echo [1/2] compiling ...
"%CLI%" compile --fqbn "%FQBN%" --config-file "%CFG%" --libraries "%LIBS%" --build-path "%BUILD%" "%SKETCH%"

if errorlevel 1 (
    echo.
    echo ============================================
    echo  [FAIL] compile error - NOT uploading.
    echo  (fix the error, then run flash.bat again)
    echo ============================================
    exit /b 1
)

echo.
echo [2/2] uploading to %PORT% ...
"%CLI%" upload --fqbn "%FQBN%" --config-file "%CFG%" --port %PORT% --input-dir "%BUILD%" "%SKETCH%"

if errorlevel 1 (
    echo.
    echo ============================================
    echo  [FAIL] upload to %PORT% failed.
    echo  Common causes:
    echo    - port busy: close Serial Monitor / Tabby /
    echo      any script holding the COM port
    echo    - wrong port: run "flash.bat COM6"
    echo ============================================
    exit /b 1
)

echo.
echo ============================================
echo  [OK] flashed to %PORT%.
echo ============================================
endlocal
