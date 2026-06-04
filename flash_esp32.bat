@echo off
REM ============================================================================
REM  flash_esp32.bat  -  Build & flash LidarAGL firmware to an ESP32-S3.
REM
REM  Why this wraps PowerShell:
REM    The ESP-IDF v6.0.1 EIM install exposes its environment through a PowerShell
REM    profile (there is no plain-cmd export), AND that profile leaves
REM    IDF_TOOLS_PATH / IDF_PYTHON_ENV_PATH empty - so idf.py looks for its venv in
REM    the wrong place and dies with "python.exe doesn't exist". We set them
REM    explicitly here (the same incantation that works by hand) and run idf.py.
REM
REM  Port selection:
REM    flash_esp32_menu.ps1 draws an arrow-key list of COM devices and remembers
REM    your pick in flash_esp32_default.txt (git-ignored). Just hit Enter to
REM    re-flash the last-used port.
REM
REM  Usage:
REM    Double-click it           -> arrow-key picker (Enter = last-used port).
REM    flash_esp32.bat           -> same picker.
REM    flash_esp32.bat COM7      -> skip the picker and flash COM7 directly.
REM ============================================================================
setlocal enabledelayedexpansion

REM --- Always build from the directory this script lives in (the project root). ---
cd /d "%~dp0"

REM ----------------------------------------------------------------------------
REM  Run the interactive picker. It renders the menu to the console itself and
REM  prints ONLY the chosen "COMx" on stdout, which we capture here. Any
REM  argument (e.g. COM7) is forwarded so the picker can short-circuit.
REM ----------------------------------------------------------------------------
set "PORT="
for /f "usebackq delims=" %%P in (`powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0flash_esp32_menu.ps1" "%~1"`) do set "PORT=%%P"

REM --- Picker cancelled (Esc) or found no ports -> nothing to flash. -----------
if not defined PORT (
    echo.
    echo  No port selected - nothing flashed.
    echo.
    pause
    endlocal
    exit /b 1
)

echo.
echo  Building ^& flashing LidarAGL  -^>  %PORT%
echo  ----------------------------------------------
powershell -NoProfile -ExecutionPolicy Bypass -Command ". 'C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1' | Out-Null; $env:IDF_TOOLS_PATH='C:\Espressif\tools'; $env:IDF_PYTHON_ENV_PATH='C:\Espressif\tools\python\v6.0.1\venv'; idf.py -p %PORT% flash; exit $LASTEXITCODE"

echo.
echo  ==== idf.py exited with code %ERRORLEVEL% ====
echo  (close the bench-sim / serial monitor first if you see a port-busy error)
echo.
pause
endlocal
exit /b 0
