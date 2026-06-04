@echo off
REM ============================================================================
REM  flash_com4.bat  -  Build & flash LidarAGL firmware to the ESP32-S3 on COM4.
REM
REM  Why this wraps PowerShell:
REM    The ESP-IDF v6.0.1 EIM install exposes its environment through a PowerShell
REM    profile (there is no plain-cmd export), AND that profile leaves
REM    IDF_TOOLS_PATH / IDF_PYTHON_ENV_PATH empty - so idf.py looks for its venv in
REM    the wrong place and dies with "python.exe doesn't exist". We set them
REM    explicitly here (the same incantation that works by hand) and run idf.py.
REM
REM  Usage:
REM    Double-click it, or from a terminal:  flash_com4.bat
REM    Override the port (e.g. it moved):    flash_com4.bat COM7
REM ============================================================================
setlocal

REM --- Port: first argument, or default to COM4 (the S3's VID_303A port here). ---
set "PORT=%~1"
if "%PORT%"=="" set "PORT=COM4"

REM --- Always build from the directory this script lives in (the project root). ---
cd /d "%~dp0"

echo.
echo  Building ^& flashing LidarAGL  ->  %PORT%
echo  ----------------------------------------------
powershell -NoProfile -ExecutionPolicy Bypass -Command ". 'C:\Espressif\tools\Microsoft.v6.0.1.PowerShell_profile.ps1' | Out-Null; $env:IDF_TOOLS_PATH='C:\Espressif\tools'; $env:IDF_PYTHON_ENV_PATH='C:\Espressif\tools\python\v6.0.1\venv'; idf.py -p %PORT% flash; exit $LASTEXITCODE"

echo.
echo  ==== idf.py exited with code %ERRORLEVEL% ====
echo  (close the bench-sim / serial monitor first if you see a port-busy error)
echo.
pause
endlocal
