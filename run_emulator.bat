@echo off
REM ============================================================================
REM  run_emulator.bat - one-click build + serve the LidarAGL web emulator
REM ----------------------------------------------------------------------------
REM  1. Compiles the real firmware logic to WASM via emulator/build_wasm.ps1
REM  2. Launches a local http server from the REPO ROOT (so emulator/ AND
REM     assets/ are both reachable - the WAV clips live under assets/).
REM  3. Pops open the browser at the emulator page.
REM
REM  Just double-click me. Run from anywhere - paths are anchored to this file.
REM ============================================================================

REM --- Anchor to this script's own directory (the repo root) ------------------
cd /d "%~dp0"

REM --- Step 1: build the WASM module -----------------------------------------
echo [run_emulator] Building WASM...
powershell -NoProfile -ExecutionPolicy Bypass -File "emulator\build_wasm.ps1"
if errorlevel 1 (
    echo.
    echo [run_emulator] WASM build FAILED - see the error above. Aborting.
    pause
    exit /b 1
)

REM --- Step 2: open the browser (slight head start before the server blocks) --
echo [run_emulator] Opening browser at http://localhost:8000/emulator/index.html
start "" "http://localhost:8000/emulator/index.html"

REM --- Step 3: serve from repo root. Ctrl+C in this window stops the server. --
echo [run_emulator] Serving on http://localhost:8000  (Ctrl+C to stop)
python -m http.server 8000
