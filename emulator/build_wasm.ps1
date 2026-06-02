# =============================================================================
#  build_wasm.ps1 — compile the REAL LidarAGL logic to WebAssembly
# -----------------------------------------------------------------------------
#  Mirrors test/Makefile's host build (gcc -I../main -DUNIT_TEST) but swaps
#  gcc -> emcc and adds the module options the browser needs. Run this from the
#  emulator/ directory after any change to the C logic or the shim:
#
#      cd L:\Dev\LidarAGL\emulator
#      ./build_wasm.ps1
#
#  Output lands in emulator/dist/ as lidar_sim.js + lidar_sim.wasm.
#  Requires the Emscripten SDK (emcc). If emcc isn't on PATH, this script tries
#  to source emsdk_env.ps1 from a sibling L:\Dev\emsdk install automatically.
# =============================================================================

$ErrorActionPreference = 'Stop'

# --- Locate emcc -------------------------------------------------------------
# If emcc isn't already on PATH, pull it in from the standard emsdk location
# this project installs to (L:\Dev\emsdk). Adjust EMSDK_DIR if you put it
# somewhere else.
if (-not (Get-Command emcc -ErrorAction SilentlyContinue)) {
    $EMSDK_DIR = 'L:\Dev\emsdk'
    $envScript = Join-Path $EMSDK_DIR 'emsdk_env.ps1'
    if (Test-Path $envScript) {
        Write-Host "emcc not on PATH; sourcing $envScript ..." -ForegroundColor Yellow
        & $envScript
    }
}

# Fail early with a clear message if emcc still isn't available.
if (-not (Get-Command emcc -ErrorAction SilentlyContinue)) {
    Write-Error "emcc not found. Install the Emscripten SDK (see README.md) and retry."
}

# --- Paths -------------------------------------------------------------------
# Resolve everything relative to THIS script so it works no matter where it's
# invoked from. main/ holds the firmware sources we compile unchanged.
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$main = Join-Path $here '..\main'
$dist = Join-Path $here 'dist'

# Ensure the output directory exists.
New-Item -ItemType Directory -Force -Path $dist | Out-Null

# --- Compile -----------------------------------------------------------------
# Compile set = the three PURE firmware modules + our flat-ABI shim.
# (callouts.c is intentionally excluded — see sim_glue.c header for why.)
Write-Host "Building lidar_sim.wasm ..." -ForegroundColor Cyan

emcc `
    "$main\state_machine.c" `
    "$main\sensor_profile.c" `
    "$main\audio_math.c" `
    "$here\sim_glue.c" `
    -I"$main" -DUNIT_TEST -O2 `
    "-sMODULARIZE=1" "-sEXPORT_ES6=1" "-sEXPORT_NAME=createLidarSim" `
    "-sENVIRONMENT=web" "-sALLOW_MEMORY_GROWTH=1" `
    "-sEXPORTED_RUNTIME_METHODS=cwrap,ccall" `
    -o "$dist\lidar_sim.js"

Write-Host "Done -> $dist\lidar_sim.js (+ .wasm)" -ForegroundColor Green
