#!/usr/bin/env bash
# =============================================================================
#  build_wasm.sh — compile the REAL LidarAGL logic to WebAssembly (bash/WSL)
# -----------------------------------------------------------------------------
#  Identical to build_wasm.ps1, for a bash shell. Run from anywhere:
#
#      ./build_wasm.sh
#
#  Output lands in emulator/dist/ as lidar_sim.js + lidar_sim.wasm.
#  Requires emcc on PATH (source your emsdk_env.sh first if needed).
# =============================================================================
set -euo pipefail

# --- Locate emcc -------------------------------------------------------------
# Try to source a sibling emsdk install if emcc isn't already available.
if ! command -v emcc >/dev/null 2>&1; then
    for env in "$HOME/emsdk/emsdk_env.sh" "/l/Dev/emsdk/emsdk_env.sh" "L:/Dev/emsdk/emsdk_env.sh"; do
        if [ -f "$env" ]; then
            echo "emcc not on PATH; sourcing $env ..."
            # shellcheck disable=SC1090
            source "$env"
            break
        fi
    done
fi

if ! command -v emcc >/dev/null 2>&1; then
    echo "error: emcc not found. Install the Emscripten SDK (see README.md)." >&2
    exit 1
fi

# --- Paths -------------------------------------------------------------------
# Resolve relative to this script so the build is location-independent.
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
main="$here/../main"
dist="$here/dist"
mkdir -p "$dist"

# --- Compile -----------------------------------------------------------------
# Three PURE firmware modules + the flat-ABI shim (callouts.c excluded on purpose).
echo "Building lidar_sim.wasm ..."

emcc \
    "$main/state_machine.c" \
    "$main/sensor_profile.c" \
    "$main/audio_math.c" \
    "$here/sim_glue.c" \
    -I"$main" -DUNIT_TEST -O2 \
    -sMODULARIZE=1 -sEXPORT_ES6=1 -sEXPORT_NAME=createLidarSim \
    -sENVIRONMENT=web -sALLOW_MEMORY_GROWTH=1 \
    -sEXPORTED_RUNTIME_METHODS=cwrap,ccall \
    -o "$dist/lidar_sim.js"

echo "Done -> $dist/lidar_sim.js (+ .wasm)"
