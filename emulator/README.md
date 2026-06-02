# LidarAGL Web Emulator

A browser test bench for the LidarAGL altitude logic. Drag a simulated aircraft
up and down an altitude tape and watch/hear the **real firmware** fire its
callouts, drive the presence tone, and walk its state machine — **no ESP32, no
SF30 sensor, no flashing required.**

The trick: the firmware already isolates its safety-critical logic
(`state_machine.c`, `audio_math.c`, `sensor_profile.c`) from the ESP-IDF
hardware, with a host build that compiles those modules with plain `gcc`. This
emulator compiles the **same C files to WebAssembly** via Emscripten and drives
them from a canvas UI. So a green run here is evidence the *firmware logic
itself* is correct — not a JavaScript re-implementation that could drift.

```
emulator/
  index.html      # canvas UI (Apple-HIG minimal)
  sim.js          # WASM glue, render loop, WebAudio
  sim_glue.c      # tiny flat-ABI shim over the firmware's sm_* + audio_math
  build_wasm.ps1  # emcc build (PowerShell)
  build_wasm.sh   # emcc build (bash/WSL)
  dist/           # build output: lidar_sim.js + lidar_sim.wasm (gitignored)
```

## Prerequisites

- **Emscripten SDK** (`emcc`). One-time install:

  ```powershell
  git clone https://github.com/emscripten-core/emsdk.git L:\Dev\emsdk
  cd L:\Dev\emsdk
  ./emsdk install latest
  ./emsdk activate latest
  ```

  The build scripts auto-source `L:\Dev\emsdk\emsdk_env.ps1` if `emcc` isn't
  already on your PATH, so you don't have to activate it in every shell.

- **Python 3** (only to serve files over http for local testing).

## Build

From the `emulator/` directory, after any change to the C logic or the shim:

```powershell
cd L:\Dev\LidarAGL\emulator
./build_wasm.ps1            # or, in bash/WSL:  ./build_wasm.sh
```

This produces `emulator/dist/lidar_sim.js` and `emulator/dist/lidar_sim.wasm`.

## Run

WASM and the WAV clips must be served over http (not `file://`). Serve from the
**repo root** so both `emulator/` and `assets/` are reachable:

```powershell
cd L:\Dev\LidarAGL
python -m http.server 8000
```

Open <http://localhost:8000/emulator/index.html>, click **Start audio** (or just
start dragging), and fly.

## What to try

With **SF30/C** selected, drag the aircraft from the ground up past the dashed
**ARM** line to cruise, then back down:

- **Silent climb-out** — no callouts on the way up.
- **Descent ladder** — on the way down you hear `200, 100, 50, 40, 30, 20, 10`,
  each once, in order.
- **Presence tone** — silent above 100 ft; below it the tone fades in and its
  **pitch rises** toward the ground (600 → 1800 Hz), louder by 50 ft.
- **Hysteresis** — jitter around 50 ft after it fires and it won't re-fire; you
  must climb ~20 ft above it to re-arm (go-around).
- **Profile switch** — pick **SF30/D** and the tape gains 500/400/300 ticks, the
  cruise line jumps to 500, and a high descent fires those extra callouts. The
  SF30/C never does — proof the real per-profile C table is in charge.

Every number you hear and every state you see comes from the compiled
`sm_step()` / `audio_math` — the firmware brain, running in your browser.

## Notes

- `callouts.c` is intentionally **not** compiled into the WASM: its only pure
  piece shares a translation unit with embedded-binary (`_binary_*_pcm`) symbols
  that don't exist outside the firmware build. The emulator doesn't need it — the
  callout WAVs are named by height (`50.wav`), which comes straight from the
  profile's callout list.
- No existing firmware source is modified; the `emulator/` directory is invisible
  to the ESP-IDF build (`main/CMakeLists.txt` uses an explicit source list).
