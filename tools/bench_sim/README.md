# LidarAGL Hardware-in-the-Loop Bench Sim

Test the **real firmware** with headphones and **no LiDAR connected**. This tool
streams byte-accurate simulated SF30/C **LWNX** distance frames into the board's
native USB-Serial-JTAG port. The firmware can't tell them from the real sensor —
they run through the exact same parser → CRC → distance → AGL → callout/tone
chain — so what you hear on the bench is what you'll get in the aircraft.

## What it does

- **Drag the altitude tape** (or fly a configurable **ILS approach**) and the box
  reacts in real time: presence tone fading in below 100 ft, pitch ascending as
  you near the ground, spoken callouts at the profile heights, tone fading out in
  the flare.
- **Realistic sensor model**: gaussian range jitter + occasional lost-signal
  dropouts, so you exercise the firmware's range filtering and last-good hold.
- **Unreliable sensor (fault injection)**: deliberately corrupt the stream —
  one-shot **spikes**, continuous **random garbage**, periodic **corruption
  bursts**, and a **stuck-value** fault — to prove the robust range pipeline
  (median-of-drain → Hampel gate → re-acquire, `main/range_filter.c`) holds the
  line on real hardware. See the pass/fail table below.
- **Self-learned ground offset**: stream a few feet of "ground" while the box
  boots and its real ground-fill learns the mount offset (AGL = range − ground).
- **Fully remote config menu**: reboot into the boot config menu and drive it
  (audio mode → start-altitude → volume) entirely from the PC.

## Install

```
pip install -r tools/bench_sim/requirements.txt
```
(needs `pyserial` and `customtkinter`)

## Run

```
python tools/bench_sim/main.py
```

1. **Pick the COM port** (the ESP32-S3 shows as "USB Serial Device" / VID 303A)
   and click **Connect**. (Close `idf.py monitor` first — only one program can
   own the port.)
2. Click **Enter Sim**, then tap the board's **EN/reset** button (or **Reset
   Board**). When you see `BENCH SIM MODE…` in the Device Log, the status turns
   green — you're attached. (Opening a USB-Serial-JTAG port doesn't reset the
   chip, so the box has to reboot once for the bench to catch its boot window.)
3. Put headphones on the PCM5102A and **drag the altitude tape** down through the
   callout ladder, or set a glideslope/speed and click **Fly approach**.

### Unreliable sensor — reliability torture tests

The **Unreliable sensor** card breaks the stream on purpose. Faults corrupt the
**values** (the LWNX CRC on this transport drops byte-level garbage before the
decoder — value corruption is exactly what survives the checksum-free legacy
serial path in the aircraft, so it's the right thing to inject). Park the tape
at 0 ft, put headphones on, and torture away:

| Fault | What it simulates | Expected (v1.59 filter) |
|-------|-------------------|--------------------------|
| **Spike** ×1 frame | isolated corrupt sample (the taxi-incident trigger) | **SILENCE** — dies in the drain median |
| **Spike** ×5–20 frames | short corrupt burst | **SILENCE** — Hampel gate holds last-good |
| **Random garbage** ≤20 % | uncorrelated corruption | **SILENCE** — uncorrelated values never win the median or re-acquire |
| **Garbage bursts** (every N s) | light-sleep wake-edge corruption | **SILENCE** — each burst is held, stream resumes cleanly |
| **Stuck value** | correlated corruption / a genuine terrain step (indistinguishable!) | box **follows it after ~3 polls** — the designed re-acquire path, *not* a failure. If the stuck level is above 100 ft it will also (silently) arm after the 1.5 s dwell, and **releasing** the fault speaks exactly **one low number** (e.g. "ten") as the range snaps back down through the ladder — a genuine crossing from the firmware's point of view |

The `corrupted frames` counter (amber while any fault is armed) tallies what was
injected, so a session is auditable against what was — or wasn't — heard. Any
callout or tone heard during the silent-expectation rows is a regression:
capture the Device Log and the counter.

A pre-v1.59 firmware fails these instantly (a single 400 ft spike arms the
ladder and speaks a phantom descending "50 40 30 20 10") — flash old firmware
if you want to hear the difference.

### Config menu (remote)

Click **Enter Config** → the box reboots into the boot config menu. Use **Next**
(cycle) and **Confirm** (accept) to step through audio mode, callout start
altitude, and master volume. On commit it reboots; click **Enter Sim** to bench
again.

## Safety

Sim mode is **runtime-only — never written to NVS**. A plain power cycle always
brings the box up on the **real sensor**, so a unit can't accidentally end up in
sim mode in the aircraft. The PC must be connected and actively attaching for the
box to enter sim at all.

## Files

| File | Role |
|------|------|
| `lwnx_codec.py` | byte-exact port of the firmware's CRC + frame builder |
| `protocol.py` | constants mirrored from `main/lwnx.h` + `main/config.h` |
| `serial_worker.py` | thread-safe pyserial TX + RX-log reader |
| `altitude_model.py` | manual drag source + ILS glideslope generator |
| `noise.py` | gaussian jitter + lost-signal dropouts + FaultInjector (spikes/bursts/random/stuck) |
| `sim_engine.py` | streams cmd-44 frames at ~78 Hz |
| `controller.py` | bench-control senders + boot-window attach spammer |
| `app_ctk.py` | CustomTkinter GUI |
| `main.py` | entry point |
| `test_codec.py` | offline parity check of the encoder |

Verify the encoder offline first:

```
python tools/bench_sim/test_codec.py
```

## Troubleshooting

- **Nothing happens after Connect** — the box was already past its boot window.
  Click **Enter Sim** and tap **EN** so it reboots into the window.
- **Port busy / access denied** — close `idf.py monitor` or any other serial
  monitor holding the port.
- **No callouts, only tone (or silence)** — confirm the audio mode in the config
  menu includes callouts, and that the clips were embedded (the firmware logs
  the ground reference and `callout NN ft` lines as you descend).
