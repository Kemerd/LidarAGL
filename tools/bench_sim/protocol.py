#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
protocol.py -- LidarAGL bench-sim protocol constants.

This is the Python MIRROR of the firmware's protocol definitions. The values
here MUST match the firmware byte-for-byte:

    LWNX framing / commands ...... main/lwnx.h
    bench-control command + ops .. main/config.h  (LWNX_CMD_BENCH_CTRL, OP_*)
    cm<->ft + lost-signal ........ main/config.h  (CM_TO_FT, SF30_LOST_SIGNAL_CM)
    distance field offset ........ main/sf30c.c   (firstReturnFiltered @ offset 6)

If you change any of these on the firmware side, change them here too.
"""

# --- LWNX framing (main/lwnx.h) ----------------------------------------------
LWNX_START_BYTE        = 0xAA       # start-of-frame
LWNX_MAX_FRAME         = 256        # parser working-buffer cap (informational)

# --- LWNX command ids (main/lwnx.h) ------------------------------------------
LWNX_CMD_PRODUCT_NAME  = 0          # read: product-name string
LWNX_CMD_DISTANCE_CFG  = 29         # write u32: which distance fields stream
LWNX_CMD_STREAM        = 30         # write u32: 5 = stream distance data
LWNX_CMD_DISTANCE_DATA = 44         # stream: distance frame (cm)
LWNX_CMD_UPDATE_RATE   = 76         # write u32: output rate code

# --- Bench-control command + opcodes (main/config.h) -------------------------
# Custom command id carried in the same LWNX framing/CRC as the sensor uses, but
# host->device. Safely outside LightWare's well-known set.
LWNX_CMD_BENCH_CTRL    = 200

OP_HELLO        = 0x01              # attach: enter sim mode this boot
OP_BENCH_REAL   = 0x02             # attach: REAL sensor + scaled-AGL debug boot
                                   # may carry an optional '<f' float after the
                                   # opcode: the AGL scale gain (overrides the
                                   # firmware's compile-time BENCH_SCALE_GAIN)
OP_ENTER_CONFIG = 0x10             # attach + run the boot config menu this boot
OP_REBOOT       = 0x11             # esp_restart(); host re-attaches afterwards
OP_MENU_NEXT    = 0x20             # config menu: single-tap (cycle a selection)
OP_MENU_CONFIRM = 0x21             # config menu: double-tap (confirm a selection)

# --- Distance frame layout (main/sf30c.c, main/config.h) ---------------------
# The firmware reads ONLY the int16 LE centimetre value at this payload offset
# ("firstReturnFiltered"). The real SF30 distance frame is ~20 bytes; we send a
# realistic-length payload with the distance placed at the right offset.
DISTANCE_PAYLOAD_LEN   = 20        # bytes (matches the real SF30/C frame size)
DISTANCE_OFFSET_FILT   = 6         # int16 LE cm offset the firmware decodes

# --- Units / sentinels (main/config.h) ---------------------------------------
CM_TO_FT               = 0.0328084 # the firmware's single cm->ft factor
SF30_LOST_SIGNAL_CM    = 16000     # sentinel the SF30 emits on no-return

# Bench real-sensor AGL scale gain (main/config.h BENCH_SCALE_GAIN). This is the
# default the GUI seeds its field with; the firmware clamps incoming values to
# [1, 1000] (see bench_attach_detected). x100 => +4 real ft sweeps ~0..400 ft AGL.
BENCH_SCALE_GAIN_DEFAULT = 100.0
BENCH_SCALE_GAIN_MIN     = 1.0
BENCH_SCALE_GAIN_MAX     = 1000.0

# Nominal serial baud. The USB-Serial-JTAG is a CDC device and ignores the baud
# rate, but pyserial still requires a value when opening the port.
NOMINAL_BAUD           = 115200

# The real sensor's active streaming rate (main/app_main.c SF30_RATE_CODE_ACTIVE
# = 8, which LightWare documents as ~78 readings/sec). We stream cmd-44 frames at
# this cadence so the firmware's poll/EMA/timing behave exactly as in flight.
DEFAULT_STREAM_HZ      = 78
