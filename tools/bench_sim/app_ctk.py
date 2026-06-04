#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
app_ctk.py -- CustomTkinter GUI for the LidarAGL hardware-in-the-loop bench.

Dark, rounded, cockpit-flavoured. Drag the altitude tape (or fly an ILS) and the
bench streams byte-accurate SF30/C frames to the real firmware over USB so you
can hear the callouts + presence tone on headphones with no LiDAR attached.

The toolkit-agnostic core lives in the sibling modules (serial_worker, sim_engine,
altitude_model, noise, controller, lwnx_codec); this file is purely the view +
wiring, so swapping to another GUI toolkit later only means rewriting this file.
"""

import queue

import customtkinter as ctk

import protocol as P
from serial_worker import SerialWorker, list_ports
from altitude_model import AltitudeModel
from sim_engine import SimEngine
from controller import BenchController

# --- Apple-ish palette (mirrors the web emulator's tokens) -------------------
ACCENT = "#0a84ff"      # iOS blue
GOOD   = "#30d158"      # iOS green  (sim active)
WARN   = "#ff9f0a"      # iOS orange (attaching)
DANGER = "#ff453a"      # iOS red    (reboot / disconnected)
MUTED  = "#8e8e93"

ALT_MAX_FT = 1000.0     # top of the altitude tape (climb well past sensor range)


class BenchApp(ctk.CTk):
    def __init__(self):
        super().__init__()

        # --- core (toolkit-agnostic) ----------------------------------------
        self._rx_q = queue.Queue()
        self.serial = SerialWorker(on_rx=self._on_rx, on_status=self._on_status)
        self.model = AltitudeModel()
        self.engine = SimEngine(self.serial, self.model)
        self.ctrl = BenchController(self.serial)

        self.sim_active = False
        self._slider_guard = False     # suppress slider callback during sync

        # --- window ----------------------------------------------------------
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("dark-blue")
        self.title("LidarAGL · Hardware Bench Sim")
        self.geometry("1040x720")
        self.minsize(900, 640)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(1, weight=1)

        self._build_topbar()
        self._build_altitude_panel()
        self._build_controls()
        self._build_log()

        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(100, self._tick)    # UI refresh + RX drain loop

    # =========================================================================
    #  Layout
    # =========================================================================

    def _build_topbar(self):
        bar = ctk.CTkFrame(self, corner_radius=14)
        bar.grid(row=0, column=0, columnspan=2, padx=14, pady=(14, 7), sticky="ew")
        bar.grid_columnconfigure(5, weight=1)

        ctk.CTkLabel(bar, text="Port", text_color=MUTED).grid(
            row=0, column=0, padx=(14, 6), pady=12)
        self.port_menu = ctk.CTkOptionMenu(bar, values=["(refresh)"], width=240)
        self.port_menu.grid(row=0, column=1, pady=12)
        ctk.CTkButton(bar, text="↻", width=36, command=self._refresh_ports).grid(
            row=0, column=2, padx=6, pady=12)

        self.connect_btn = ctk.CTkButton(bar, text="Connect", width=120,
                                         command=self._toggle_connect)
        self.connect_btn.grid(row=0, column=3, padx=6, pady=12)

        self.status_dot = ctk.CTkLabel(bar, text="●", text_color=DANGER,
                                       font=ctk.CTkFont(size=18))
        self.status_dot.grid(row=0, column=4, padx=(12, 4), pady=12)
        self.status_lbl = ctk.CTkLabel(bar, text="disconnected", text_color=MUTED)
        self.status_lbl.grid(row=0, column=5, padx=4, pady=12, sticky="w")

        self._refresh_ports()

    def _build_altitude_panel(self):
        panel = ctk.CTkFrame(self, corner_radius=16)
        panel.grid(row=1, column=0, padx=(14, 7), pady=7, sticky="ns")
        panel.grid_rowconfigure(2, weight=1)

        ctk.CTkLabel(panel, text="ALTITUDE  (AGL ft)",
                     text_color=MUTED, font=ctk.CTkFont(size=12, weight="bold")
                     ).grid(row=0, column=0, padx=24, pady=(16, 2))

        self.agl_value = ctk.CTkLabel(panel, text="0",
                                      font=ctk.CTkFont(size=44, weight="bold"))
        self.agl_value.grid(row=1, column=0, padx=24, pady=(0, 8))

        # Vertical "tape": drag it to command altitude by hand. Bottom = 0 ft,
        # top = ALT_MAX_FT. Dragging pauses any running ILS (you've taken over).
        self.alt_slider = ctk.CTkSlider(
            panel, from_=0, to=ALT_MAX_FT, orientation="vertical", height=360,
            command=self._on_slider)
        self.alt_slider.set(0)
        self.alt_slider.grid(row=2, column=0, padx=24, pady=8)

        ctk.CTkButton(panel, text="Reset to ground", fg_color="transparent",
                      border_width=1, command=self._reset_ground).grid(
            row=3, column=0, padx=24, pady=(8, 16), sticky="ew")

    def _build_controls(self):
        col = ctk.CTkScrollableFrame(self, corner_radius=0, fg_color="transparent")
        col.grid(row=1, column=1, padx=(7, 14), pady=7, sticky="nsew")
        col.grid_columnconfigure(0, weight=1)

        self._build_sim_card(col)
        self._build_ils_card(col)
        self._build_noise_card(col)
        self._build_menu_card(col)

    def _card(self, parent, title):
        card = ctk.CTkFrame(parent, corner_radius=16)
        card.grid(sticky="ew", pady=7)
        card.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(card, text=title, font=ctk.CTkFont(size=15, weight="bold")
                     ).grid(row=0, column=0, columnspan=3, padx=16, pady=(14, 6),
                            sticky="w")
        return card

    def _build_sim_card(self, parent):
        card = self._card(parent, "Device")
        self.sim_state_lbl = ctk.CTkLabel(card, text="not in sim mode",
                                          text_color=MUTED)
        self.sim_state_lbl.grid(row=1, column=0, columnspan=3, padx=16,
                                sticky="w")

        row = ctk.CTkFrame(card, fg_color="transparent")
        row.grid(row=2, column=0, columnspan=3, padx=12, pady=(8, 14), sticky="ew")
        for i in range(3):
            row.grid_columnconfigure(i, weight=1)
        ctk.CTkButton(row, text="Enter Sim", command=self._enter_sim
                      ).grid(row=0, column=0, padx=4, sticky="ew")
        ctk.CTkButton(row, text="Enter Config", fg_color=WARN, text_color="#000",
                      command=self._enter_config).grid(row=0, column=1, padx=4,
                                                       sticky="ew")
        ctk.CTkButton(row, text="Reboot", fg_color=DANGER,
                      command=self.serial.pulse_reset).grid(row=0, column=2,
                                                            padx=4, sticky="ew")

        # Bench REAL-sensor scaled debug: NOT a sim — the real LiDAR stays live on
        # UART, the firmware just scales the AGL up so a close bench target sweeps
        # the full callout band and logs each raw reading. Its own row so it reads
        # as the distinct mode it is.
        ctk.CTkButton(card, text="Bench real sensor (scaled debug)",
                      command=self._enter_bench_real).grid(
                          row=3, column=0, columnspan=3, padx=16, pady=(0, 8),
                          sticky="ew")

        ctk.CTkLabel(card, text="Enter Sim: the board reboots (RTS) and the bench "
                     "rides the reconnect to catch its sim window — watch for the "
                     "green dot (tap the physical EN button if RTS reset doesn't "
                     "work on your board). Enter Config: must be in sim FIRST — it "
                     "software-reboots the box into the config menu (then drive it "
                     "with Next/Confirm below). Bench real sensor: keep the LiDAR "
                     "connected — readings get scaled x80 (≈1 ft → 0, 6 ft → 400 ft) "
                     "and logged so you can confirm the sensor is talking on the bench.",
                     text_color=MUTED, font=ctk.CTkFont(size=11), wraplength=520,
                     justify="left").grid(row=4, column=0, columnspan=3, padx=16,
                                          pady=(0, 14), sticky="w")

    def _build_ils_card(self, parent):
        card = self._card(parent, "ILS approach")

        self.gs_entry  = self._labelled_entry(card, 1, "Glideslope (deg)", "3.0")
        self.spd_entry = self._labelled_entry(card, 2, "Groundspeed (kt)", "70")
        self.alt_entry = self._labelled_entry(card, 3, "Start AGL (ft)", "400")

        # Hand-flying error: 0 % = perfect on-rails glideslope, up to 25 % = a sloppy
        # hand-flown approach (wander + over-corrections + partial level-offs) so the
        # firmware sees a realistic, jittery descent instead of a ruler-straight one.
        self.ils_err_lbl = ctk.CTkLabel(card, text="Hand-flying error: 0 %")
        self.ils_err_lbl.grid(row=4, column=0, columnspan=3, padx=16, pady=(10, 0),
                              sticky="w")
        self.ils_err_slider = ctk.CTkSlider(card, from_=0, to=25,
                                            command=self._on_ils_error)
        self.ils_err_slider.set(0)              # default: perfect glideslope
        self.ils_err_slider.grid(row=5, column=0, columnspan=3, padx=16, pady=4,
                                 sticky="ew")

        row = ctk.CTkFrame(card, fg_color="transparent")
        row.grid(row=6, column=0, columnspan=3, padx=12, pady=(8, 14), sticky="ew")
        for i in range(3):
            row.grid_columnconfigure(i, weight=1)
        self.fly_btn = ctk.CTkButton(row, text="Fly approach", fg_color=GOOD,
                                     text_color="#000", command=self._fly_ils)
        self.fly_btn.grid(row=0, column=0, padx=4, sticky="ew")
        ctk.CTkButton(row, text="Pause", fg_color="transparent", border_width=1,
                      command=self._pause_ils).grid(row=0, column=1, padx=4,
                                                   sticky="ew")
        ctk.CTkButton(row, text="Reset", fg_color="transparent", border_width=1,
                      command=self._reset_ils).grid(row=0, column=2, padx=4,
                                                    sticky="ew")

    def _build_noise_card(self, parent):
        card = self._card(parent, "Sensor model")

        self.offset_entry = self._labelled_entry(
            card, 1, "Ground/mount offset (ft)", "3.0")
        ctk.CTkLabel(card, text="(streamed at AGL 0 so the box LEARNS this as "
                     "ground during its boot fill)", text_color=MUTED,
                     font=ctk.CTkFont(size=11), wraplength=520, justify="left"
                     ).grid(row=2, column=0, columnspan=3, padx=16, sticky="w")

        self.range_entry = self._labelled_entry(
            card, 3, "Sensor max range (ft)", "328")
        ctk.CTkLabel(card, text="(SF30/C ~328 ft / 100 m. Above this the bench "
                     "sends lost-signal — like the real sensor losing the return "
                     "— so the box holds last-good until you descend back in range)",
                     text_color=MUTED, font=ctk.CTkFont(size=11), wraplength=520,
                     justify="left").grid(row=4, column=0, columnspan=3, padx=16,
                                          sticky="w")

        self.sigma_lbl = ctk.CTkLabel(card, text="Jitter σ: 0.2 ft")
        self.sigma_lbl.grid(row=5, column=0, columnspan=3, padx=16, pady=(10, 0),
                            sticky="w")
        self.sigma_slider = ctk.CTkSlider(card, from_=0, to=5, command=self._on_sigma)
        self.sigma_slider.set(0.2)              # realistic default (matches engine)
        self.sigma_slider.grid(row=6, column=0, columnspan=3, padx=16, pady=4,
                               sticky="ew")

        self.drop_lbl = ctk.CTkLabel(card, text="Lost-signal dropouts: 1 %")
        self.drop_lbl.grid(row=7, column=0, columnspan=3, padx=16, pady=(10, 0),
                           sticky="w")
        self.drop_slider = ctk.CTkSlider(card, from_=0, to=30, command=self._on_drop)
        self.drop_slider.set(1)                 # realistic default (matches engine)
        self.drop_slider.grid(row=8, column=0, columnspan=3, padx=16,
                              pady=(4, 4), sticky="ew")

        # Live stream-rate override. The engine re-reads rate_hz every tick, so
        # dragging this re-paces the frame output immediately (no restart) — handy
        # for testing whether the raw DATA RATE itself influences any audio artifact.
        # 78 Hz is the real sensor's active rate; drop it low or push it high to A/B.
        self.rate_lbl = ctk.CTkLabel(card, text="Stream rate: %d Hz (real sensor)"
                                     % P.DEFAULT_STREAM_HZ)
        self.rate_lbl.grid(row=9, column=0, columnspan=3, padx=16, pady=(10, 0),
                           sticky="w")
        self.rate_slider = ctk.CTkSlider(card, from_=5, to=200, command=self._on_rate)
        self.rate_slider.set(P.DEFAULT_STREAM_HZ)
        self.rate_slider.grid(row=10, column=0, columnspan=3, padx=16,
                              pady=(4, 14), sticky="ew")

    def _build_menu_card(self, parent):
        card = self._card(parent, "Config menu (remote)")
        ctk.CTkLabel(card, text="After Enter Config: tap Next to cycle, Confirm "
                     "to accept each level (audio mode → start-alt → volume).",
                     text_color=MUTED, font=ctk.CTkFont(size=11), wraplength=520,
                     justify="left").grid(row=1, column=0, columnspan=3, padx=16,
                                          sticky="w")
        row = ctk.CTkFrame(card, fg_color="transparent")
        row.grid(row=2, column=0, columnspan=3, padx=12, pady=(8, 14), sticky="ew")
        row.grid_columnconfigure(0, weight=1)
        row.grid_columnconfigure(1, weight=1)
        ctk.CTkButton(row, text="Next (tap)", command=self.ctrl.menu_next).grid(
            row=0, column=0, padx=4, sticky="ew")
        ctk.CTkButton(row, text="Confirm (double-tap)", command=self.ctrl.menu_confirm
                      ).grid(row=0, column=1, padx=4, sticky="ew")

    def _build_log(self):
        frame = ctk.CTkFrame(self, corner_radius=14)
        frame.grid(row=2, column=0, columnspan=2, padx=14, pady=(7, 14),
                   sticky="ew")
        frame.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(frame, text="DEVICE LOG", text_color=MUTED,
                     font=ctk.CTkFont(size=11, weight="bold")).grid(
            row=0, column=0, padx=14, pady=(8, 0), sticky="w")
        self.log = ctk.CTkTextbox(frame, height=150, font=ctk.CTkFont(
            family="Consolas", size=12))
        self.log.grid(row=1, column=0, padx=14, pady=(2, 12), sticky="ew")

    def _labelled_entry(self, card, row, label, default):
        ctk.CTkLabel(card, text=label).grid(row=row, column=0, padx=16, pady=4,
                                            sticky="w")
        e = ctk.CTkEntry(card, width=90)
        e.insert(0, default)
        e.grid(row=row, column=2, padx=16, pady=4, sticky="e")
        return e

    # =========================================================================
    #  Actions
    # =========================================================================

    def _refresh_ports(self):
        ports = list_ports()
        self._port_devices = []
        labels = []
        esp_idx = None
        for i, p in enumerate(ports):
            self._port_devices.append(p["device"])
            vidpid = ("  [%04X:%04X]" % (p["vid"], p["pid"] or 0)
                      if p["vid"] is not None else "")
            tag = "   * ESP32" if p["is_esp"] else ""
            labels.append("%s  -  %s%s%s" % (p["device"], p["desc"], vidpid, tag))
            if p["is_esp"] and esp_idx is None:
                esp_idx = i
        if not labels:
            labels = ["(none found)"]
            self._port_devices = []
        self.port_menu.configure(values=labels)
        # Pre-select the Espressif port so you can't accidentally talk to some
        # other USB-serial gadget (a Steam controller also shows as "USB Serial").
        self.port_menu.set(labels[esp_idx] if esp_idx is not None else labels[0])

    def _selected_port(self):
        idx = 0
        try:
            idx = self.port_menu.cget("values").index(self.port_menu.get())
        except Exception:
            pass
        if 0 <= idx < len(getattr(self, "_port_devices", [])):
            return self._port_devices[idx]
        return None

    def _toggle_connect(self):
        # is_active() reflects user intent; the actual open happens asynchronously
        # in the worker (which also auto-reopens across the chip's resets).
        if self.serial.is_active():
            self.engine.stop()
            self.ctrl.stop_attach()
            self.serial.disconnect()
            self.connect_btn.configure(text="Connect")
            self._set_status("disconnected", DANGER)
            self.sim_active = False
            return
        port = self._selected_port()
        if not port:
            self._append_log("No port selected.")
            return
        if self.serial.connect(port):
            self.connect_btn.configure(text="Disconnect")
            # Stream immediately so the device's boot ground-fill is fed, and
            # spam HELLO so the next boot window is caught.
            self.engine.start()
            self.ctrl.start_attach(P.OP_HELLO)
            self._set_status("attaching…", WARN)

    def _enter_sim(self):
        # Hardware-reset the chip (works whether or not it's already in sim),
        # then spam HELLO so the post-reboot sim window is caught as the port
        # re-enumerates and the worker auto-reconnects.
        self.serial.pulse_reset()
        self.ctrl.start_attach(P.OP_HELLO)
        self._set_status("attaching…", WARN)

    def _enter_bench_real(self):
        # Real-sensor scaled debug: reboot the chip and spam OP_BENCH_REAL so the
        # post-reboot attach window catches it. The firmware keeps the LiDAR on
        # UART (no sim frames needed), so we DON'T start the sim engine here — the
        # board reports its own real, scaled readings on the log instead.
        self.serial.pulse_reset()
        self.ctrl.start_attach(P.OP_BENCH_REAL)
        self._set_status("attaching (bench real sensor)…", WARN)

    def _enter_config(self):
        # While in sim the chip's RTS reset is suppressed, so trigger a SOFTWARE
        # reboot into the config menu (OP_ENTER_CONFIG → device sets a reboot-to-
        # config flag and restarts), then re-attach with HELLO so it comes back
        # into sim and the menu runs. Enter Sim FIRST if you're not attached yet.
        self.ctrl.enter_config()
        self.ctrl.start_attach(P.OP_HELLO)
        self._set_status("rebooting to config…", WARN)

    def _on_slider(self, value):
        if self._slider_guard:
            return
        # Hand flying: take over from any running ILS.
        self.model.mode = "manual"
        self.model.running = False
        self.model.set_manual(float(value))

    def _reset_ground(self):
        self.model.mode = "manual"
        self.model.running = False
        self.model.set_manual(0.0)
        self._set_slider(0.0)

    def _fly_ils(self):
        self.model.glideslope_deg = self._read_float(self.gs_entry, 3.0)
        self.model.groundspeed_kt = self._read_float(self.spd_entry, 70.0)
        self.model.start_alt_ft = self._read_float(self.alt_entry, 400.0)
        self.model.set_error_rate(self.ils_err_slider.get() / 100.0)
        self.model.arm_ils()
        self.model.play_ils()

    def _pause_ils(self):
        self.model.pause_ils()

    def _reset_ils(self):
        self.model.start_alt_ft = self._read_float(self.alt_entry, 400.0)
        self.model.glideslope_deg = self._read_float(self.gs_entry, 3.0)
        self.model.set_error_rate(self.ils_err_slider.get() / 100.0)
        self.model.arm_ils()
        self._set_slider(self.model.current_agl())

    def _on_ils_error(self, value):
        # Live: a real pilot's sloppiness as a 0..25 % deviation from the glideslope.
        # The model re-reads error_rate every tick, so dragging this re-flies the
        # current approach dirtier/cleaner immediately (no restart).
        pct = float(value)
        self.ils_err_lbl.configure(text="Hand-flying error: %d %%" % int(round(pct)))
        self.model.set_error_rate(pct / 100.0)

    def _on_sigma(self, value):
        self.engine.sigma_ft = float(value)
        self.sigma_lbl.configure(text="Jitter σ: %.1f ft" % value)

    def _on_drop(self, value):
        self.engine.dropout_prob = float(value) / 100.0
        self.drop_lbl.configure(text="Lost-signal dropouts: %d %%" % int(value))

    def _on_rate(self, value):
        # Live re-pace: the stream thread reads engine.rate_hz on its next tick.
        hz = int(value)
        self.engine.rate_hz = hz
        tag = " (real sensor)" if hz == P.DEFAULT_STREAM_HZ else ""
        self.rate_lbl.configure(text="Stream rate: %d Hz%s" % (hz, tag))

    # =========================================================================
    #  Periodic refresh + serial callbacks
    # =========================================================================

    def _tick(self):
        # Keep the ground offset + sensor max range live from their entries.
        self.engine.ground_offset_ft = self._read_float(self.offset_entry, 3.0)
        self.engine.max_range_ft = self._read_float(self.range_entry, 328.0)

        # Drain RX log lines (thread -> UI handoff via the queue).
        drained = 0
        while drained < 200:
            try:
                line = self._rx_q.get_nowait()
            except queue.Empty:
                break
            self._append_log(line)
            if "BENCH SIM MODE" in line:
                # Sim (re)entered. We deliberately KEEP the HELLO heartbeat
                # running so any later reboot (e.g. after a config commit) is
                # caught straight back into sim with no user action.
                self.sim_active = True
                self._set_status("SIM ACTIVE", GOOD)
                self.sim_state_lbl.configure(text="sim mode active",
                                             text_color=GOOD)
            elif "BENCH REAL-SENSOR MODE" in line:
                # Real-sensor scaled debug armed: the board is reading its own
                # LiDAR, so there's no sim engine to track — just reflect the mode
                # and stop the attach spam (the window has been caught).
                self.sim_active = False
                self.ctrl.stop_attach()
                self._set_status("BENCH REAL SENSOR", GOOD)
                self.sim_state_lbl.configure(text="real sensor — scaled debug",
                                             text_color=GOOD)
            elif "config committed" in line:
                self.sim_active = False
                self._set_status("config saved — re-entering sim…", WARN)
            elif "waiting for" in line or "connection lost" in line:
                if self.sim_active:
                    self.sim_active = False
                    self._set_status("re-attaching…", WARN)
            drained += 1

        # Update the AGL read-out; reflect a running ILS on the tape.
        agl = self.engine.last_agl_ft
        self.agl_value.configure(text="%d" % int(round(agl)))
        if self.model.is_ils_running():
            self._set_slider(agl)

        self.after(100, self._tick)

    def _on_rx(self, text):
        # Called from the serial reader thread -> just enqueue for the UI thread.
        self._rx_q.put(text)

    def _on_status(self, msg):
        self._rx_q.put("[serial] " + msg)

    # =========================================================================
    #  Helpers
    # =========================================================================

    def _set_slider(self, ft):
        self._slider_guard = True
        try:
            self.alt_slider.set(max(0.0, min(ALT_MAX_FT, ft)))
        finally:
            self._slider_guard = False

    def _set_status(self, text, color):
        self.status_lbl.configure(text=text)
        self.status_dot.configure(text_color=color)

    def _append_log(self, text):
        self.log.insert("end", text + "\n")
        # Cap the buffer so a long session doesn't grow unbounded.
        if int(self.log.index("end-1c").split(".")[0]) > 500:
            self.log.delete("1.0", "100.0")
        self.log.see("end")

    def _read_float(self, entry, default):
        try:
            return float(entry.get())
        except (ValueError, TypeError):
            return default

    def _on_close(self):
        try:
            self.engine.stop()
            self.ctrl.stop_attach()
            self.serial.disconnect()
        finally:
            self.destroy()


def run():
    BenchApp().mainloop()
