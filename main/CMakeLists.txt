# =============================================================================
#  LidarAGL — main component build
# -----------------------------------------------------------------------------
#  Registers all firmware sources and embeds whatever voice-clip PCM files
#  currently exist in assets/clips/. The clip set is GLOBBED (not hard-listed)
#  so the project still builds when the builder has only generated some of the
#  clips — the firmware skips any clip whose embedded symbol is absent.
# =============================================================================

# -----------------------------------------------------------------------------
#  Discover the raw PCM voice clips to embed.
#
#  We glob rather than enumerate so a partial clip set never breaks the build.
#  CONFIGURE_DEPENDS makes CMake re-run the glob when files are added/removed,
#  so newly generated clips get picked up without a manual clean.
#
#  Caveat (ESP-IDF v6+): before the real configure, the build system parses each
#  component's CMakeLists.txt in CMake *script mode* (cmake -P) purely to harvest
#  its REQUIRES. CONFIGURE_DEPENDS is illegal in script mode and aborts that pass
#  ("CONFIGURE_DEPENDS is invalid for script and find package modes"). The
#  built-in CMAKE_SCRIPT_MODE_FILE variable is defined ONLY during that pass, so
#  we omit CONFIGURE_DEPENDS there and keep it for the normal project-mode
#  configure, where the auto-reglob is the part that actually matters.
# -----------------------------------------------------------------------------
if(CMAKE_SCRIPT_MODE_FILE)
    # Requirement-harvest pass: plain glob (CONFIGURE_DEPENDS not allowed here).
    file(GLOB CLIP_FILES "${CMAKE_CURRENT_LIST_DIR}/../assets/clips/*.pcm")
else()
    # Normal configure: keep CONFIGURE_DEPENDS so added/removed clips re-trigger
    # the glob without a manual clean.
    file(GLOB CLIP_FILES CONFIGURE_DEPENDS
         "${CMAKE_CURRENT_LIST_DIR}/../assets/clips/*.pcm")
endif()

idf_component_register(
    SRCS
        "app_main.c"
        "sensor_profile.c"
        "lwnx.c"
        "sf30c.c"
        "robust.c"
        "range_filter.c"
        "boot_buffer.c"
        "state_machine.c"
        "audio_math.c"
        "audio.c"
        "callouts.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        #  ESP-IDF v6 split the monolithic 'driver' component into one component
        #  per peripheral and now strictly enforces that each <driver/xxx.h> we
        #  include lists its OWNING component here (a bare "driver" no longer
        #  pulls them in transitively). The three peripherals main actually uses:
        esp_driver_i2s   # driver/i2s_std.h — PCM5102A I2S audio output
        esp_driver_gpio  # driver/gpio.h    — DAC mute / status / config pins
        esp_driver_uart  # driver/uart.h    — SF30 LiDAR serial link
        esp_driver_usb_serial_jtag  # driver/usb_serial_jtag.h — bench HIL side-door
        nvs_flash        # boot-buffer persistence
        esp_pm           # automatic light-sleep
        esp_timer        # microsecond timing for dt / poll cadence
    EMBED_FILES
        ${CLIP_FILES}

    #  WHOLE_ARCHIVE — force every object in libmain.a into the final link.
    #
    #  The clips are declared in callouts.c with __attribute__((weak)) so an
    #  absent clip degrades gracefully (NULL) instead of failing the link. But a
    #  *weak* undefined reference does NOT make the linker pull the matching
    #  member (the generated <clip>.pcm.S.obj) out of a static library. Without
    #  this flag every embedded clip is therefore left out of the image and the
    #  firmware boots completely SILENT. Whole-archiving main force-includes the
    #  embedded clip objects; the weak refs in callouts.c then bind to real data,
    #  while clips that truly weren't globbed still resolve to NULL. main is the
    #  application component, so force-linking all of its objects is free.
    WHOLE_ARCHIVE
)

# -----------------------------------------------------------------------------
#  DEMO_MODE — the show-floor pulley-rig build (see the DEMO_MODE section of
#  config.h for the full story). Opt in per build directory with:
#
#      idf.py -DDEMO_MODE=1 build
#
#  The -D lands in the CMake cache, so the choice is STICKY for that build dir:
#  later plain `idf.py build` runs stay demo until -DDEMO_MODE=0 (or fullclean)
#  switches back. Flight builds — where the cache variable is absent/0 — compile
#  every demo branch out, so non-demo units spend zero flash on the feature.
#
#  Skipped during the early requirement-harvest pass (cmake -P script mode):
#  COMPONENT_LIB does not exist yet there, and only the real configure matters.
# -----------------------------------------------------------------------------
if(NOT CMAKE_SCRIPT_MODE_FILE AND DEMO_MODE)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE DEMO_MODE=1)
    message(STATUS "LidarAGL: DEMO_MODE build — scaled-AGL show firmware")
elseif(NOT CMAKE_SCRIPT_MODE_FILE)
    #  State the mode POSITIVELY on every configure, so a stale sticky cache is
    #  visible in the build log — and note the firmware itself now embeds the
    #  discriminator too (FIRMWARE_VERSION gains a "-DEMO" suffix in demo
    #  images), so the boot banner alone identifies which image is on a unit.
    message(STATUS "LidarAGL: FLIGHT build (DEMO_MODE off)")
endif()

# -----------------------------------------------------------------------------
#  SLEEP_MODE_ENABLE — light-sleep master switch (see the matching section of
#  config.h). Take light-sleep out of the picture entirely with:
#
#      idf.py -DSLEEP_MODE_ENABLE=0 build
#
#  IMPORTANT: `idf.py -D<x>` sets a CMake *cache variable*, which is a different
#  namespace from a C preprocessor define — it does NOT reach the compiler on its
#  own. Without this forwarding block config.h's `#ifndef SLEEP_MODE_ENABLE`
#  fallback silently wins and the image ships with sleep ENABLED, exactly
#  contradicting the requested build. That is a genuinely dangerous failure mode
#  for a flight test whose whole purpose is to remove sleep as a variable, so we
#  forward it explicitly and announce the result on every configure.
#
#  Only 0 disables: the variable is compared against 0 rather than tested for
#  truthiness so an empty/absent cache entry keeps the safe default (enabled),
#  and the value is passed through verbatim to keep config.h the single source
#  of truth for what the macro means.
# -----------------------------------------------------------------------------
if(NOT CMAKE_SCRIPT_MODE_FILE)
    if(DEFINED SLEEP_MODE_ENABLE AND SLEEP_MODE_ENABLE EQUAL 0)
        target_compile_definitions(${COMPONENT_LIB} PRIVATE SLEEP_MODE_ENABLE=0)
        message(STATUS "LidarAGL: light-sleep DISABLED (SLEEP_MODE_ENABLE=0)")
    else()
        message(STATUS "LidarAGL: light-sleep enabled (default)")
    endif()
endif()
