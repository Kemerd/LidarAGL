# =============================================================================
#  LidarAGL — top-level ESP-IDF project file
# -----------------------------------------------------------------------------
#  Advisory AGL height-callout + flare-tone box for an experimental aircraft.
#  Target: ESP32-S3 + LightWare SF30/C (default) or SF30/D + PCM5102A I2S DAC.
#
#  Build with the standard ESP-IDF flow (the BUILDER runs these, not the repo):
#      idf.py set-target esp32s3
#      idf.py build
#
#  The pure-logic modules also build on the host for unit testing — see test/.
# =============================================================================

cmake_minimum_required(VERSION 3.16)

# Pull in the ESP-IDF build system. IDF_PATH must point at a v5.x install.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# A custom partition table gives us a dedicated NVS region for the boot buffer
# (the rolling ground-reference readings) alongside the application image.
set(PARTITION_TABLE_CSV "${CMAKE_CURRENT_LIST_DIR}/partitions.csv")

project(LidarAGL)
