# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# --- Pin ESP-IDF to v5.4.1 -----------------------------------------------------
# This firmware runs WiFi + BLE + two concurrent TLS (MQTT) sessions on PSRAM,
# right at the internal-RAM limit (see the PSRAM/TLS/BLE coexistence tuning).
# IDF 5.4.4's larger WiFi/NimBLE/mbedtls footprint exhausts internal RAM at boot:
# BLE advertising malloc-fails ("BLE_ERR_MEM_CAPACITY") and the MQTT client task
# can't be created ("Error create mqtt task"). Verified on-device 2026-07.
# Do NOT bump the IDF version without re-tuning the memory coexistence first.
if(DEFINED IDF_VERSION_MAJOR)
  if(NOT (IDF_VERSION_MAJOR EQUAL 5 AND IDF_VERSION_MINOR EQUAL 4 AND IDF_VERSION_PATCH EQUAL 1))
    message(FATAL_ERROR
      "noisemonitor is pinned to ESP-IDF v5.4.1 but found "
      "v${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}.${IDF_VERSION_PATCH}. "
      "Newer IDFs exhaust internal RAM and break BLE + MQTT/TLS coexistence. "
      "Re-tune memory before changing this pin.")
  endif()
endif()
# -------------------------------------------------------------------------------

# Derive the firmware version from git tags: use the most recent tag verbatim
# (e.g. "v0.1.0"), appending "-dirty" when the working tree has uncommitted
# changes. Keeps the string short for esp_app_desc_t.version's 32-byte cap.
# Falls back to "0.0.0-nogit" when there are no tags or git is unavailable, so
# the build never breaks.
execute_process(
  COMMAND git describe --tags --abbrev=0
  WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  OUTPUT_VARIABLE PROJECT_VER
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET)
if(NOT PROJECT_VER)
  set(PROJECT_VER "0.0.0-nogit")
else()
  execute_process(
    COMMAND git status --porcelain
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    OUTPUT_VARIABLE GIT_DIRTY
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
  if(GIT_DIRTY)
    set(PROJECT_VER "${PROJECT_VER}-dirty")
  endif()
endif()

# ESP-IDF v5.4.1's bundled NimBLE (ble_gap.c, ble_store_config.c) triggers false
# -Warray-bounds under the GCC 14.2 toolchain. It's upstream code, not ours, so
# downgrade that one warning from an error to let the build link.
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=array-bounds" APPEND)

project(noisemonitor)
