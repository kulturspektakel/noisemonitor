# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

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
