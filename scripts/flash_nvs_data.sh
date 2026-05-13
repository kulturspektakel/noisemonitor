#!/bin/bash
# Generates an NVS partition binary from ../nvs_data.csv and flashes it to the
# `nvs` partition on a connected ESP32-S3. Reads WiFi credentials, salt, etc.
#
# Run with IDF_PATH set (use VS Code's "ESP-IDF: Open ESP-IDF Terminal", or
# source the ESP-IDF env in your shell). Override the serial port via
# ESPPORT=/dev/tty.usbmodemXXXX if needed.
#
# This is reversible: re-running with new values overwrites the NVS partition.

set -e

if [ -z "$IDF_PATH" ]; then
    echo "ERROR: IDF_PATH is not set. Open an ESP-IDF terminal first." >&2
    exit 1
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
PORT="${ESPPORT:-/dev/tty.usbmodem201301}"
TEMP_DIR=$(mktemp -d)
NVS_BIN="$TEMP_DIR/nvs_data.bin"

echo "Generating NVS binary from $DIR/../nvs_data.csv ..."
"$IDF_PATH/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py" \
    generate --version 2 "$DIR/../nvs_data.csv" "$NVS_BIN" 0x3000

echo "Flashing NVS partition via $PORT ..."
"$IDF_PATH/components/partition_table/parttool.py" \
    --port "$PORT" \
    write_partition --partition-name=nvs --input "$NVS_BIN"

rm "$NVS_BIN"
echo "NVS partition flashed. Reboot the device to apply."
