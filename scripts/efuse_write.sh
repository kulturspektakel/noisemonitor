#!/bin/bash
# Burns the device-name string into ESP32-S3 eFuse BLOCK3 (the custom DEVICE_ID
# field defined in main/esp_efuse_custom_table.csv).
#
# *** IRREVERSIBLE *** — eFuses can only transition bits from 0 → 1 within a
# block, and only once per block in practice. Triple-check the device name
# before confirming.
#
# Run with IDF_PATH set (VS Code "ESP-IDF: Open ESP-IDF Terminal" is easiest).
# Override port via ESPPORT=/dev/tty.usbmodemXXXX.

set -e

if [ -z "$IDF_PATH" ]; then
    echo "ERROR: IDF_PATH is not set. Open an ESP-IDF terminal first." >&2
    exit 1
fi

PORT="${ESPPORT:-/dev/tty.usbmodem101}"

echo "Enter device name (1-16 chars, [A-Za-z0-9äßöü-]):"
read -r DEVICE_NAME

REGEX='^[A-Za-z0-9äßöü-]+$'
if ! [[ $DEVICE_NAME =~ $REGEX ]]; then
    echo "Error: device name contains invalid characters: $DEVICE_NAME" >&2
    exit 1
fi

BYTE_COUNT=$(echo -n "$DEVICE_NAME" | wc -c)
if [ "$BYTE_COUNT" -gt 16 ]; then
    echo "Error: device name is longer than 16 bytes." >&2
    exit 1
fi

echo "About to burn device name '$DEVICE_NAME' to eFuse BLOCK3 on $PORT."
echo "This is PERMANENT. Type 'yes' to continue:"
read -r CONFIRM
if [ "$CONFIRM" != "yes" ]; then
    echo "Aborted."
    exit 1
fi

TEMP_DIR=$(mktemp -d)
echo -n "$DEVICE_NAME" | dd bs=32 conv=sync status=none of="$TEMP_DIR/device_id.bin"

# Locate espefuse. Newer ESP-IDF installs it as a package in the espressif
# Python env rather than as a file under components/esptool_py/esptool/.
if command -v espefuse.py >/dev/null 2>&1; then
    ESPEFUSE=(espefuse.py)
elif command -v espefuse >/dev/null 2>&1; then
    ESPEFUSE=(espefuse)
elif python -c "import espefuse" >/dev/null 2>&1; then
    ESPEFUSE=(python -m espefuse)
else
    echo "ERROR: espefuse not found. Activate the ESP-IDF environment first:" >&2
    echo "  . \"\$IDF_PATH/export.sh\"" >&2
    rm -rf "$TEMP_DIR"
    exit 1
fi

"${ESPEFUSE[@]}" \
    --port "$PORT" --chip esp32s3 \
    burn_block_data BLOCK3 "$TEMP_DIR/device_id.bin"

rm -rf "$TEMP_DIR"
echo "Done. Reboot the device to load the new device ID."
