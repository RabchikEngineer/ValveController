#!/usr/bin/env bash
set -euo pipefail

BAUD="${2:-460800}"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR"
BUILD_DIR="$PROJECT_ROOT/.pio/build/tenstar_robot_esp32c3"

BIN="$BUILD_DIR/firmware.bin"
PARTITIONS="$BUILD_DIR/partitions.bin"

find_first() {
  local f
  for f in "$@"; do
    if [[ -f "$f" ]]; then
      printf '%s\n' "$f"
      return 0
    fi
  done
  return 1
}

find_port() {
  local ports=()

  if [[ $# -ge 1 && -n "${1:-}" ]]; then
    printf '%s\n' "$1"
    return 0
  fi

  for p in /dev/ttyACM* /dev/ttyUSB*; do
    [[ -e "$p" ]] && ports+=("$p")
  done

  if [[ ${#ports[@]} -eq 0 ]]; then
    echo "No serial port found. Checked /dev/ttyACM* and /dev/ttyUSB*" >&2
    exit 1
  fi

  if [[ ${#ports[@]} -gt 1 ]]; then
    echo "Multiple serial ports found: ${ports[*]}" >&2
    echo "Using first one: ${ports[0]}" >&2
  fi

  printf '%s\n' "${ports[0]}"
}

PORT="$(find_port "${1:-}")"

if command -v esptool >/dev/null 2>&1; then
  ESPTOOL=(esptool)
elif command -v esptool.py >/dev/null 2>&1; then
  ESPTOOL=(esptool.py)
else
  ESPTOOL=(python3 -m esptool)
fi

BOOTLOADER="$(
  find_first \
    "$BUILD_DIR/bootloader.bin" \
    "$HOME/.platformio/packages/framework-arduinoespressif32/tools/sdk/esp32c3/bin/bootloader_dio_40m.bin" \
    "$HOME/.platformio/packages/framework-arduinoespressif32/tools/sdk/esp32c3/bin/bootloader_qio_80m.bin" \
    "$HOME/.platformio/packages/framework-arduinoespressif32/tools/sdk/bin/bootloader_dio_40m.bin" \
    "$HOME/.platformio/packages/framework-arduinoespressif32/tools/sdk/bin/bootloader_qio_80m.bin" \
  || true
)"

BOOT_APP0="$(
  find_first \
    "$BUILD_DIR/boot_app0.bin" \
    "$HOME/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin" \
  || true
)"

echo "Using port: $PORT"

if [[ -n "$BOOTLOADER" && -f "$PARTITIONS" && -n "$BOOT_APP0" ]]; then
  echo "Full flash:"
  echo "  bootloader : $BOOTLOADER"
  echo "  partitions : $PARTITIONS"
  echo "  boot_app0  : $BOOT_APP0"
  echo "  firmware   : $BIN"

  "${ESPTOOL[@]}" \
    --chip esp32c3 \
    --port "$PORT" \
    --baud "$BAUD" \
    --before default-reset \
    --after hard-reset \
    write-flash -z \
    --flash-mode dio \
    --flash-freq 40m \
    --flash-size detect \
    0x0 "$BOOTLOADER" \
    0x8000 "$PARTITIONS" \
    0xe000 "$BOOT_APP0" \
    0x10000 "$BIN"
else
  echo "Warning: bootloader/partitions/boot_app0 not fully found." >&2
  echo "Flashing app only at 0x10000: $BIN" >&2

  "${ESPTOOL[@]}" \
    --chip esp32c3 \
    --port "$PORT" \
    --baud "$BAUD" \
    --before default-reset \
    --after hard-reset \
    write-flash -z \
    --flash-mode dio \
    --flash-freq 40m \
    --flash-size detect \
    0x10000 "$BIN"
fi
