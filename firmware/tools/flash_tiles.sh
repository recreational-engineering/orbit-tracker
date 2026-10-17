#!/bin/sh
# Write the pilot view's high-res tiles (main/generated/tiles.bin) to the "tiles" partition.
# Only needed once, or after the tiles change: the app flash does not touch it.
cd "$(dirname "$0")/.." && python -m esptool --chip esp32p4 -p "${1:-/dev/cu.usbmodem5CF71961181}" -b 921600 \
    write-flash 0xC20000 main/generated/tiles.bin
