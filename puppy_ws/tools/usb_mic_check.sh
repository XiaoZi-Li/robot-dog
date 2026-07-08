#!/usr/bin/env bash
set -e

echo "===== LSUSB ====="
lsusb

echo
echo "===== ALSA CARDS ====="
cat /proc/asound/cards || true

echo
echo "===== ARECORD DEVICES ====="
arecord -l || true

echo
echo "===== APLAY DEVICES ====="
aplay -l || true
