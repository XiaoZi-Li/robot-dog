#!/usr/bin/env bash
set -e

OUT_FILE="${1:-/tmp/usb_mic_test.wav}"
DURATION="${2:-5}"
DEVICE="${3:-plughw:0,0}"

echo "===== USB MIC RECORD START ====="
echo "OUT_FILE=$OUT_FILE"
echo "DURATION=$DURATION"
echo "DEVICE=$DEVICE"

rm -f "$OUT_FILE"

arecord \
  -D "$DEVICE" \
  -d "$DURATION" \
  -f S16_LE \
  -r 16000 \
  -c 1 \
  -t wav \
  "$OUT_FILE"

echo
echo "===== RECORD DONE ====="
ls -lh "$OUT_FILE"
file "$OUT_FILE" || true
