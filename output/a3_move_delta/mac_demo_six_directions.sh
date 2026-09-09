#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PYTHON="$SCRIPT_DIR/../a3_mac_sim/.venv/bin/python"
CLIENT="$SCRIPT_DIR/mac_move_delta.py"
STEP="${STEP:-coarse}"
COUNTDOWN_SECONDS="${COUNTDOWN_SECONDS:-5}"
PAUSE_AFTER_SECONDS="${PAUSE_AFTER_SECONDS:-2}"

if [[ "$STEP" != "coarse" && "$STEP" != "fine" ]]; then
    echo "STEP must be coarse or fine"
    exit 2
fi

if [[ ! -x "$PYTHON" ]]; then
    echo "Python environment not found: $PYTHON"
    exit 2
fi

run_move() {
    local axis="$1"
    local direction="$2"
    local label="$3"

    echo
    echo "NEXT: $label (axis=$axis, dir=$direction, step=$STEP)"
    echo "Switch to WebRTC now; moving in $COUNTDOWN_SECONDS seconds..."
    sleep "$COUNTDOWN_SECONDS"
    "$PYTHON" "$CLIENT" --axis "$axis" --dir="$direction" --step "$STEP"
    sleep "$PAUSE_AFTER_SECONDS"
}

echo "A3 SIX-DIRECTION WEBRTC DEMO"
echo "Step mode: $STEP"
echo "The paired commands return the arm near its starting position."

run_move z 1  "Z+ / UP"
run_move z -1 "Z- / DOWN"
run_move x 1  "X+ / HORIZONTAL 1"
run_move x -1 "X- / HORIZONTAL 1 RETURN"
run_move y 1  "Y+ / HORIZONTAL 2"
run_move y -1 "Y- / HORIZONTAL 2 RETURN"

echo
echo "A3 SIX-DIRECTION WEBRTC DEMO PASS"
