#!/usr/bin/env bash
set -euo pipefail

isaaclab_root=/root/Documents/robotics/upstream/leisaac/dependencies/IsaacLab
controller=/root/Documents/robotics/work/so101_shared_autonomy/scripts/s4_pick_place/s4_sim_pick_place.py

cd "$isaaclab_root"
export PYTHONPATH="$isaaclab_root/source/isaaclab:$isaaclab_root/source/isaaclab_assets:$isaaclab_root/source/isaaclab_tasks:/root/Documents/robotics/upstream/leisaac/source/leisaac${PYTHONPATH:+:$PYTHONPATH}"

exec env \
  CUDA_VISIBLE_DEVICES=0 \
  LEISAAC_ASSETS_ROOT=/root/Documents/robotics/upstream/leisaac/assets \
  PUBLIC_IP=140.115.54.63 \
  LIVESTREAM=1 \
  ENABLE_CAMERAS=1 \
  PYTHONUNBUFFERED=1 \
  ./isaaclab.sh -p "$controller" \
  --livestream 1 \
  --enable_cameras \
  --ws-host 127.0.0.1 \
  --ws-port 8766 \
  --operator-multiview
