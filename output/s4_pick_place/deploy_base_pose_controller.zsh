#!/bin/zsh
set -euo pipefail

source_dir="$(cd "$(dirname "$0")" && pwd)"
remote_host="leo@140.115.54.63"
remote_stage="/home/leo/s4_base_pose_update"
container_target="/root/Documents/robotics/work/so101_shared_autonomy/scripts/s4_pick_place"
files=(
  a3_transport.py
  s4_task_logic.py
  s4_sim_pick_place.py
  start_s4_base_pose.sh
)

for file_name in "${files[@]}"; do
  if [[ ! -f "$source_dir/$file_name" ]]; then
    print -u2 "DEPLOY FAIL: missing $source_dir/$file_name"
    exit 1
  fi
done

tar -C "$source_dir" -czf - "${files[@]}" | ssh "$remote_host" '
set -e
remote_stage=/home/leo/s4_base_pose_update
container_target=/root/Documents/robotics/work/so101_shared_autonomy/scripts/s4_pick_place
mkdir -p "$remote_stage"
tar -xzf - -C "$remote_stage"
for file_name in a3_transport.py s4_task_logic.py s4_sim_pick_place.py start_s4_base_pose.sh; do
  docker cp "$remote_stage/$file_name" "isaac-sim:$container_target/$file_name"
done
docker exec isaac-sim \
  /root/Documents/robotics/upstream/leisaac/dependencies/IsaacLab/_isaac_sim/python.sh \
  -m py_compile \
  "$container_target/a3_transport.py" \
  "$container_target/s4_task_logic.py" \
  "$container_target/s4_sim_pick_place.py"
docker exec isaac-sim grep -n \
  "S4 BASE CONTROL READY" \
  "$container_target/s4_sim_pick_place.py"
echo "S4 BASE CONTROL DEPLOY PASS"
'
