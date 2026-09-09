# A-3 `move_delta` six-direction stage

This stage replaces the passed one-shot `test_move_up` smoke command with the
frozen v0 message shape:

```json
{
  "v": 0,
  "type": "move_delta",
  "ts": 0,
  "seq": 2,
  "payload": {"axis": "x", "dir": 1, "step": "coarse"}
}
```

Implemented now:

- base-frame `x`, `y`, and `z` translation
- `dir` equal to `1` or `-1`
- `coarse` = 2 cm and `fine` = 0.5 cm
- distance-scaled trajectory: coarse = 90 steps and fine = 23 steps by
  default, followed by 5 stable samples before the final ack
- measured-position tolerance: coarse = 5 mm and fine = 1.5 mm
- gripper `open` = 100 degrees and `close` = 0 degrees
- final `ack` with `ok`, `clamped`, or `rejected`
- `state` at 5 Hz with measured end-effector position
- stale-sequence dropping, single-command busy rejection, workspace clamp, IK
  checks, and JSONL evidence
- same-connection `estop`, freeze confirmation, command blocking, and
  `estop_reset`
- automatic Estop when an executing command fails or its owning control
  connection is lost

The Server-to-Isaac-Sim paths were demonstrated first with a Mac surrogate
client. Thomas subsequently completed the phone-App integration through the
Mac relay: hello/test, six directions, gripper, acknowledgements, and JSONL
evidence passed on 2026-09-07. The I-4 joint ticket is complete and A-3 is ready
to move from review to complete. Follow-up protocol questions are tracked for
v0.1 rather than blocking this stage.

## Files

- `a3_transport.py`: protocol validation, WebSocket server, state and logging
- `a3_sim_move_delta.py`: Isaac Sim SO-101 Differential IK execution
- `mac_move_delta.py`: sends one command and checks measured displacement
- `mac_gripper.py`: sends one open/close command and checks final state
- `mac_estop.py`: interrupts an in-flight move, verifies freeze and blocking,
  then resets the emergency stop
- `mac_disconnect_estop.py`: drops an in-flight control connection and verifies
  automatic Estop, blocking, and reset after reconnecting
- `mac_phone_relay.py`: accepts Thomas's App on the Mac and transparently
  forwards messages through the Mac-owned SSH tunnel
- `test_move_delta.py`: local protocol tests with a fake simulation worker
- `test_mac_phone_relay.py`: local bidirectional relay integration check

## Upload from Mac

Stop the older one-shot simulation with `Ctrl-C`, but keep the Docker container
running. In a Mac Terminal:

```bash
cd /Users/lichengyan/Documents/ChatGPT/碩士論文/output
scp -r a3_move_delta leo@140.115.54.63:~/
```

Then enter the Server host and copy only the runtime files into the container:

```bash
ssh leo@140.115.54.63
docker exec isaac-sim mkdir -p /root/Documents/robotics/work/so101_shared_autonomy/scripts/a3_move_delta
docker cp ~/a3_move_delta/a3_transport.py isaac-sim:/root/Documents/robotics/work/so101_shared_autonomy/scripts/a3_move_delta/
docker cp ~/a3_move_delta/a3_sim_move_delta.py isaac-sim:/root/Documents/robotics/work/so101_shared_autonomy/scripts/a3_move_delta/
docker exec -it isaac-sim bash
```

## Start the Server/container simulation

Run inside the container at the `root@RTXWS` prompt:

```bash
cd /root/Documents/robotics/upstream/leisaac/dependencies/IsaacLab

PYTHONPATH="/root/Documents/robotics/upstream/leisaac/dependencies/IsaacLab/source/isaaclab:/root/Documents/robotics/upstream/leisaac/dependencies/IsaacLab/source/isaaclab_assets:/root/Documents/robotics/upstream/leisaac/dependencies/IsaacLab/source/isaaclab_tasks:/root/Documents/robotics/upstream/leisaac/source/leisaac${PYTHONPATH:+:$PYTHONPATH}" \
CUDA_VISIBLE_DEVICES=0 \
PUBLIC_IP=140.115.54.63 \
LIVESTREAM=1 \
ENABLE_CAMERAS=1 \
PYTHONUNBUFFERED=1 \
./isaaclab.sh -p \
/root/Documents/robotics/work/so101_shared_autonomy/scripts/a3_move_delta/a3_sim_move_delta.py \
--livestream 1 --enable_cameras --ws-host 127.0.0.1 --ws-port 8766
```

Readiness evidence:

```text
A3 MOVE_DELTA SIM READY
A3 WS READY ws://127.0.0.1:8766
Accepted: move_delta axis=x/y/z, dir=+/-1, step=coarse/fine
Motion profile: coarse=90 trajectory steps, fine=23 trajectory steps, stable=5, hold_limit=90
```

## Test from the Mac

Keep the existing SSH tunnel running. If it was closed, start it in one Mac
Terminal and leave that Terminal open:

```bash
ssh -N -o ExitOnForwardFailure=yes -L 127.0.0.1:8766:127.0.0.1:8766 leo@140.115.54.63
```

Use another Mac Terminal to send one command at a time:

```bash
cd /Users/lichengyan/Documents/ChatGPT/碩士論文/output/a3_move_delta

../a3_mac_sim/.venv/bin/python mac_move_delta.py --axis x --dir 1 --step coarse
../a3_mac_sim/.venv/bin/python mac_move_delta.py --axis x --dir -1 --step coarse
../a3_mac_sim/.venv/bin/python mac_move_delta.py --axis y --dir 1 --step coarse
../a3_mac_sim/.venv/bin/python mac_move_delta.py --axis y --dir -1 --step coarse
../a3_mac_sim/.venv/bin/python mac_move_delta.py --axis z --dir 1 --step coarse
../a3_mac_sim/.venv/bin/python mac_move_delta.py --axis z --dir -1 --step coarse
```

Run the next line only after the prior line prints `PASS`. Example:

```text
[ACK] status=ok reason=completed
A3 MOVE_DELTA X PLUS COARSE PASS
```

The Server-side evidence is written to:

```text
/root/Documents/robotics/data/network_logs/a3_move_delta.jsonl
```

## Test the gripper

After uploading the updated transport and simulator files and restarting the
simulation, run one action at a time from the Mac:

```bash
../a3_mac_sim/.venv/bin/python mac_gripper.py --action open
../a3_mac_sim/.venv/bin/python mac_gripper.py --action close
```

Expected client evidence:

```text
A3 GRIPPER OPEN PASS
A3 GRIPPER CLOSE PASS
```

The Server output additionally records measured start, target, and end angles.

## Test emergency stop and reset

This test uses one WebSocket connection. It starts a coarse Z+ move, waits until
at least 2 mm of motion is measured, sends `estop`, verifies the original move
is rejected, verifies less than 2 mm of post-freeze drift, confirms another
movement is blocked, and finally sends `estop_reset`.

```bash
../a3_mac_sim/.venv/bin/python mac_estop.py
```

Expected client evidence:

```text
A3 ESTOP SAME-CONNECTION INTERRUPT PASS
```

After uploading the disconnect-safe `a3_transport.py` and restarting the
simulation, verify loss of the active control connection:

```bash
../a3_mac_sim/.venv/bin/python mac_disconnect_estop.py
```

Expected evidence:

```text
A3 DISCONNECT ESTOP PASS
```

## Connect Thomas's App through the Mac relay

The App does not log in to RTXWS and does not know the RTXWS password. Keep
these three processes running:

1. Isaac Sim and the A-3 WebSocket server in the RTXWS container.
2. The SSH tunnel in one Mac Terminal:

   ```bash
   ssh -N -o ExitOnForwardFailure=yes -L 127.0.0.1:8766:127.0.0.1:8766 leo@140.115.54.63
   ```

3. The phone-facing relay in a second Mac Terminal:

   ```bash
   cd /Users/lichengyan/Documents/ChatGPT/碩士論文/output/a3_move_delta
   ../a3_mac_sim/.venv/bin/python mac_phone_relay.py
   ```

The relay prints the URL to enter in Thomas's App, normally
`ws://<MAC_LAN_IP>:8765`. The phone and Mac must be on a network that allows
them to reach each other. The relay forwards JSON text unchanged in both
directions and closes both sides if either connection drops; it never retries
or replays a command.

For a complete cold-start sequence, acceptance tests, evidence checklist, and
shutdown procedure, see `A3_冷啟動與_Thomas_App_整合操作手冊.md`.
