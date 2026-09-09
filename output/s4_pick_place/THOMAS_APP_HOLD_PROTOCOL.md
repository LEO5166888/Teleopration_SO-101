# Thomas App press-and-hold control protocol

Updated: 2026-09-09

This is the current phone control contract for Leo's SO-101 S4 simulator. The
Mac relay is transparent: it forwards text WebSocket messages without changing
or retrying them.

## Connection

- App endpoint: `ws://<LEO_MAC_LAN_IP>:8765`
- Leo's Mac and the phone must be on a network that allows direct LAN traffic.
- The Mac LAN IP may change after reconnecting to Wi-Fi. Do not permanently
  hard-code `192.168.50.13` without first checking it.
- Only one phone App connection is accepted by the Mac relay.
- Do not run the Mac keyboard controller while Thomas's App is controlling.

Every text message uses this envelope:

```json
{
  "v": 0,
  "type": "hello",
  "ts": 1788940000000,
  "seq": 1,
  "payload": {}
}
```

`ts` is Unix epoch time in integer milliseconds. `seq` must be a non-negative,
strictly increasing integer for every outbound message on one connection,
including keepalives and stop messages. Never reuse a sequence number after
reconnect within the same socket. Payload is always a JSON object.

## Handshake

The first message must be:

```json
{
  "v": 0,
  "type": "hello",
  "ts": 1788940000000,
  "seq": 1,
  "payload": {
    "client": "phone",
    "name": "thomas-app"
  }
}
```

The server replies with `welcome`, followed by `state`. Do not enable motion
buttons until `welcome` and a state with `estop=false` have been received.

## Translation buttons: press, hold, release

On `pointerDown`/`touchDown`, send exactly one `jog_start`:

```json
{
  "v": 0,
  "type": "jog_start",
  "ts": 1788940000100,
  "seq": 2,
  "payload": {
    "axis": "z",
    "dir": 1,
    "step": "fine",
    "frame": "base"
  }
}
```

Current base-frame button mapping:

| UI meaning | axis | dir |
|---|---:|---:|
| toward green target / left in the current view | `x` | `1` |
| away from green target / right in the current view | `x` | `-1` |
| farther onto table / forward | `y` | `-1` |
| toward robot base / backward | `y` | `1` |
| up | `z` | `1` |
| down | `z` | `-1` |

Use `frame="base"` as the default for S4. `frame="tool"` is supported, but
its directions rotate with the gripper orientation and are easier to misuse
near the table.

While any translation, wrist, or gripper hold is active, send this message
every 150 ms:

```json
{
  "v": 0,
  "type": "jog_keepalive",
  "ts": 1788940000250,
  "seq": 3,
  "payload": {}
}
```

The server watchdog is 750 ms. `jog_keepalive` deliberately has no ACK. Do not
wait for an ACK before sending the next keepalive or stop message.

On `pointerUp`, `pointerCancel`, view disappearance, App backgrounding, or lost
focus, immediately send:

```json
{
  "v": 0,
  "type": "jog_stop",
  "ts": 1788940000400,
  "seq": 4,
  "payload": {}
}
```

## Wrist flex buttons

Press and hold:

```json
{
  "v": 0,
  "type": "joint_jog_start",
  "ts": 1788940001000,
  "seq": 10,
  "payload": {
    "joint": "wrist_flex",
    "dir": 1,
    "step": "fine"
  }
}
```

- `dir=1`: wrist flex positive, same behavior as Mac key `I`.
- `dir=-1`: wrist flex negative, same behavior as Mac key `K`.
- Send `jog_keepalive` every 150 ms while held.
- On release or cancellation, send `joint_jog_stop` with `{}` payload.

## Gripper buttons

Press and hold Open or Close:

```json
{
  "v": 0,
  "type": "gripper_jog_start",
  "ts": 1788940002000,
  "seq": 20,
  "payload": {
    "action": "close",
    "step": "fine"
  }
}
```

- `action` is `open` or `close`.
- Send `jog_keepalive` every 150 ms while held.
- On release or cancellation, send `gripper_jog_stop` with `{}` payload.
- Closing may stop automatically when contact is detected. The App must then
  clear its pressed state and stop sending keepalives.

## Step modes and current speeds

| Control | fine | coarse |
|---|---:|---:|
| free Cartesian jog | 0.09 m/s | 0.18 m/s |
| closed-gripper carry jog | 0.03 m/s | 0.06 m/s |
| free wrist flex | 45 deg/s | 90 deg/s |
| carrying wrist flex | 15 deg/s | 30 deg/s |
| gripper open | 90 deg/s | 180 deg/s |
| gripper close | 30 deg/s | 60 deg/s |

Use `fine` by default near the table, cube, or goal. The selected step is sent
inside every start payload. The optional discrete message below updates the
server's displayed mode:

```json
{
  "v": 0,
  "type": "mode",
  "ts": 1788940003000,
  "seq": 30,
  "payload": {"step": "fine"}
}
```

## ACK behavior

The start command remains active until a stop, contact stop, watchdog timeout,
workspace limit, or failure. Its final ACK therefore may arrive only after the
button is released. The App must not wait for the start ACK before sending
keepalives or the stop command.

The release sequence is:

1. Send the matching stop message with a new `seq`.
2. Receive the stop message's ACK, normally `status="ok"` and a reason ending
   in `_stop_requested`.
3. Receive the original start message's final ACK, normally `status="ok"` and
   a reason ending in `_stopped`.
4. Use `seq` to match every ACK to its request. Messages can arrive interleaved
   with state broadcasts.

An ACK has this form:

```json
{
  "v": 0,
  "type": "ack",
  "ts": 1788940003100,
  "seq": 30,
  "payload": {
    "seq": 30,
    "status": "ok",
    "reason": "mode_updated"
  }
}
```

If `status="rejected"`, do not automatically resend the command. Display the
reason and wait for a new user action.

## State messages

The server broadcasts state at approximately 5 Hz. The payload can include:

```json
{
  "ee_pos": [0.02, -0.14, 0.16],
  "gripper": "open",
  "step": "fine",
  "ik": "ok",
  "estop": false,
  "object_pos": [0.005, -0.2, 0.1025],
  "goal_pos": [0.075, -0.2, 0.1035],
  "task_success": false,
  "episode": 0
}
```

Treat `ik` and `gripper` as display/status strings. Use the Boolean `estop` to
enable or block motion controls. Unknown future state fields must be ignored.

## Reset and emergency stop

Scene reset:

```json
{"v":0,"type":"reset_scene","ts":1788940004000,"seq":40,"payload":{}}
```

Emergency stop:

```json
{"v":0,"type":"estop","ts":1788940004100,"seq":41,"payload":{}}
```

Reset emergency stop only after the server has frozen motion:

```json
{"v":0,"type":"estop_reset","ts":1788940004200,"seq":42,"payload":{}}
```

If the controlling WebSocket disconnects during an active command, the server
engages emergency stop. After reconnecting, read `state.estop`; do not silently
resume the old held button. Require a fresh user press after reset.

## UI/event requirements

- Use touch-down and touch-up/cancel events. A tap-only callback cannot provide
  press-and-hold control.
- Allow only one active held control at a time.
- Highlight a button only while its hold session is active.
- Always stop on release, cancel, background, navigation away, socket error, or
  loss of control ownership.
- Never buffer or automatically retry motion commands.
- Disable motion buttons while disconnected, before `welcome`, or while
  `state.estop=true`.
- Keep Estop available as a discrete, prominent button.

The old one-shot `move_delta` and `gripper` messages remain supported for
backward compatibility, but the new App control surface should use the hold
messages above.
