# A-3 test evidence

Date: 2026-09-07
Path: Mac -> SSH local forwarding -> RTXWS WebSocket -> Isaac Sim -> SO-101
Frame: robot base

The user returned both Mac and Server PASS output for all twelve translation
combinations. Values below are in millimetres.

| Command | Expected axis displacement | Actual delta `[x, y, z]` | Target error | Limit | Result |
|---|---:|---:|---:|---:|---|
| coarse X+ | +20 | `[+19.993, +0.044, -0.111]` | 0.120 | 5.0 | PASS |
| coarse X- | -20 | `[-20.000, -0.004, +0.007]` | 0.008 | 5.0 | PASS |
| coarse Y+ | +20 | `[-0.001, +19.599, +1.265]` | 1.327 | 5.0 | PASS |
| coarse Y- | -20 | `[+0.001, -19.666, -0.545]` | 0.639 | 5.0 | PASS |
| coarse Z+ | +20 | `[-0.003, +0.599, +18.400]` | 1.709 | 5.0 | PASS |
| coarse Z- | -20 | `[+0.003, +1.035, -22.183]` | 2.416 | 5.0 | PASS |
| fine X+ | +5 | `[+5.000, +0.003, -0.007]` | 0.007 | 1.5 | PASS |
| fine X- | -5 | `[-5.000, -0.000, +0.000]` | 0.001 | 1.5 | PASS |
| fine Y+ | +5 | `[-0.000, +4.874, +0.336]` | 0.358 | 1.5 | PASS |
| fine Y- | -5 | `[+0.000, -4.969, -0.047]` | 0.056 | 1.5 | PASS |
| fine Z+ | +5 | `[-0.001, +0.130, +4.790]` | 0.247 | 1.5 | PASS |
| fine Z- | -5 | `[+0.000, +0.132, -5.194]` | 0.234 | 1.5 | PASS |

The largest coarse error was 2.416 mm on Z-. The largest fine error was
0.358 mm on Y+. Both remained below their configured limits.

Evidence boundary:

- Confirmed: v0 `move_delta`, both step sizes, all six translation directions,
  final ack, measured state, and SO-101 motion in Isaac Sim.
- Confirmed: gripper open from -0.007 degrees to 99.992 degrees with 0.008
  degrees error, then close from 99.992 degrees to -0.007 degrees with 0.007
  degrees error. Both remained below the 2 degree limit.
- Confirmed: same-connection emergency stop during an in-flight move, frozen
  state, command blocking while stopped, and emergency-stop reset.
- Confirmed by the 2026-09-07 I-4 joint acceptance log: Thomas's App connected
  through the Mac relay; hello/test, six directions, gripper, acknowledgements,
  and JSONL evidence passed.

## Emergency-stop attempt 1

The first Isaac Sim estop run failed the freeze criterion. The command was sent
after 2.028 mm of measured Z motion, but the end effector subsequently drifted
8.035 mm, exceeding the 2 mm limit. The implementation had replaced the last
gravity-compensated actuator target with the measured joint pose at estop time.
It was changed to preserve the last actuator target and to acknowledge the
interrupted move only after one frozen physics step. A repeat Isaac Sim run is
required before marking estop complete.

## Emergency-stop attempt 2

The repeat run passed after preserving the last gravity-compensated actuator
target:

- estop sent after 2.028 mm of measured motion
- maximum measured post-freeze drift: 0.102 mm
- configured freeze limit: 2.000 mm
- a new movement was rejected while estop was active
- `estop_reset` returned the state to `estop=false`

Result: `A3 ESTOP SAME-CONNECTION INTERRUPT PASS`.

## Post-fix motion profile

The server motion profile was revised after the first phone run exposed equal
180-step trajectories for both coarse and fine commands. The accepted revision
uses coarse=90 trajectory steps, fine=23, five stable samples, and a 90-step
hold limit. A Mac surrogate rerun produced six fine-direction passes in
1.49-1.65 seconds, with target errors from 0.111 to 0.498 mm. Thomas's phone
acceptance log reports fine moves at about 1.6 seconds and coarse moves at about
5-6 seconds.

## Phone-App acceptance

The I-4 Notion work log records the 2026-09-07 joint result:

- phone connected to `ws://192.168.50.13:8765` through the Mac relay
- hello/test passed
- all six directions passed
- gripper passed
- JSONL evidence was saved

The user additionally confirmed that Thomas's agent reported the A-3 tests as
passing. The I-4 ticket is marked complete. Follow-up items for protocol v0.1
include ack semantics, state field values, single-client policy, the observed
Mac-relay/simulator clock offset, and coarse-speed policy; these do not block
the frozen v0 A-3 acceptance.

## Final conclusion

The Leo-side Server-to-Isaac-Sim implementation is demonstrated with a Mac
surrogate client for coarse/fine XYZ translation, gripper open/close, emergency
stop, reset, final acknowledgements, state feedback, and JSONL logging. The
saved JSONL was inspected and contains the incoming estop, `estop_engaged`
acknowledgement, interrupted-move rejection, `estop_active` command rejection,
reset acknowledgement, and final `estop=false` state.

A-3 has met its frozen v0 implementation and phone-integration acceptance
boundary. The A-3 Notion ticket is in review with a completion log and can be
moved to complete; I-4 is already marked complete.
