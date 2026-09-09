#!/usr/bin/env python3
"""Run SO-101 in Isaac Sim and execute v0 base-frame x/y/z move_delta."""

import argparse
import math
import os
import time

from a3_transport import Bridge, STEP_METERS

os.environ.setdefault(
    "LEISAAC_ASSETS_ROOT",
    "/root/Documents/robotics/upstream/leisaac/assets",
)

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(
    description="A-3: continuous v0 move_delta bridge for simulated SO-101."
)
parser.add_argument(
    "--move_steps",
    type=int,
    default=90,
    help="trajectory steps for a coarse 20 mm move; fine scales by distance",
)
parser.add_argument("--gripper_steps", type=int, default=120)
parser.add_argument("--hold_steps", type=int, default=90)
parser.add_argument("--ws-host", default="127.0.0.1")
parser.add_argument("--ws-port", type=int, default=8766)
parser.add_argument(
    "--log-file",
    default="/root/Documents/robotics/data/network_logs/a3_move_delta.jsonl",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
if args_cli.move_steps < 4 or args_cli.gripper_steps < 1 or args_cli.hold_steps < 10:
    parser.error("move_steps must be >= 4, gripper_steps >= 1, hold_steps >= 10")

bridge = Bridge(args_cli.ws_host, args_cli.ws_port, args_cli.log_file)
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


import torch
import isaaclab.sim as sim_utils

from isaaclab.assets import Articulation
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.utils.math import matrix_from_quat, quat_inv, subtract_frame_transforms
from leisaac.assets.robots.lerobot import SO101_FOLLOWER_CFG


ARM_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]
EE_BODY_NAME = "gripper"
GRIPPER_JOINT_NAME = "gripper"
INITIAL_JOINT_POS_DEG = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -60.0,
    "elbow_flex": 70.0,
    "wrist_flex": 40.0,
    "wrist_roll": 0.0,
    "gripper": 0.0,
}

PASS_ERROR_M = {"coarse": 0.005, "fine": 0.0015}
MOVE_STABLE_STEPS = 5
GRIPPER_STABLE_STEPS = 10
DLS_LAMBDA = 0.05
MAX_JOINT_DELTA_RAD = 0.01
SAFETY_COMMAND_ERROR_M = 0.10
WORKSPACE_RADIUS_M = 0.08
AXIS_INDEX = {"x": 0, "y": 1, "z": 2}
GRIPPER_TARGET_DEG = {"close": 0.0, "open": 100.0}
GRIPPER_TOLERANCE_DEG = 2.0


def step_simulation(sim, robot, target, sim_dt):
    robot.set_joint_position_target(target)
    robot.write_data_to_sim()
    sim.step()
    robot.update(sim_dt)


def get_ee_pose_in_base(robot, ee_body_id):
    root_pose_w = robot.data.root_pose_w
    ee_pose_w = robot.data.body_pose_w[:, ee_body_id]
    return subtract_frame_transforms(
        root_pose_w[:, 0:3],
        root_pose_w[:, 3:7],
        ee_pose_w[:, 0:3],
        ee_pose_w[:, 3:7],
    )


def get_position_jacobian_in_base(robot, ee_jacobian_index, arm_joint_ids):
    position_jacobian_w = robot.root_physx_view.get_jacobians()[
        :, ee_jacobian_index, 0:3, arm_joint_ids
    ]
    rotation_b_w = matrix_from_quat(quat_inv(robot.data.root_quat_w))
    return torch.bmm(rotation_b_w, position_jacobian_w)


def vector_list(tensor):
    if tensor is None or not torch.isfinite(tensor).all().item():
        return None
    return tensor[0].detach().cpu().tolist()


def diagnostics(
    start,
    target,
    end,
    error_m,
    tolerance_m,
    stable_steps,
    sim_steps,
    trajectory_steps,
    best_error_m,
    wall_time_s,
):
    return {
        "frame": "base",
        "start_m": vector_list(start),
        "target_m": vector_list(target),
        "end_m": vector_list(end),
        "actual_delta_m": vector_list(end - start),
        "position_error_m": error_m,
        "tolerance_m": tolerance_m,
        "stable_steps": stable_steps,
        "sim_steps": sim_steps,
        "trajectory_steps": trajectory_steps,
        "best_error_m": best_error_m,
        "wall_time_s": wall_time_s,
    }


def main():
    bridge.start()
    sim_cfg = sim_utils.SimulationCfg(dt=1.0 / 60.0, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[0.8, 0.8, 0.6], target=[0.0, -0.15, 0.18])

    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/GroundPlane", ground_cfg)
    light_cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    light_cfg.func("/World/Light", light_cfg)

    robot_cfg = SO101_FOLLOWER_CFG.replace(prim_path="/World/SO101")
    robot_cfg.init_state.pos = (0.0, 0.0, 0.0)
    robot_cfg.init_state.joint_pos = {
        name: math.radians(position_deg)
        for name, position_deg in INITIAL_JOINT_POS_DEG.items()
    }
    robot = Articulation(cfg=robot_cfg)

    sim.reset()
    robot.reset()
    sim_dt = sim.get_physics_dt()
    arm_joint_ids = [robot.joint_names.index(name) for name in ARM_JOINT_NAMES]
    gripper_joint_id = robot.joint_names.index(GRIPPER_JOINT_NAME)
    gripper_joint_limits = robot.data.soft_joint_pos_limits[
        :, gripper_joint_id, :
    ].clone()
    ee_body_id = robot.body_names.index(EE_BODY_NAME)
    ee_jacobian_index = ee_body_id - 1

    default_joint_pos = robot.data.default_joint_pos.clone()
    default_joint_vel = robot.data.default_joint_vel.clone()
    robot.write_joint_state_to_sim(default_joint_pos, default_joint_vel)
    robot.reset()
    for _ in range(60):
        step_simulation(sim, robot, default_joint_pos, sim_dt)

    initial_ee_pos_b, _ = get_ee_pose_in_base(robot, ee_body_id)
    initial_ee_pos_b = initial_ee_pos_b.clone()
    initial_arm_joint_pos = robot.data.joint_pos[:, arm_joint_ids].clone()
    gravity_hold_offset = default_joint_pos[:, arm_joint_ids] - initial_arm_joint_pos
    arm_joint_limits = robot.data.soft_joint_pos_limits[:, arm_joint_ids, :].clone()

    ik_cfg = DifferentialIKControllerCfg(
        command_type="position",
        use_relative_mode=False,
        ik_method="dls",
        ik_params={"lambda_val": DLS_LAMBDA},
    )
    ik_controller = DifferentialIKController(ik_cfg, num_envs=1, device=robot.device)
    ik_controller.reset()

    full_joint_target = default_joint_pos.clone()
    active = None
    motion_step = 0
    stable_count = 0
    start_ee_pos_b = None
    goal_ee_pos_b = None
    completion_status = "ok"
    tolerance_m = PASS_ERROR_M["coarse"]
    command_kind = None
    gripper_action = None
    start_gripper_pos = None
    goal_gripper_pos = None
    trajectory_steps = args_cli.move_steps
    best_error_m = math.inf
    motion_started_at = None

    initial_list = vector_list(initial_ee_pos_b)
    bridge.update_state(ee_pos=initial_list, gripper="closed", ik="ok")
    bridge.ready.set()
    print("A3 MOVE_DELTA SIM READY", flush=True)
    print(f"A3 WS READY ws://{args_cli.ws_host}:{bridge.port}", flush=True)
    print("A3 initial EE base position [m]:", initial_list, flush=True)
    print(
        "Accepted: move_delta axis=x/y/z, dir=+/-1, step=coarse/fine; "
        "gripper action=open/close; estop/estop_reset",
        flush=True,
    )
    print(
        "Step sizes: coarse=0.020 m (tol=0.005 m), "
        "fine=0.005 m (tol=0.0015 m); state broadcast=5 Hz",
        flush=True,
    )
    fine_trajectory_steps = math.ceil(
        args_cli.move_steps * STEP_METERS["fine"] / STEP_METERS["coarse"]
    )
    print(
        f"Motion profile: coarse={args_cli.move_steps} trajectory steps, "
        f"fine={fine_trajectory_steps} trajectory steps, "
        f"stable={MOVE_STABLE_STEPS}, hold_limit={args_cli.hold_steps}",
        flush=True,
    )

    while simulation_app.is_running():
        current_ee_pos_b, current_ee_quat_b = get_ee_pose_in_base(robot, ee_body_id)
        bridge.update_state(ee_pos=vector_list(current_ee_pos_b))

        if bridge.estop_event.is_set():
            interrupted = active
            if active is not None:
                active = None
            # Preserve the last actuator target. Replacing it with the measured
            # pose removes the gravity-hold offset and lets the arm sag.
            step_simulation(sim, robot, full_joint_target, sim_dt)
            frozen_ee_pos_b, _ = get_ee_pose_in_base(robot, ee_body_id)
            bridge.update_state(
                ee_pos=vector_list(frozen_ee_pos_b),
                estop=True,
                ik="stopped",
            )
            bridge.mark_estop_frozen()
            if interrupted is not None:
                bridge.finish(interrupted, "rejected", "estop")
                print(
                    f"A3 ESTOP FREEZE seq={interrupted['seq']} "
                    f"sim_steps={motion_step} ee_pos={vector_list(frozen_ee_pos_b)}",
                    flush=True,
                )
            time.sleep(sim_dt)
            continue

        if active is None:
            request = bridge.take()
            if request is None:
                step_simulation(sim, robot, full_joint_target, sim_dt)
                time.sleep(sim_dt)
                continue
            if time.monotonic() > request["deadline"]:
                bridge.finish(request, "rejected", "expired_before_execution")
                continue

            active = request
            command_kind = request["message"]["type"]
            payload = request["message"]["payload"]
            motion_step = 0
            stable_count = 0
            motion_started_at = time.monotonic()
            if command_kind == "move_delta":
                axis = payload["axis"]
                direction = payload["dir"]
                step_name = payload["step"]
                requested_delta_m = direction * STEP_METERS[step_name]
                tolerance_m = PASS_ERROR_M[step_name]
                trajectory_steps = math.ceil(
                    args_cli.move_steps
                    * STEP_METERS[step_name]
                    / STEP_METERS["coarse"]
                )
                best_error_m = math.inf

                start_ee_pos_b = current_ee_pos_b.clone()
                requested_goal = start_ee_pos_b.clone()
                requested_goal[:, AXIS_INDEX[axis]] += requested_delta_m
                lower = initial_ee_pos_b - WORKSPACE_RADIUS_M
                upper = initial_ee_pos_b + WORKSPACE_RADIUS_M
                goal_ee_pos_b = torch.clamp(requested_goal, min=lower, max=upper)
                was_clamped = not torch.allclose(goal_ee_pos_b, requested_goal, atol=1e-7)
                completion_status = "clamped" if was_clamped else "ok"

                ik_controller.reset()
                bridge.update_state(step=step_name, ik="moving")
                print(
                    f"[A3 EXEC] seq={active['seq']} axis={axis} dir={direction:+d} "
                    f"step={step_name} start={vector_list(start_ee_pos_b)} "
                    f"target={vector_list(goal_ee_pos_b)} "
                    f"trajectory_steps={trajectory_steps}",
                    flush=True,
                )
            else:
                gripper_action = payload["action"]
                start_gripper_pos = robot.data.joint_pos[:, gripper_joint_id].clone()
                requested_goal = torch.full_like(
                    start_gripper_pos,
                    math.radians(GRIPPER_TARGET_DEG[gripper_action]),
                )
                goal_gripper_pos = torch.clamp(
                    requested_goal,
                    min=gripper_joint_limits[:, 0],
                    max=gripper_joint_limits[:, 1],
                )
                completion_status = (
                    "clamped"
                    if not torch.allclose(goal_gripper_pos, requested_goal, atol=1e-7)
                    else "ok"
                )
                print(
                    f"[A3 GRIPPER] seq={active['seq']} action={gripper_action} "
                    f"start_deg={math.degrees(start_gripper_pos[0].item()):.3f} "
                    f"target_deg={math.degrees(goal_gripper_pos[0].item()):.3f}",
                    flush=True,
                )

        if time.monotonic() > active["deadline"]:
            bridge.update_state(ik="no_solution")
            bridge.finish(active, "rejected", "execution_timeout")
            bridge.engage_estop()
            print(f"[A3 REJECT] seq={active['seq']} reason=execution_timeout", flush=True)
            active = None
            continue

        if command_kind == "gripper":
            alpha = min(float(motion_step + 1) / args_cli.gripper_steps, 1.0)
            command_gripper_pos = start_gripper_pos + alpha * (
                goal_gripper_pos - start_gripper_pos
            )
            full_joint_target[:, gripper_joint_id] = command_gripper_pos
            step_simulation(sim, robot, full_joint_target, sim_dt)
            final_gripper_pos = robot.data.joint_pos[:, gripper_joint_id]
            gripper_error_rad = torch.abs(goal_gripper_pos - final_gripper_pos)[0].item()
            gripper_error_deg = math.degrees(gripper_error_rad)
            motion_step += 1
            if (
                motion_step >= args_cli.gripper_steps
                and gripper_error_deg <= GRIPPER_TOLERANCE_DEG
            ):
                stable_count += 1
            else:
                stable_count = 0

            if motion_step % 60 == 0:
                print(
                    f"[A3 GRIPPER MOVE] seq={active['seq']} step={motion_step} "
                    f"error_deg={gripper_error_deg:.3f}",
                    flush=True,
                )

            if (
                stable_count >= GRIPPER_STABLE_STEPS
                or motion_step >= args_cli.gripper_steps + args_cli.hold_steps
            ):
                ok = (
                    stable_count >= GRIPPER_STABLE_STEPS
                    and math.isfinite(gripper_error_deg)
                )
                details = {
                    "action": gripper_action,
                    "start_deg": math.degrees(start_gripper_pos[0].item()),
                    "target_deg": math.degrees(goal_gripper_pos[0].item()),
                    "end_deg": math.degrees(final_gripper_pos[0].item()),
                    "error_deg": gripper_error_deg,
                    "tolerance_deg": GRIPPER_TOLERANCE_DEG,
                    "stable_steps": stable_count,
                    "sim_steps": motion_step,
                }
                if ok:
                    bridge.update_state(gripper=gripper_action, ik="ok")
                    bridge.finish(active, completion_status, "completed", details)
                    print(
                        f"A3 GRIPPER PASS seq={active['seq']} status={completion_status}",
                        details,
                        flush=True,
                    )
                else:
                    bridge.finish(active, "rejected", "gripper_tolerance_not_met", details)
                    bridge.engage_estop()
                    print(f"A3 GRIPPER FAIL seq={active['seq']}", details, flush=True)
                active = None

            time.sleep(sim_dt)
            continue

        alpha = min(float(motion_step + 1) / trajectory_steps, 1.0)
        command_ee_pos_b = start_ee_pos_b + alpha * (goal_ee_pos_b - start_ee_pos_b)
        current_ee_pos_b, current_ee_quat_b = get_ee_pose_in_base(robot, ee_body_id)
        ik_controller.set_command(command_ee_pos_b, ee_quat=current_ee_quat_b)
        position_jacobian = get_position_jacobian_in_base(
            robot, ee_jacobian_index, arm_joint_ids
        )
        current_arm_joint_pos = robot.data.joint_pos[:, arm_joint_ids]
        desired_arm_joint_pos = ik_controller.compute(
            current_ee_pos_b,
            current_ee_quat_b,
            position_jacobian,
            current_arm_joint_pos,
        )

        if not (
            torch.isfinite(desired_arm_joint_pos).all().item()
            and torch.isfinite(current_ee_pos_b).all().item()
        ):
            bridge.update_state(ik="no_solution")
            bridge.finish(active, "rejected", "nonfinite_ik")
            bridge.engage_estop()
            full_joint_target = robot.data.joint_pos.clone()
            print(f"[A3 REJECT] seq={active['seq']} reason=nonfinite_ik", flush=True)
            active = None
            continue

        raw_joint_delta = desired_arm_joint_pos - current_arm_joint_pos
        limited_joint_delta = torch.clamp(
            raw_joint_delta,
            min=-MAX_JOINT_DELTA_RAD,
            max=MAX_JOINT_DELTA_RAD,
        )
        arm_joint_target = current_arm_joint_pos + gravity_hold_offset + limited_joint_delta
        arm_joint_target = torch.clamp(
            arm_joint_target,
            min=arm_joint_limits[:, :, 0],
            max=arm_joint_limits[:, :, 1],
        )
        full_joint_target = robot.data.joint_pos.clone()
        full_joint_target[:, arm_joint_ids] = arm_joint_target
        full_joint_target[:, gripper_joint_id] = default_joint_pos[:, gripper_joint_id]
        step_simulation(sim, robot, full_joint_target, sim_dt)

        final_ee_pos_b, _ = get_ee_pose_in_base(robot, ee_body_id)
        bridge.update_state(ee_pos=vector_list(final_ee_pos_b))
        command_error_m = torch.linalg.vector_norm(
            command_ee_pos_b - final_ee_pos_b, dim=1
        )[0].item()
        if not math.isfinite(command_error_m) or command_error_m > SAFETY_COMMAND_ERROR_M:
            bridge.update_state(ik="no_solution")
            bridge.finish(active, "rejected", "tracking_safety_stop")
            bridge.engage_estop()
            full_joint_target = robot.data.joint_pos.clone()
            print(
                f"[A3 REJECT] seq={active['seq']} reason=tracking_safety_stop "
                f"tracking_error_m={command_error_m}",
                flush=True,
            )
            active = None
            continue

        goal_error_m = torch.linalg.vector_norm(
            goal_ee_pos_b - final_ee_pos_b, dim=1
        )[0].item()
        best_error_m = min(best_error_m, goal_error_m)
        motion_step += 1
        if motion_step >= trajectory_steps and goal_error_m <= tolerance_m:
            stable_count += 1
        else:
            stable_count = 0

        if motion_step % 60 == 0:
            print(
                f"[A3 MOVE] seq={active['seq']} step={motion_step} "
                f"goal_error_m={goal_error_m:.6f}",
                flush=True,
            )

        if (
            stable_count >= MOVE_STABLE_STEPS
            or motion_step >= trajectory_steps + args_cli.hold_steps
        ):
            ok = (
                stable_count >= MOVE_STABLE_STEPS
                and math.isfinite(goal_error_m)
            )
            result_details = diagnostics(
                start_ee_pos_b,
                goal_ee_pos_b,
                final_ee_pos_b,
                goal_error_m,
                tolerance_m,
                stable_count,
                motion_step,
                trajectory_steps,
                best_error_m,
                time.monotonic() - motion_started_at,
            )
            if ok:
                bridge.update_state(ik="ok", ee_pos=vector_list(final_ee_pos_b))
                bridge.finish(active, completion_status, "completed", result_details)
                print(
                    f"A3 MOVE_DELTA PASS seq={active['seq']} status={completion_status}",
                    result_details,
                    flush=True,
                )
            else:
                bridge.update_state(ik="no_solution", ee_pos=vector_list(final_ee_pos_b))
                bridge.finish(active, "rejected", "position_tolerance_not_met", result_details)
                bridge.engage_estop()
                print(
                    f"A3 MOVE_DELTA FAIL seq={active['seq']}",
                    result_details,
                    flush=True,
                )
            active = None

        time.sleep(sim_dt)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("A3 move_delta stopped by user.", flush=True)
    finally:
        bridge.close()
        simulation_app.close()
