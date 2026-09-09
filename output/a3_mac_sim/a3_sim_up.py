#!/usr/bin/env python3

import argparse
import math
import os
import time

from a3_transport import Bridge

os.environ.setdefault(
    "LEISAAC_ASSETS_ROOT",
    "/root/Documents/robotics/upstream/leisaac/assets",
)

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(
    description="A3: wait for one Mac command, move SO-101 up 2 cm."
)
parser.add_argument("--move_steps", type=int, default=180)
parser.add_argument("--hold_steps", type=int, default=180)
parser.add_argument("--ws-host", default="127.0.0.1")
parser.add_argument("--ws-port", type=int, default=8766)
parser.add_argument("--log-file", default="/root/Documents/robotics/data/network_logs/a3_smoke.jsonl")

AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
if args_cli.move_steps < 1 or args_cli.hold_steps < 30:
    parser.error("move_steps must be >= 1 and hold_steps >= 30")
bridge = Bridge(args_cli.ws_host, args_cli.ws_port, args_cli.log_file)

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


import torch
import isaaclab.sim as sim_utils

from isaaclab.assets import Articulation
from isaaclab.controllers import (
    DifferentialIKController,
    DifferentialIKControllerCfg,
)
from isaaclab.utils.math import (
    matrix_from_quat,
    quat_inv,
    subtract_frame_transforms,
)
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

UP_DELTA_M = 0.02
PASS_ERROR_M = 0.005
STABLE_STEPS = 30

DLS_LAMBDA = 0.05
MAX_JOINT_DELTA_RAD = 0.01
SAFETY_COMMAND_ERROR_M = 0.10


def step_simulation(sim, robot, target, sim_dt):
    robot.set_joint_position_target(target)
    robot.write_data_to_sim()

    sim.step()
    robot.update(sim_dt)


def get_ee_pose_in_base(robot, ee_body_id):
    root_pose_w = robot.data.root_pose_w
    ee_pose_w = robot.data.body_pose_w[:, ee_body_id]

    ee_pos_b, ee_quat_b = subtract_frame_transforms(
        root_pose_w[:, 0:3],
        root_pose_w[:, 3:7],
        ee_pose_w[:, 0:3],
        ee_pose_w[:, 3:7],
    )

    return ee_pos_b, ee_quat_b


def get_position_jacobian_in_base(
    robot,
    ee_jacobian_index,
    arm_joint_ids,
):
    position_jacobian_w = (
        robot.root_physx_view.get_jacobians()[
            :,
            ee_jacobian_index,
            0:3,
            arm_joint_ids,
        ]
    )

    rotation_b_w = matrix_from_quat(
        quat_inv(robot.data.root_quat_w)
    )

    position_jacobian_b = torch.bmm(
        rotation_b_w,
        position_jacobian_w,
    )

    return position_jacobian_b


def vector_list(tensor):
    if not torch.isfinite(tensor).all().item():
        return None
    return tensor[0].detach().cpu().tolist()


def main():
    bridge.start()
    sim_cfg = sim_utils.SimulationCfg(
        dt=1.0 / 60.0,
        device=args_cli.device,
    )
    sim = sim_utils.SimulationContext(sim_cfg)

    sim.set_camera_view(
        eye=[0.8, 0.8, 0.6],
        target=[0.0, -0.15, 0.18],
    )

    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func(
        "/World/GroundPlane",
        ground_cfg,
    )

    light_cfg = sim_utils.DomeLightCfg(
        intensity=3000.0,
        color=(0.75, 0.75, 0.75),
    )
    light_cfg.func(
        "/World/Light",
        light_cfg,
    )

    robot_cfg = SO101_FOLLOWER_CFG.replace(
        prim_path="/World/SO101"
    )
    robot_cfg.init_state.pos = (
        0.0,
        0.0,
        0.0,
    )

    robot_cfg.init_state.joint_pos = {
        name: math.radians(position_deg)
        for name, position_deg
        in INITIAL_JOINT_POS_DEG.items()
    }

    robot = Articulation(cfg=robot_cfg)

    sim.reset()
    robot.reset()

    sim_dt = sim.get_physics_dt()

    arm_joint_ids = [
        robot.joint_names.index(name)
        for name in ARM_JOINT_NAMES
    ]

    gripper_joint_id = robot.joint_names.index(
        GRIPPER_JOINT_NAME
    )

    ee_body_id = robot.body_names.index(
        EE_BODY_NAME
    )

    ee_jacobian_index = ee_body_id - 1

    default_joint_pos = (
        robot.data.default_joint_pos.clone()
    )
    default_joint_vel = (
        robot.data.default_joint_vel.clone()
    )

    robot.write_joint_state_to_sim(
        default_joint_pos,
        default_joint_vel,
    )
    robot.reset()

    # 等待姿態在重力下穩定。
    for _ in range(60):
        step_simulation(
            sim,
            robot,
            default_joint_pos,
            sim_dt,
        )

    initial_ee_pos_b, _ = get_ee_pose_in_base(
        robot,
        ee_body_id,
    )

    initial_arm_joint_pos = (
        robot.data.joint_pos[
            :,
            arm_joint_ids,
        ].clone()
    )

    gravity_hold_offset = (
        default_joint_pos[
            :,
            arm_joint_ids,
        ]
        - initial_arm_joint_pos
    )

    arm_joint_limits = (
        robot.data.soft_joint_pos_limits[
            :,
            arm_joint_ids,
            :,
        ].clone()
    )

    ik_cfg = DifferentialIKControllerCfg(
        command_type="position",
        use_relative_mode=False,
        ik_method="dls",
        ik_params={
            "lambda_val": DLS_LAMBDA,
        },
    )

    ik_controller = DifferentialIKController(
        ik_cfg,
        num_envs=1,
        device=robot.device,
    )
    ik_controller.reset()

    print("A3 SIM READY", flush=True)
    print(f"A3 WS READY ws://{args_cli.ws_host}:{bridge.port}", flush=True)
    print("A3 initial EE base position [m]:", vector_list(initial_ee_pos_b), flush=True)
    print("Waiting for one test_move_up: base-frame +Z 0.020 m.", flush=True)
    print("This smoke protocol does not implement the full A-3/Thomas protocol.", flush=True)

    full_joint_target = default_joint_pos.clone()
    active = None
    motion_step = 0
    stable_count = 0
    bridge.ready.set()

    while simulation_app.is_running():
        if active is None:
            request = bridge.take()
            if request is None:
                step_simulation(sim, robot, full_joint_target, sim_dt)
                time.sleep(sim_dt)
                continue
            if time.monotonic() > request["deadline"]:
                bridge.finish(request, {"ok": False, "reason": "expired_before_execution"})
                continue
            active = request
            start_ee_pos_b, _ = get_ee_pose_in_base(robot, ee_body_id)
            start_ee_pos_b = start_ee_pos_b.clone()
            goal_ee_pos_b = start_ee_pos_b.clone()
            goal_ee_pos_b[:, 2] += UP_DELTA_M
            motion_step = 0
            stable_count = 0
            ik_controller.reset()
            print(f"[A3 EXEC] seq={active['seq']} start={vector_list(start_ee_pos_b)} "
                  f"target={vector_list(goal_ee_pos_b)}", flush=True)

        if time.monotonic() > active["deadline"]:
            bridge.finish(active, {"ok": False, "reason": "execution_timeout"})
            print("A3 FAIL: execution_timeout", flush=True)
            break

        alpha = min(float(motion_step + 1) / args_cli.move_steps, 1.0)
        command_ee_pos_b = start_ee_pos_b + alpha * (goal_ee_pos_b - start_ee_pos_b)

        current_ee_pos_b, current_ee_quat_b = (
            get_ee_pose_in_base(
                robot,
                ee_body_id,
            )
        )

        ik_controller.set_command(
            command_ee_pos_b,
            ee_quat=current_ee_quat_b,
        )

        position_jacobian = (
            get_position_jacobian_in_base(
                robot,
                ee_jacobian_index,
                arm_joint_ids,
            )
        )

        current_arm_joint_pos = (
            robot.data.joint_pos[
                :,
                arm_joint_ids,
            ]
        )

        desired_arm_joint_pos = (
            ik_controller.compute(
                current_ee_pos_b,
                current_ee_quat_b,
                position_jacobian,
                current_arm_joint_pos,
            )
        )

        if not (torch.isfinite(desired_arm_joint_pos).all().item()
                and torch.isfinite(current_ee_pos_b).all().item()):
            bridge.finish(active, {"ok": False, "reason": "nonfinite_ik"})
            print("A3 FAIL: nonfinite IK output; closing simulation", flush=True)
            break

        raw_joint_delta = (
            desired_arm_joint_pos
            - current_arm_joint_pos
        )

        limited_joint_delta = torch.clamp(
            raw_joint_delta,
            min=-MAX_JOINT_DELTA_RAD,
            max=MAX_JOINT_DELTA_RAD,
        )

        arm_joint_target = (
            current_arm_joint_pos
            + gravity_hold_offset
            + limited_joint_delta
        )

        arm_joint_target = torch.clamp(
            arm_joint_target,
            min=arm_joint_limits[:, :, 0],
            max=arm_joint_limits[:, :, 1],
        )

        full_joint_target = robot.data.joint_pos.clone()

        full_joint_target[
            :,
            arm_joint_ids,
        ] = arm_joint_target

        full_joint_target[
            :,
            gripper_joint_id,
        ] = default_joint_pos[
            :,
            gripper_joint_id,
        ]

        step_simulation(
            sim,
            robot,
            full_joint_target,
            sim_dt,
        )

        final_ee_pos_b, _ = get_ee_pose_in_base(
            robot,
            ee_body_id,
        )

        command_error_m = (
            torch.linalg.vector_norm(
                command_ee_pos_b
                - final_ee_pos_b,
                dim=1,
            )[0].item()
        )

        if not math.isfinite(command_error_m) or command_error_m > SAFETY_COMMAND_ERROR_M:
            bridge.finish(active, {"ok": False, "reason": "tracking_safety_stop"})
            print(f"A3 FAIL: tracking_error={command_error_m}; closing simulation", flush=True)
            break

        goal_error_m = torch.linalg.vector_norm(goal_ee_pos_b - final_ee_pos_b, dim=1)[0].item()
        motion_step += 1
        if motion_step >= args_cli.move_steps and goal_error_m <= PASS_ERROR_M:
            stable_count += 1
        else:
            stable_count = 0

        if motion_step % 60 == 0:
            print(f"[A3 MOVE] step={motion_step} goal_error_m={goal_error_m:.6f}", flush=True)

        if motion_step >= args_cli.move_steps + args_cli.hold_steps:
            ok = stable_count >= STABLE_STEPS and math.isfinite(goal_error_m)
            result = {
                "ok": ok,
                "reason": "completed" if ok else "position_tolerance_not_met",
                "frame": "base", "start_m": vector_list(start_ee_pos_b),
                "target_m": vector_list(goal_ee_pos_b), "end_m": vector_list(final_ee_pos_b),
                "actual_delta_m": vector_list(final_ee_pos_b - start_ee_pos_b),
                "position_error_m": goal_error_m, "tolerance_m": PASS_ERROR_M,
                "stable_steps": stable_count, "sim_steps": motion_step,
            }
            bridge.finish(active, result)
            print("A3 SIM UP 2CM " + ("PASS" if ok else "FAIL"), result, flush=True)
            active = None
            # Hold the last joint target; another movement requires a server restart.

        time.sleep(sim_dt)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("A3 stopped by user.", flush=True)
    finally:
        bridge.close()
        simulation_app.close()
