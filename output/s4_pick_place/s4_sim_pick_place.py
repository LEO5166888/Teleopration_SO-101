#!/usr/bin/env python3
"""S-4 fixed pick-and-place scene with A3 phone control and automatic reset."""

import argparse
import math
import os
import struct
import time
import zlib
from pathlib import Path

from a3_transport import Bridge, STEP_METERS
from s4_task_logic import StableSuccess, success_sample

os.environ.setdefault(
    "LEISAAC_ASSETS_ROOT",
    "/root/Documents/robotics/upstream/leisaac/assets",
)

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(
    description="S-4: fixed SO-101 pick-and-place scene with A3 controls."
)
parser.add_argument(
    "--move_steps",
    type=int,
    default=90,
    help="trajectory steps for a coarse 20 mm move; fine scales by distance",
)
parser.add_argument("--gripper_steps", type=int, default=240)
parser.add_argument("--hold_steps", type=int, default=90)
parser.add_argument("--ws-host", default="127.0.0.1")
parser.add_argument("--ws-port", type=int, default=8766)
parser.add_argument(
    "--log-file",
    default="/root/Documents/robotics/data/network_logs/s4_pick_place.jsonl",
)
parser.add_argument("--success-stable-steps", type=int, default=30)
parser.add_argument("--success-display-steps", type=int, default=90)
parser.add_argument(
    "--camera-smoke-test",
    action="store_true",
    help="capture one fixed S-3 RGB frame, validate it, then exit",
)
parser.add_argument(
    "--camera-output",
    default="/root/Documents/robotics/data/s3_camera/front_rgb_smoke.png",
    help="PNG path written by --camera-smoke-test",
)
parser.add_argument("--camera-width", type=int, default=640)
parser.add_argument("--camera-height", type=int, default=480)
parser.add_argument(
    "--record-demos",
    action="store_true",
    help="record successful S-4 episodes to the S-3 HDF5 contract",
)
parser.add_argument(
    "--record-output",
    default="/root/Documents/robotics/data/demos/s3_s4_demos.hdf5",
    help="HDF5 file used by --record-demos",
)
parser.add_argument(
    "--replay-hdf5",
    default=None,
    help="replay one S-3 HDF5 episode instead of accepting WebSocket commands",
)
parser.add_argument("--replay-episode", default="demo_0")
parser.add_argument(
    "--task-self-test",
    action="store_true",
    help="place the object in the goal once to verify success and automatic reset",
)
AppLauncher.add_app_launcher_args(parser)
args_cli = parser.parse_args()
if (
    args_cli.move_steps < 4
    or args_cli.gripper_steps < 1
    or args_cli.hold_steps < 10
    or args_cli.success_stable_steps < 1
    or args_cli.success_display_steps < 1
    or args_cli.camera_width < 1
    or args_cli.camera_height < 1
):
    parser.error("motion and task step counts must be positive")
if args_cli.record_demos and args_cli.replay_hdf5:
    parser.error("--record-demos and --replay-hdf5 cannot be used together")

bridge = Bridge(args_cli.ws_host, args_cli.ws_port, args_cli.log_file)
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


import torch
import isaaclab.sim as sim_utils

from isaaclab.assets import Articulation, RigidObject, RigidObjectCfg
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.sensors.camera import Camera, CameraCfg
from isaaclab.utils.math import matrix_from_quat, quat_inv, subtract_frame_transforms
from leisaac.assets.robots.lerobot import SO101_FOLLOWER_CFG


CARTESIAN_JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
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

PASS_ERROR_M = {"coarse": 0.005, "fine": 0.0020}
MOVE_STABLE_STEPS = 5
GRIPPER_STABLE_STEPS = 10
DLS_LAMBDA = 0.05
MAX_JOINT_DELTA_RAD = 0.01
SAFETY_COMMAND_ERROR_M = 0.10
WORKSPACE_RADIUS_M = 0.08
AXIS_INDEX = {"x": 0, "y": 1, "z": 2}
GRIPPER_TARGET_DEG = {"close": 0.0, "open": 100.0}
GRIPPER_TOLERANCE_DEG = 2.0
GRIPPER_CLOSE_COMPLETE_DEG = 10.0
GRIPPER_CONTACT_TRACKING_ERROR_DEG = 1.0
GRIPPER_CONTACT_CONFIRM_STEPS = 2
GRIPPER_CONTACT_PRELOAD_DEG = 0.25
GRIPPER_JOG_CONTACT_ERROR_DEG = 5.0
GRIPPER_JOG_CONTACT_CONFIRM_STEPS = 5
JOINT_STEP_DEG = {"fine": 5.0, "coarse": 15.0}
JOINT_TRAJECTORY_STEPS = {"fine": 30, "coarse": 90}
JOINT_TOLERANCE_DEG = 1.0
JOINT_STABLE_STEPS = 5
JOINT_JOG_SPEED_DEG_S = {"fine": 45.0, "coarse": 90.0}
JOINT_JOG_CARRY_SPEED_DEG_S = {"fine": 15.0, "coarse": 30.0}
JOG_SPEED_M_S = {"fine": 0.090, "coarse": 0.180}
JOG_CARRY_SPEED_M_S = {"fine": 0.030, "coarse": 0.060}
GRIPPER_JOG_SPEED_DEG_S = {"fine": 90.0, "coarse": 180.0}
GRIPPER_JOG_CLOSE_SPEED_DEG_S = {"fine": 30.0, "coarse": 60.0}

ROBOT_BASE_POS_W = (0.0, 0.0, 0.050)
ROBOT_SUPPORT_SIZE_M = (0.080, 0.080, 0.050)
ROBOT_SUPPORT_POS_W = (0.0, 0.0, ROBOT_SUPPORT_SIZE_M[2] / 2.0)
TABLE_POS_W = (0.035, -0.190, 0.045)
TABLE_SIZE_M = (0.300, 0.300, 0.090)
TABLE_TOP_Z_M = TABLE_POS_W[2] + TABLE_SIZE_M[2] / 2.0
OBJECT_SIZE_M = 0.025
OBJECT_START_POS_W = (0.005, -0.200, TABLE_TOP_Z_M + OBJECT_SIZE_M / 2.0 + 0.001)
GOAL_POS_W = (0.075, -0.200, TABLE_TOP_Z_M + OBJECT_SIZE_M / 2.0 + 0.001)
GOAL_HALF_EXTENTS_M = (0.025, 0.025, 0.020)
MAX_SUCCESS_LINEAR_SPEED_M_S = 0.020
MAX_SUCCESS_ANGULAR_SPEED_RAD_S = 0.50
FALLEN_OBJECT_Z_M = 0.030
RESET_ROBOT_SETTLE_STEPS = 30
RESET_OBJECT_SETTLE_STEPS = 15
RESET_OBJECT_MAX_ERROR_M = 0.005
INITIAL_ROBOT_MAX_ERROR_DEG = 3.0
S3_CAMERA_EYE_W = (0.45, 0.20, 0.55)
S3_CAMERA_TARGET_W = (0.035, -0.19, 0.12)
S3_COMMAND_TYPE = {
    "move_delta": 1,
    "jog_start": 1,
    "joint_delta": 2,
    "joint_jog_start": 2,
    "gripper": 3,
    "gripper_jog_start": 3,
}
S3_STEP_MODE = {"fine": 1, "coarse": 2}
_S3_CAMERA = None
_S3_BEFORE_STEP = None
_S3_RECORDER = None


def write_rgb_png(path, rgb):
    """Write an HWC uint8 RGB array as PNG using only the standard library."""
    height, width, channels = rgb.shape
    if channels != 3 or str(rgb.dtype) != "uint8":
        raise ValueError(f"expected HWC uint8 RGB, got shape={rgb.shape} dtype={rgb.dtype}")

    def chunk(kind, payload):
        body = kind + payload
        return struct.pack(">I", len(payload)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

    scanlines = b"".join(b"\x00" + rgb[row].tobytes() for row in range(height))
    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(scanlines, level=6))
    png += chunk(b"IEND", b"")
    output_path = Path(path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(png)


def capture_camera_smoke_frame(sim, camera, sim_dt, robot, target, task_object):
    """Warm up the renderer, validate one RGB frame, write it, and return metrics."""
    for _ in range(12):
        step_simulation(sim, robot, target, sim_dt, (task_object,))
        camera.update(sim_dt)

    rgb_output = camera.data.output.get("rgb")
    if rgb_output is None or rgb_output.numel() == 0:
        raise RuntimeError("S3 camera produced no RGB data; launch with --enable_cameras")
    rgb = rgb_output[0, ..., :3].detach().cpu()
    if rgb.dtype != torch.uint8:
        scale = 255.0 if float(rgb.max().item()) <= 1.0 else 1.0
        rgb = (rgb * scale).round().clamp(0, 255).to(torch.uint8)
    rgb_np = rgb.numpy()
    metrics = {
        "shape": list(rgb_np.shape),
        "dtype": str(rgb_np.dtype),
        "min": int(rgb_np.min()),
        "max": int(rgb_np.max()),
        "mean": float(rgb_np.mean()),
        "std": float(rgb_np.std()),
    }
    if metrics["max"] - metrics["min"] < 10 or metrics["std"] < 1.0:
        raise RuntimeError(f"S3 camera frame appears blank: {metrics}")
    write_rgb_png(args_cli.camera_output, rgb_np)
    return metrics


def step_simulation(sim, robot, target, sim_dt, rigid_objects=()):
    if _S3_BEFORE_STEP is not None:
        _S3_BEFORE_STEP(target)
    robot.set_joint_position_target(target)
    robot.write_data_to_sim()
    for rigid_object in rigid_objects:
        rigid_object.write_data_to_sim()
    sim.step()
    robot.update(sim_dt)
    for rigid_object in rigid_objects:
        rigid_object.update(sim_dt)
    if _S3_CAMERA is not None:
        _S3_CAMERA.update(sim_dt)


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
    command_frame,
    requested_delta_b,
    error_m,
    tolerance_m,
    stable_steps,
    sim_steps,
    trajectory_steps,
    best_error_m,
    wall_time_s,
):
    return {
        "command_frame": command_frame,
        "coordinates": "base",
        "start_m": vector_list(start),
        "target_m": vector_list(target),
        "end_m": vector_list(end),
        "requested_delta_base_m": vector_list(requested_delta_b),
        "actual_delta_m": vector_list(end - start),
        "position_error_m": error_m,
        "tolerance_m": tolerance_m,
        "stable_steps": stable_steps,
        "sim_steps": sim_steps,
        "trajectory_steps": trajectory_steps,
        "best_error_m": best_error_m,
        "wall_time_s": wall_time_s,
    }


def replay_hdf5_episode(sim, robot, task_object, sim_dt, gripper_joint_id):
    """Replay 30 Hz joint targets and require the S-4 success condition."""
    try:
        import h5py
    except ModuleNotFoundError as exc:
        raise RuntimeError("S3 replay requires h5py") from exc

    with h5py.File(args_cli.replay_hdf5, "r") as stream:
        episode_path = f"data/{args_cli.replay_episode}"
        if episode_path not in stream:
            raise RuntimeError(f"S3 replay episode not found: {episode_path}")
        episode = stream[episode_path]
        fps = float(episode.attrs["fps"])
        if abs(fps - 30.0) > 1e-6:
            raise RuntimeError(f"S3 replay requires fps=30, got {fps}")
        joint_targets = episode["actions/joint_target"][...]

    expected_shape = (joint_targets.shape[0], len(robot.joint_names))
    if joint_targets.ndim != 2 or joint_targets.shape != expected_shape:
        raise RuntimeError(
            f"S3 replay joint_target shape={joint_targets.shape}, "
            f"expected [N,{len(robot.joint_names)}]"
        )
    if joint_targets.shape[0] < 2:
        raise RuntimeError("S3 replay requires at least two action samples")

    print(
        f"S3 HDF5 REPLAY START episode={args_cli.replay_episode} "
        f"samples={joint_targets.shape[0]} fps=30",
        flush=True,
    )
    replay_target = None
    for index, target_np in enumerate(joint_targets):
        replay_target = torch.as_tensor(
            target_np,
            device=robot.device,
            dtype=robot.data.joint_pos.dtype,
        ).unsqueeze(0)
        if not torch.isfinite(replay_target).all().item():
            raise RuntimeError(f"S3 replay action contains NaN/Inf at sample {index}")
        for _ in range(2):
            step_simulation(
                sim,
                robot,
                replay_target,
                sim_dt,
                (task_object,),
            )
            time.sleep(sim_dt)
        if (index + 1) % 300 == 0:
            print(
                f"S3 HDF5 REPLAY progress={index + 1}/{joint_targets.shape[0]}",
                flush=True,
            )

    success_latch = StableSuccess(args_cli.success_stable_steps)
    last_details = None
    for hold_step in range(240):
        step_simulation(
            sim,
            robot,
            replay_target,
            sim_dt,
            (task_object,),
        )
        time.sleep(sim_dt)
        object_state_w = task_object.data.root_state_w[0]
        object_pos = object_state_w[:3].detach().cpu().tolist()
        gripper_deg = math.degrees(
            robot.data.joint_pos[0, gripper_joint_id].item()
        )
        sample_ok = success_sample(
            object_pos=object_pos,
            goal_pos=GOAL_POS_W,
            half_extents=GOAL_HALF_EXTENTS_M,
            linear_velocity=object_state_w[7:10].detach().cpu().tolist(),
            angular_velocity=object_state_w[10:13].detach().cpu().tolist(),
            gripper_open=gripper_deg >= 10.0,
            max_linear_speed=MAX_SUCCESS_LINEAR_SPEED_M_S,
            max_angular_speed=MAX_SUCCESS_ANGULAR_SPEED_RAD_S,
        )
        last_details = {
            "object_pos": object_pos,
            "goal_pos": list(GOAL_POS_W),
            "gripper_deg": gripper_deg,
            "stable_steps": success_latch.count,
            "hold_step": hold_step + 1,
        }
        if success_latch.update(sample_ok):
            last_details["stable_steps"] = success_latch.count
            print(f"S3 HDF5 REPLAY PASS {last_details}", flush=True)
            return

    raise RuntimeError(f"S3 HDF5 REPLAY FAIL {last_details}")


def main():
    global _S3_CAMERA, _S3_BEFORE_STEP, _S3_RECORDER
    if not args_cli.camera_smoke_test and not args_cli.replay_hdf5:
        bridge.start()
    sim_cfg = sim_utils.SimulationCfg(
        dt=1.0 / 60.0,
        device=args_cli.device,
        physx=sim_utils.PhysxCfg(
            solver_type=1,
            min_position_iteration_count=8,
            max_position_iteration_count=64,
            min_velocity_iteration_count=1,
            max_velocity_iteration_count=8,
        ),
    )
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view(eye=[0.8, 0.8, 0.6], target=[0.0, -0.19, 0.18])

    ground_cfg = sim_utils.GroundPlaneCfg()
    ground_cfg.func("/World/GroundPlane", ground_cfg)
    light_cfg = sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    light_cfg.func("/World/Light", light_cfg)

    support_cfg = sim_utils.CuboidCfg(
        size=ROBOT_SUPPORT_SIZE_M,
        collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.18, 0.18, 0.20)),
    )
    support_cfg.func(
        "/World/S4RobotSupport",
        support_cfg,
        translation=ROBOT_SUPPORT_POS_W,
    )

    table_cfg = sim_utils.CuboidCfg(
        size=TABLE_SIZE_M,
        collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.32, 0.24, 0.18)),
    )
    table_cfg.func("/World/S4Table", table_cfg, translation=TABLE_POS_W)

    goal_marker_cfg = sim_utils.CuboidCfg(
        size=(
            GOAL_HALF_EXTENTS_M[0] * 2.0,
            GOAL_HALF_EXTENTS_M[1] * 2.0,
            0.002,
        ),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.1, 0.8, 0.2)),
    )
    goal_marker_cfg.func(
        "/World/S4GoalMarker",
        goal_marker_cfg,
        translation=(GOAL_POS_W[0], GOAL_POS_W[1], TABLE_TOP_Z_M + 0.001),
    )

    object_cfg = RigidObjectCfg(
        prim_path="/World/S4Object",
        spawn=sim_utils.CuboidCfg(
            size=(OBJECT_SIZE_M, OBJECT_SIZE_M, OBJECT_SIZE_M),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                solver_position_iteration_count=64,
                solver_velocity_iteration_count=8,
                max_depenetration_velocity=1.0,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=0.015),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(
                static_friction=2.0,
                dynamic_friction=1.5,
                restitution=0.0,
                friction_combine_mode="max",
            ),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.85, 0.1, 0.1)),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=OBJECT_START_POS_W),
    )
    task_object = RigidObject(cfg=object_cfg)

    robot_cfg = SO101_FOLLOWER_CFG.replace(prim_path="/World/SO101")
    robot_cfg.init_state.pos = ROBOT_BASE_POS_W
    # The LeIsaac asset defaults to a 180-degree yaw.  S4 places the table on
    # the negative-Y side of the robot, so use an identity root orientation to
    # make the gripper face the table while keeping commands in the base frame.
    robot_cfg.init_state.rot = (1.0, 0.0, 0.0, 0.0)
    robot_cfg.spawn.articulation_props.solver_position_iteration_count = 64
    robot_cfg.spawn.articulation_props.solver_velocity_iteration_count = 8
    robot_cfg.init_state.joint_pos = {
        name: math.radians(position_deg)
        for name, position_deg in INITIAL_JOINT_POS_DEG.items()
    }
    robot = Articulation(cfg=robot_cfg)

    s3_camera = None
    if args_cli.camera_smoke_test or args_cli.record_demos:
        camera_cfg = CameraCfg(
            prim_path="/World/S3FrontCamera",
            update_period=(0.0 if args_cli.camera_smoke_test else 1.0 / 30.0),
            height=args_cli.camera_height,
            width=args_cli.camera_width,
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=24.0,
                focus_distance=0.75,
                horizontal_aperture=20.955,
                clipping_range=(0.05, 10.0),
            ),
        )
        s3_camera = Camera(cfg=camera_cfg)
        _S3_CAMERA = s3_camera

    sim.reset()
    robot.reset()
    task_object.reset()
    sim_dt = sim.get_physics_dt()
    if s3_camera is not None:
        camera_eyes = torch.tensor([S3_CAMERA_EYE_W], device=sim.device)
        camera_targets = torch.tensor([S3_CAMERA_TARGET_W], device=sim.device)
        s3_camera.set_world_poses_from_view(camera_eyes, camera_targets)
        s3_camera.reset()
    arm_joint_ids = [
        robot.joint_names.index(name) for name in CARTESIAN_JOINT_NAMES
    ]
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
    for _ in range(120):
        step_simulation(sim, robot, default_joint_pos, sim_dt, (task_object,))

    initial_joint_error_deg = math.degrees(
        torch.max(torch.abs(robot.data.joint_pos - default_joint_pos)).item()
    )
    if initial_joint_error_deg > INITIAL_ROBOT_MAX_ERROR_DEG:
        raise RuntimeError(
            "S4 initial robot pose did not settle: "
            f"max_joint_error_deg={initial_joint_error_deg:.3f} "
            f"limit_deg={INITIAL_ROBOT_MAX_ERROR_DEG:.3f}"
        )
    print(
        f"S4 INITIAL POSE CHECK PASS max_joint_error_deg={initial_joint_error_deg:.3f}",
        flush=True,
    )

    if args_cli.camera_smoke_test:
        metrics = capture_camera_smoke_frame(
            sim,
            s3_camera,
            sim_dt,
            robot,
            default_joint_pos,
            task_object,
        )
        print(
            f"S3 CAMERA SMOKE PASS output={args_cli.camera_output} metrics={metrics}",
            flush=True,
        )
        return

    object_initial_state = task_object.data.default_root_state.clone()
    reset_object_velocity = torch.zeros_like(object_initial_state[:, 7:])
    reset_object_staging_pose = object_initial_state[:, :7].clone()
    # Keep the cube away from the work cell while the position-driven robot is
    # teleported and settles.  Otherwise the reset impulse can hit a cube that
    # was just released at the goal and throw it away from the fixed start.
    reset_object_staging_pose[:, 0] = TABLE_POS_W[0] + 1.0
    reset_object_staging_pose[:, 1] = TABLE_POS_W[1] + 1.0
    reset_object_staging_pose[:, 2] = TABLE_TOP_Z_M + 0.5

    if args_cli.replay_hdf5:
        replay_hdf5_episode(
            sim,
            robot,
            task_object,
            sim_dt,
            gripper_joint_id,
        )
        return

    initial_ee_pos_b, _ = get_ee_pose_in_base(robot, ee_body_id)
    initial_ee_pos_b = initial_ee_pos_b.clone()
    print("S4 ROBOT BODY POSES [world m]", flush=True)
    for body_index, body_name in enumerate(robot.body_names):
        body_pos = robot.data.body_pose_w[0, body_index, :3].detach().cpu().tolist()
        print(f"  {body_name}: {body_pos}", flush=True)
    print("S4 JOINT LIMITS [deg]", flush=True)
    for joint_index, joint_name in enumerate(robot.joint_names):
        lower_rad = robot.data.soft_joint_pos_limits[0, joint_index, 0].item()
        upper_rad = robot.data.soft_joint_pos_limits[0, joint_index, 1].item()
        print(
            f"  {joint_name}: [{math.degrees(lower_rad):.1f}, "
            f"{math.degrees(upper_rad):.1f}]",
            flush=True,
        )
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
    held_gripper_target = default_joint_pos[:, gripper_joint_id].clone()
    active = None
    motion_step = 0
    stable_count = 0
    gripper_contact_count = 0
    start_ee_pos_b = None
    goal_ee_pos_b = None
    completion_status = "ok"
    tolerance_m = PASS_ERROR_M["coarse"]
    command_kind = None
    gripper_action = None
    gripper_jog_stalled = False
    start_gripper_pos = None
    goal_gripper_pos = None
    trajectory_steps = args_cli.move_steps
    best_error_m = math.inf
    motion_started_at = None
    jog_start_ee_pos_b = None
    jog_command_ee_pos_b = None
    jog_speed_m_s = None
    joint_jog_speed_deg_s = None
    gripper_jog_speed_deg_s = None
    episode_index = 0
    success_latch = StableSuccess(args_cli.success_stable_steps)
    success_display_remaining = 0
    self_test_countdown = 60 if args_cli.task_self_test else -1
    self_test_active = False

    s3_recorder = None
    if args_cli.record_demos:
        try:
            from s3_recorder import S3Recorder
        except ModuleNotFoundError as exc:
            if exc.name == "h5py":
                raise RuntimeError(
                    "S3 recording requires h5py; install it with "
                    "./isaaclab.sh -p -m pip install h5py"
                ) from exc
            raise
        rgb_shape = tuple(s3_camera.data.output["rgb"][0, ..., :3].shape)
        s3_recorder = S3Recorder(
            output_path=args_cli.record_output,
            joint_names=robot.joint_names,
            image_shape=rgb_shape,
            task="S4 fixed red-cube pick and place",
        )
        _S3_RECORDER = s3_recorder

    def s3_control_context():
        operator_delta = torch.zeros(5, device=robot.device, dtype=torch.float32)
        source_ts_ms = -1
        source_seq = -1
        command_code = 0
        step_code = 0
        if active is None:
            return operator_delta, source_ts_ms, source_seq, command_code, step_code

        message = active["message"]
        kind = message["type"]
        payload = message["payload"]
        source_ts_ms = int(message.get("ts", -1))
        source_seq = int(message.get("seq", active.get("seq", -1)))
        command_code = S3_COMMAND_TYPE.get(kind, 0)
        step_code = S3_STEP_MODE.get(payload.get("step"), 0)

        if kind == "jog_start":
            delta_command = torch.zeros((1, 3), device=robot.device)
            delta_command[:, AXIS_INDEX[payload["axis"]]] = (
                payload["dir"] * jog_speed_m_s / 30.0
            )
            if payload.get("frame", "base") == "tool":
                _, current_quat_b = get_ee_pose_in_base(robot, ee_body_id)
                delta_command = torch.bmm(
                    matrix_from_quat(current_quat_b),
                    delta_command.unsqueeze(-1),
                ).squeeze(-1)
            operator_delta[:3] = delta_command[0]
        elif kind == "move_delta" and motion_step < trajectory_steps:
            operator_delta[:3] = requested_delta_b[0] * (
                2.0 / max(trajectory_steps, 1)
            )
        elif kind == "joint_jog_start" and payload["joint"] == "wrist_flex":
            operator_delta[3] = math.radians(
                payload["dir"] * joint_jog_speed_deg_s / 30.0
            )
        elif kind == "joint_delta" and payload["joint"] == "wrist_flex":
            if motion_step < trajectory_steps:
                operator_delta[3] = math.radians(
                    payload["dir"] * JOINT_STEP_DEG[payload["step"]]
                ) * (2.0 / max(trajectory_steps, 1))
        elif kind == "gripper_jog_start":
            sign = 1.0 if payload["action"] == "open" else -1.0
            operator_delta[4] = math.radians(
                sign * gripper_jog_speed_deg_s / 30.0
            )
        elif kind == "gripper" and motion_step < args_cli.gripper_steps:
            operator_delta[4] = (
                goal_gripper_pos[0] - start_gripper_pos[0]
            ) * (2.0 / max(args_cli.gripper_steps, 1))
        return operator_delta, source_ts_ms, source_seq, command_code, step_code

    def s3_before_step(target):
        if s3_recorder is None or not s3_recorder.active:
            return
        rgb = s3_camera.data.output["rgb"][0, ..., :3].detach().cpu()
        if rgb.dtype != torch.uint8:
            scale = 255.0 if float(rgb.max().item()) <= 1.0 else 1.0
            rgb = (rgb * scale).round().clamp(0, 255).to(torch.uint8)
        ee_pos_b, ee_quat_b = get_ee_pose_in_base(robot, ee_body_id)
        operator_delta, source_ts, source_seq, command_code, step_code = (
            s3_control_context()
        )
        capture_started_now = s3_recorder.before_physics_step(
            joint_pos=robot.data.joint_pos[0].detach().cpu().numpy(),
            joint_vel=robot.data.joint_vel[0].detach().cpu().numpy(),
            ee_pose=torch.cat((ee_pos_b[0], ee_quat_b[0])).detach().cpu().numpy(),
            front_rgb=rgb.numpy(),
            operator_delta=operator_delta.detach().cpu().numpy(),
            joint_target=target[0].detach().cpu().numpy(),
            source_command_ts_ms=source_ts,
            source_command_seq=source_seq,
            estop=bridge.estop_event.is_set(),
            command_type=command_code,
            step_mode=step_code,
        )
        if capture_started_now:
            print(
                f"S3 CAPTURE START episode={episode_index} "
                f"first_command_seq={source_seq} fps=30",
                flush=True,
            )

    if s3_recorder is not None:
        _S3_BEFORE_STEP = s3_before_step

    def reset_episode(reason):
        nonlocal full_joint_target, held_gripper_target, episode_index
        nonlocal success_display_remaining, self_test_active

        bridge.ready.clear()
        if s3_recorder is not None and s3_recorder.active:
            discarded = s3_recorder.discard(f"reset_{reason}")
            print(
                f"S3 EPISODE DISCARDED reason=reset_{reason} samples={discarded}",
                flush=True,
            )
        # Stage 1: park the cube outside the work cell, then reset and settle
        # the robot.  Resetting both at once can generate a large contact
        # impulse when the robot leaves its placement pose.
        task_object.write_root_pose_to_sim(reset_object_staging_pose)
        task_object.write_root_velocity_to_sim(reset_object_velocity)
        task_object.reset()
        robot.write_joint_state_to_sim(default_joint_pos, default_joint_vel)
        robot.reset()
        full_joint_target = default_joint_pos.clone()
        held_gripper_target = default_joint_pos[:, gripper_joint_id].clone()
        ik_controller.reset()
        success_latch.reset()
        success_display_remaining = 0
        for _ in range(RESET_ROBOT_SETTLE_STEPS):
            step_simulation(
                sim, robot, full_joint_target, sim_dt, (task_object,)
            )

        # Stage 2: once the robot is stationary, restore the cube and clear all
        # velocity.  A short settle period lets it rest on the table before the
        # next episode accepts commands.
        task_object.write_root_pose_to_sim(object_initial_state[:, :7])
        task_object.write_root_velocity_to_sim(reset_object_velocity)
        task_object.reset()
        for _ in range(RESET_OBJECT_SETTLE_STEPS):
            step_simulation(
                sim, robot, full_joint_target, sim_dt, (task_object,)
            )
        episode_index += 1
        object_pos = vector_list(task_object.data.root_state_w[:, :3])
        reset_error_m = torch.linalg.vector_norm(
            task_object.data.root_state_w[:, :3]
            - object_initial_state[:, :3],
            dim=1,
        )[0].item()
        bridge.update_state(
            ee_pos=vector_list(get_ee_pose_in_base(robot, ee_body_id)[0]),
            gripper="closed",
            ik="ok",
            object_pos=object_pos,
            goal_pos=list(GOAL_POS_W),
            task_success=False,
            episode=episode_index,
        )
        bridge.log("task_reset", {
            "episode": episode_index,
            "reason": reason,
            "object_pos": object_pos,
            "reset_error_m": reset_error_m,
            "status": (
                "ok"
                if reset_error_m <= RESET_OBJECT_MAX_ERROR_M
                else "failed"
            ),
        })
        if s3_recorder is not None:
            s3_recorder.start(episode_index)
            print(
                f"S3 RECORDING ARMED episode={episode_index} fps=30 "
                f"output={args_cli.record_output}",
                flush=True,
            )
        bridge.ready.set()
        print(
            f"S4 AUTO RESET episode={episode_index} reason={reason} "
            f"object_pos={object_pos} reset_error_m={reset_error_m:.6f} "
            f"status={'ok' if reset_error_m <= RESET_OBJECT_MAX_ERROR_M else 'failed'}",
            flush=True,
        )
        if reason == "success" and self_test_active:
            self_test_active = False
            print("S4 TASK SELF TEST PASS", flush=True)

    initial_list = vector_list(initial_ee_pos_b)
    bridge.update_state(
        ee_pos=initial_list,
        gripper="closed",
        ik="ok",
        object_pos=vector_list(task_object.data.root_state_w[:, :3]),
        goal_pos=list(GOAL_POS_W),
        task_success=False,
        episode=episode_index,
    )
    if s3_recorder is not None:
        s3_recorder.start(episode_index)
        print(
            f"S3 RECORDING ARMED episode={episode_index} fps=30 "
            f"output={args_cli.record_output}",
            flush=True,
        )
    bridge.ready.set()
    print("S4 PICK_PLACE SIM READY", flush=True)
    print(f"S4 WS READY ws://{args_cli.ws_host}:{bridge.port}", flush=True)
    print("S4 initial EE base position [m]:", initial_list, flush=True)
    print("S4 object start world position [m]:", OBJECT_START_POS_W, flush=True)
    print("S4 goal world position [m]:", GOAL_POS_W, flush=True)
    print(
        "Accepted: move_delta/jog_start axis=x/y/z, dir=+/-1, "
        "step=coarse/fine, "
        "frame=base/tool (default base); "
        "joint_delta/joint_jog_start/stop; gripper action=open/close; "
        "gripper_jog_start/stop action=open/close; "
        "estop/estop_reset; reset_scene",
        flush=True,
    )
    print(
        "Step sizes: coarse=0.020 m (tol=0.005 m), "
        "fine=0.005 m (tol=0.0020 m); state broadcast=5 Hz",
        flush=True,
    )
    print(
        "Jog speeds: free fine=0.090/coarse=0.180 m/s; "
        "carry fine=0.030/coarse=0.060 m/s; "
        "release or 0.75 s keepalive timeout stops motion",
        flush=True,
    )
    print(
        "Gripper jog speeds: open fine=90/coarse=180 deg/s; "
        "close fine=30/coarse=60 deg/s; "
        "release stops at the current opening",
        flush=True,
    )
    print(
        "Joint jog speeds: free fine=45/coarse=90 deg/s; "
        "carry fine=15/coarse=30 deg/s; "
        "release stops at the current angle",
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
        object_state_w = task_object.data.root_state_w[0]
        object_pos_w = object_state_w[:3].detach().cpu().tolist()
        bridge.update_state(
            ee_pos=vector_list(current_ee_pos_b),
            object_pos=object_pos_w,
            goal_pos=list(GOAL_POS_W),
        )

        if bridge.estop_event.is_set():
            if s3_recorder is not None and s3_recorder.active:
                discarded = s3_recorder.discard("estop")
                print(
                    f"S3 EPISODE DISCARDED reason=estop samples={discarded}",
                    flush=True,
                )
            interrupted = active
            if active is not None:
                active = None
            # Preserve the last actuator target. Replacing it with the measured
            # pose removes the gravity-hold offset and lets the arm sag.
            step_simulation(sim, robot, full_joint_target, sim_dt, (task_object,))
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

        if self_test_countdown >= 0:
            if self_test_countdown == 0:
                self_test_pose = object_initial_state[:, :7].clone()
                self_test_pose[:, :3] = torch.tensor(
                    GOAL_POS_W,
                    device=task_object.device,
                    dtype=self_test_pose.dtype,
                )
                zero_velocity = torch.zeros(
                    (1, 6),
                    device=task_object.device,
                    dtype=object_initial_state.dtype,
                )
                task_object.write_root_pose_to_sim(self_test_pose)
                task_object.write_root_velocity_to_sim(zero_velocity)
                task_object.reset()
                bridge.update_state(gripper="open")
                self_test_active = True
                self_test_countdown = -1
                print("S4 SELF TEST: object placed in goal", flush=True)
            else:
                self_test_countdown -= 1

        if success_display_remaining > 0:
            step_simulation(sim, robot, full_joint_target, sim_dt, (task_object,))
            success_display_remaining -= 1
            if success_display_remaining == 0:
                reset_episode("success")
            time.sleep(sim_dt)
            continue

        sample_is_success = success_sample(
            object_pos=object_state_w[:3].detach().cpu().tolist(),
            goal_pos=GOAL_POS_W,
            half_extents=GOAL_HALF_EXTENTS_M,
            linear_velocity=object_state_w[7:10].detach().cpu().tolist(),
            angular_velocity=object_state_w[10:13].detach().cpu().tolist(),
            gripper_open=bridge.state_snapshot()["gripper"] == "open",
            max_linear_speed=MAX_SUCCESS_LINEAR_SPEED_M_S,
            max_angular_speed=MAX_SUCCESS_ANGULAR_SPEED_RAD_S,
        )
        was_latched = success_latch.latched
        is_latched = success_latch.update(sample_is_success)
        if is_latched and not was_latched:
            success_display_remaining = args_cli.success_display_steps
            bridge.ready.clear()
            bridge.update_state(task_success=True, ik="task_success")
            details = {
                "episode": episode_index,
                "object_pos": object_pos_w,
                "goal_pos": list(GOAL_POS_W),
                "stable_steps": success_latch.count,
            }
            bridge.log("task_success", details)
            print(f"S4 PICK_PLACE SUCCESS {details}", flush=True)
            # Success can occur while the release key is still held. Complete
            # that in-flight jog now so it cannot survive the display pause and
            # expire immediately after the automatic reset.
            if active is not None:
                bridge.finish(active, "ok", "task_success", details)
                active = None
                command_kind = None
                bridge.clear_jog_stop()
                bridge.clear_joint_jog_stop()
                bridge.clear_gripper_jog_stop()
            if s3_recorder is not None:
                saved = s3_recorder.finish_success()
                if saved is not None:
                    demo_name, sample_count = saved
                    print(
                        f"S3 HDF5 EPISODE SAVED {demo_name} "
                        f"samples={sample_count} output={args_cli.record_output}",
                        flush=True,
                    )
            continue

        if active is None and object_pos_w[2] < FALLEN_OBJECT_Z_M:
            reset_episode("object_fallen")
            time.sleep(sim_dt)
            continue

        if active is None:
            request = bridge.take()
            if request is None:
                step_simulation(sim, robot, full_joint_target, sim_dt, (task_object,))
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
            gripper_contact_count = 0
            gripper_jog_stalled = False
            motion_started_at = time.monotonic()
            if command_kind == "reset_scene":
                reset_episode("manual")
                bridge.finish(
                    active,
                    "ok",
                    "scene_reset",
                    {"episode": episode_index},
                )
                print(
                    f"S4 MANUAL RESET PASS seq={active['seq']} "
                    f"episode={episode_index}",
                    flush=True,
                )
                active = None
                time.sleep(sim_dt)
                continue
            if command_kind == "jog_start":
                axis = payload["axis"]
                direction = payload["dir"]
                step_name = payload["step"]
                command_frame = payload.get("frame", "base")
                jog_start_ee_pos_b = current_ee_pos_b.clone()
                jog_command_ee_pos_b = current_ee_pos_b.clone()
                carrying = bridge.state_snapshot()["gripper"] == "close"
                jog_speed_m_s = (
                    JOG_CARRY_SPEED_M_S[step_name]
                    if carrying
                    else JOG_SPEED_M_S[step_name]
                )
                ik_controller.reset()
                bridge.update_state(step=step_name, ik="jogging")
                print(
                    f"[S4 JOG START] seq={active['seq']} axis={axis} "
                    f"dir={direction:+d} speed={jog_speed_m_s:.3f}m/s "
                    f"profile={'carry' if carrying else 'free'} "
                    f"frame={command_frame} start={vector_list(jog_start_ee_pos_b)}",
                    flush=True,
                )
            elif command_kind == "move_delta":
                axis = payload["axis"]
                direction = payload["dir"]
                step_name = payload["step"]
                command_frame = payload.get("frame", "base")
                requested_delta_m = direction * STEP_METERS[step_name]
                tolerance_m = PASS_ERROR_M[step_name]
                trajectory_steps = math.ceil(
                    args_cli.move_steps
                    * STEP_METERS[step_name]
                    / STEP_METERS["coarse"]
                )
                best_error_m = math.inf

                start_ee_pos_b = current_ee_pos_b.clone()
                requested_delta_command = torch.zeros_like(start_ee_pos_b)
                requested_delta_command[:, AXIS_INDEX[axis]] = requested_delta_m
                if command_frame == "tool":
                    rotation_b_tool = matrix_from_quat(current_ee_quat_b)
                    requested_delta_b = torch.bmm(
                        rotation_b_tool,
                        requested_delta_command.unsqueeze(-1),
                    ).squeeze(-1)
                else:
                    requested_delta_b = requested_delta_command
                requested_goal = start_ee_pos_b + requested_delta_b
                lower = initial_ee_pos_b - WORKSPACE_RADIUS_M
                upper = initial_ee_pos_b + WORKSPACE_RADIUS_M
                goal_ee_pos_b = torch.clamp(requested_goal, min=lower, max=upper)
                was_clamped = not torch.allclose(goal_ee_pos_b, requested_goal, atol=1e-7)
                completion_status = "clamped" if was_clamped else "ok"

                ik_controller.reset()
                bridge.update_state(step=step_name, ik="moving")
                print(
                    f"[A3 EXEC] seq={active['seq']} axis={axis} dir={direction:+d} "
                    f"step={step_name} frame={command_frame} "
                    f"start={vector_list(start_ee_pos_b)} "
                    f"target={vector_list(goal_ee_pos_b)} "
                    f"trajectory_steps={trajectory_steps}",
                    flush=True,
                )
            elif command_kind == "joint_jog_start":
                joint_name = payload["joint"]
                direction = payload["dir"]
                step_name = payload["step"]
                joint_id = robot.joint_names.index(joint_name)
                joint_limits = robot.data.soft_joint_pos_limits[:, joint_id, :]
                start_joint_pos = robot.data.joint_pos[:, joint_id].clone()
                full_joint_target[:, joint_id] = start_joint_pos
                carrying = bridge.state_snapshot()["gripper"] == "close"
                joint_jog_speed_deg_s = (
                    JOINT_JOG_CARRY_SPEED_DEG_S[step_name]
                    if carrying
                    else JOINT_JOG_SPEED_DEG_S[step_name]
                )
                bridge.update_state(step=step_name, ik="joint_jogging")
                print(
                    f"[S4 JOINT JOG START] seq={active['seq']} "
                    f"joint={joint_name} dir={direction:+d} "
                    f"speed={joint_jog_speed_deg_s:.1f}deg/s "
                    f"profile={'carry' if carrying else 'free'} "
                    f"start_deg={math.degrees(start_joint_pos[0].item()):.3f}",
                    flush=True,
                )
            elif command_kind == "joint_delta":
                joint_name = payload["joint"]
                direction = payload["dir"]
                step_name = payload["step"]
                joint_id = robot.joint_names.index(joint_name)
                joint_limits = robot.data.soft_joint_pos_limits[:, joint_id, :]
                start_joint_pos = robot.data.joint_pos[:, joint_id].clone()
                requested_goal = start_joint_pos + math.radians(
                    direction * JOINT_STEP_DEG[step_name]
                )
                goal_joint_pos = torch.clamp(
                    requested_goal,
                    min=joint_limits[:, 0],
                    max=joint_limits[:, 1],
                )
                trajectory_steps = JOINT_TRAJECTORY_STEPS[step_name]
                completion_status = (
                    "clamped"
                    if not torch.allclose(goal_joint_pos, requested_goal, atol=1e-7)
                    else "ok"
                )
                bridge.update_state(ik="joint_moving")
                print(
                    f"[S4 JOINT] seq={active['seq']} joint={joint_name} "
                    f"dir={direction:+d} step={step_name} "
                    f"start_deg={math.degrees(start_joint_pos[0].item()):.3f} "
                    f"target_deg={math.degrees(goal_joint_pos[0].item()):.3f}",
                    flush=True,
                )
            elif command_kind == "gripper_jog_start":
                gripper_action = payload["action"]
                step_name = payload["step"]
                gripper_jog_speed_deg_s = (
                    GRIPPER_JOG_CLOSE_SPEED_DEG_S[step_name]
                    if gripper_action == "close"
                    else GRIPPER_JOG_SPEED_DEG_S[step_name]
                )
                start_gripper_pos = robot.data.joint_pos[
                    :, gripper_joint_id
                ].clone()
                held_gripper_target = start_gripper_pos.clone()
                full_joint_target[:, gripper_joint_id] = held_gripper_target
                bridge.update_state(
                    gripper=gripper_action,
                    step=step_name,
                    ik="gripper_jogging",
                )
                print(
                    f"[S4 GRIPPER JOG START] seq={active['seq']} "
                    f"action={gripper_action} "
                    f"speed={gripper_jog_speed_deg_s:.1f}deg/s "
                    f"start_deg={math.degrees(start_gripper_pos[0].item()):.3f}",
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
            timeout_reason = {
                "jog_start": "jog_watchdog_timeout",
                "joint_jog_start": "joint_jog_watchdog_timeout",
                "gripper_jog_start": "gripper_jog_watchdog_timeout",
            }.get(command_kind, "execution_timeout")
            bridge.update_state(ik="no_solution")
            bridge.finish(active, "rejected", timeout_reason)
            bridge.engage_estop()
            print(
                f"[A3 REJECT] seq={active['seq']} reason={timeout_reason}",
                flush=True,
            )
            active = None
            continue

        if command_kind == "joint_jog_start":
            if bridge.joint_jog_stop_event.is_set():
                bridge.clear_joint_jog_stop()
                final_joint_pos = robot.data.joint_pos[:, joint_id].clone()
                full_joint_target[:, joint_id] = final_joint_pos
                details = {
                    "joint": joint_name,
                    "direction": direction,
                    "step": step_name,
                    "start_deg": math.degrees(start_joint_pos[0].item()),
                    "end_deg": math.degrees(final_joint_pos[0].item()),
                    "sim_steps": motion_step,
                    "wall_time_s": time.monotonic() - motion_started_at,
                }
                bridge.update_state(ik="ok")
                bridge.finish(active, "ok", "joint_jog_stopped", details)
                print(
                    f"S4 JOINT JOG STOP seq={active['seq']}",
                    details,
                    flush=True,
                )
                active = None
                time.sleep(sim_dt)
                continue

            requested_target = full_joint_target[:, joint_id] + math.radians(
                direction * joint_jog_speed_deg_s * sim_dt
            )
            full_joint_target[:, joint_id] = torch.clamp(
                requested_target,
                min=joint_limits[:, 0],
                max=joint_limits[:, 1],
            )
            full_joint_target[:, gripper_joint_id] = held_gripper_target
            step_simulation(sim, robot, full_joint_target, sim_dt, (task_object,))
            final_joint_pos = robot.data.joint_pos[:, joint_id]
            motion_step += 1
            if motion_step % 30 == 0:
                print(
                    f"[S4 JOINT JOG] seq={active['seq']} "
                    f"joint={joint_name} step={motion_step} "
                    f"angle_deg={math.degrees(final_joint_pos[0].item()):.3f}",
                    flush=True,
                )
            time.sleep(sim_dt)
            continue

        if command_kind == "joint_delta":
            alpha = min(float(motion_step + 1) / trajectory_steps, 1.0)
            command_joint_pos = start_joint_pos + alpha * (
                goal_joint_pos - start_joint_pos
            )
            full_joint_target[:, joint_id] = command_joint_pos
            full_joint_target[:, gripper_joint_id] = held_gripper_target
            step_simulation(sim, robot, full_joint_target, sim_dt, (task_object,))
            final_joint_pos = robot.data.joint_pos[:, joint_id]
            joint_error_deg = math.degrees(
                torch.abs(goal_joint_pos - final_joint_pos)[0].item()
            )
            motion_step += 1
            if (
                motion_step >= trajectory_steps
                and joint_error_deg <= JOINT_TOLERANCE_DEG
            ):
                stable_count += 1
            else:
                stable_count = 0

            if (
                stable_count >= JOINT_STABLE_STEPS
                or motion_step >= trajectory_steps + args_cli.hold_steps
            ):
                ok = (
                    stable_count >= JOINT_STABLE_STEPS
                    and math.isfinite(joint_error_deg)
                )
                details = {
                    "joint": joint_name,
                    "start_deg": math.degrees(start_joint_pos[0].item()),
                    "target_deg": math.degrees(goal_joint_pos[0].item()),
                    "end_deg": math.degrees(final_joint_pos[0].item()),
                    "error_deg": joint_error_deg,
                    "tolerance_deg": JOINT_TOLERANCE_DEG,
                    "stable_steps": stable_count,
                    "sim_steps": motion_step,
                }
                if ok:
                    bridge.update_state(ik="ok")
                    bridge.finish(active, completion_status, "completed", details)
                    print(
                        f"S4 JOINT PASS seq={active['seq']} "
                        f"status={completion_status}",
                        details,
                        flush=True,
                    )
                else:
                    bridge.update_state(ik="no_solution")
                    bridge.finish(
                        active,
                        "rejected",
                        "joint_tolerance_not_met",
                        details,
                    )
                    bridge.engage_estop()
                    print(f"S4 JOINT FAIL seq={active['seq']}", details, flush=True)
                active = None

            time.sleep(sim_dt)
            continue

        if command_kind == "gripper_jog_start":
            if bridge.gripper_jog_stop_event.is_set():
                bridge.clear_gripper_jog_stop()
                final_gripper_pos = robot.data.joint_pos[
                    :, gripper_joint_id
                ].clone()
                details = {
                    "action": gripper_action,
                    "step": step_name,
                    "start_deg": math.degrees(start_gripper_pos[0].item()),
                    "end_deg": math.degrees(final_gripper_pos[0].item()),
                    "sim_steps": motion_step,
                    "wall_time_s": time.monotonic() - motion_started_at,
                    "contact_stalled": gripper_jog_stalled,
                }
                bridge.update_state(gripper=gripper_action, ik="ok")
                bridge.finish(active, "ok", "gripper_jog_stopped", details)
                print(
                    f"S4 GRIPPER JOG STOP seq={active['seq']}",
                    details,
                    flush=True,
                )
                active = None
                time.sleep(sim_dt)
                continue

            if not gripper_jog_stalled:
                direction = 1.0 if gripper_action == "open" else -1.0
                requested_target = held_gripper_target + math.radians(
                    direction * gripper_jog_speed_deg_s * sim_dt
                )
                held_gripper_target = torch.clamp(
                    requested_target,
                    min=gripper_joint_limits[:, 0],
                    max=gripper_joint_limits[:, 1],
                )

            full_joint_target[:, gripper_joint_id] = held_gripper_target
            step_simulation(sim, robot, full_joint_target, sim_dt, (task_object,))
            final_gripper_pos = robot.data.joint_pos[:, gripper_joint_id]
            closing_tracking_error_deg = math.degrees(
                torch.clamp(
                    final_gripper_pos - held_gripper_target,
                    min=0.0,
                )[0].item()
            )
            motion_step += 1
            if (
                gripper_action == "close"
                and closing_tracking_error_deg
                >= GRIPPER_JOG_CONTACT_ERROR_DEG
            ):
                gripper_contact_count += 1
            else:
                gripper_contact_count = 0

            if (
                not gripper_jog_stalled
                and gripper_contact_count
                >= GRIPPER_JOG_CONTACT_CONFIRM_STEPS
            ):
                held_gripper_target = torch.clamp(
                    final_gripper_pos
                    - math.radians(GRIPPER_CONTACT_PRELOAD_DEG),
                    min=gripper_joint_limits[:, 0],
                    max=gripper_joint_limits[:, 1],
                ).clone()
                full_joint_target[:, gripper_joint_id] = held_gripper_target
                gripper_jog_stalled = True
                bridge.update_state(gripper="close", ik="gripper_contact")
                print(
                    f"S4 GRIPPER JOG CONTACT seq={active['seq']} "
                    f"angle_deg={math.degrees(final_gripper_pos[0].item()):.3f}",
                    flush=True,
                )

            if motion_step % 30 == 0:
                print(
                    f"[S4 GRIPPER JOG] seq={active['seq']} "
                    f"action={gripper_action} step={motion_step} "
                    f"angle_deg={math.degrees(final_gripper_pos[0].item()):.3f}",
                    flush=True,
                )

            time.sleep(sim_dt)
            continue

        if command_kind == "gripper":

            alpha = min(float(motion_step + 1) / args_cli.gripper_steps, 1.0)
            command_gripper_pos = start_gripper_pos + alpha * (
                goal_gripper_pos - start_gripper_pos
            )
            held_gripper_target = command_gripper_pos.clone()
            full_joint_target[:, gripper_joint_id] = command_gripper_pos
            step_simulation(sim, robot, full_joint_target, sim_dt, (task_object,))
            final_gripper_pos = robot.data.joint_pos[:, gripper_joint_id]
            object_linear_speed_m_s = torch.linalg.vector_norm(
                task_object.data.root_state_w[0, 7:10]
            ).item()
            object_angular_speed_rad_s = torch.linalg.vector_norm(
                task_object.data.root_state_w[0, 10:13]
            ).item()
            closing_tracking_error_deg = math.degrees(
                torch.clamp(
                    final_gripper_pos - command_gripper_pos,
                    min=0.0,
                )[0].item()
            )
            gripper_error_rad = torch.abs(goal_gripper_pos - final_gripper_pos)[0].item()
            gripper_error_deg = math.degrees(gripper_error_rad)
            final_gripper_deg = math.degrees(final_gripper_pos[0].item())
            if gripper_action == "close":
                gripper_position_reached = (
                    final_gripper_deg <= GRIPPER_CLOSE_COMPLETE_DEG
                )
            else:
                gripper_position_reached = (
                    gripper_error_deg <= GRIPPER_TOLERANCE_DEG
                )
            motion_step += 1
            if (
                gripper_action == "close"
                and closing_tracking_error_deg
                >= GRIPPER_CONTACT_TRACKING_ERROR_DEG
            ):
                gripper_contact_count += 1
            else:
                gripper_contact_count = 0

            tracking_stalled = (
                gripper_contact_count >= GRIPPER_CONTACT_CONFIRM_STEPS
            )

            if tracking_stalled:
                # The position-driven finger is no longer following the closing
                # command.  Treat that sustained lag as a contact-like stall and
                # stop increasing the squeeze.  Keep only a small preload so a
                # grasped cube is held without driving toward the nominal hard-close
                # position through the collision geometry.
                held_gripper_target = torch.clamp(
                    final_gripper_pos - math.radians(GRIPPER_CONTACT_PRELOAD_DEG),
                    min=gripper_joint_limits[:, 0],
                    max=gripper_joint_limits[:, 1],
                ).clone()
                full_joint_target[:, gripper_joint_id] = held_gripper_target
                details = {
                    "action": gripper_action,
                    "contact_stop": True,
                    "contact_trigger": "tracking_stall",
                    "start_deg": math.degrees(start_gripper_pos[0].item()),
                    "command_deg": math.degrees(command_gripper_pos[0].item()),
                    "end_deg": final_gripper_deg,
                    "tracking_error_deg": closing_tracking_error_deg,
                    "tracking_error_threshold_deg": (
                        GRIPPER_CONTACT_TRACKING_ERROR_DEG
                    ),
                    "confirm_steps": gripper_contact_count,
                    "preload_deg": GRIPPER_CONTACT_PRELOAD_DEG,
                    "object_linear_speed_m_s": object_linear_speed_m_s,
                    "object_angular_speed_rad_s": object_angular_speed_rad_s,
                    "sim_steps": motion_step,
                }
                bridge.update_state(gripper="close", ik="ok")
                bridge.finish(active, "ok", "contact_stopped", details)
                print(
                    f"S4 GRIPPER CONTACT STOP seq={active['seq']}",
                    details,
                    flush=True,
                )
                active = None
                time.sleep(sim_dt)
                continue

            if (
                motion_step >= args_cli.gripper_steps
                and gripper_position_reached
            ):
                stable_count += 1
            else:
                stable_count = 0

            if motion_step % 60 == 0:
                print(
                    f"[A3 GRIPPER MOVE] seq={active['seq']} step={motion_step} "
                    f"error_deg={gripper_error_deg:.3f} "
                    f"tracking_error_deg={closing_tracking_error_deg:.3f} "
                    f"object_linear_speed_m_s={object_linear_speed_m_s:.4f} "
                    f"object_angular_speed_rad_s={object_angular_speed_rad_s:.3f}",
                    flush=True,
                )

            if (
                stable_count >= GRIPPER_STABLE_STEPS
                or motion_step >= args_cli.gripper_steps + args_cli.hold_steps
            ):
                ok = (
                    stable_count >= GRIPPER_STABLE_STEPS
                    and math.isfinite(gripper_error_deg)
                    and math.isfinite(final_gripper_deg)
                )
                details = {
                    "action": gripper_action,
                    "start_deg": math.degrees(start_gripper_pos[0].item()),
                    "target_deg": math.degrees(goal_gripper_pos[0].item()),
                    "end_deg": final_gripper_deg,
                    "error_deg": gripper_error_deg,
                    "tolerance_deg": GRIPPER_TOLERANCE_DEG,
                    "close_complete_max_deg": GRIPPER_CLOSE_COMPLETE_DEG,
                    "stable_steps": stable_count,
                    "sim_steps": motion_step,
                }
                if ok:
                    bridge.update_state(gripper=gripper_action, ik="ok")
                    bridge.finish(
                        active,
                        completion_status,
                        "completed",
                        details,
                    )
                    print(
                        f"A3 GRIPPER PASS seq={active['seq']} "
                        f"status={completion_status}",
                        details,
                        flush=True,
                    )
                    active = None
                else:
                    bridge.finish(active, "rejected", "gripper_tolerance_not_met", details)
                    bridge.engage_estop()
                    print(f"A3 GRIPPER FAIL seq={active['seq']}", details, flush=True)
                    active = None

            time.sleep(sim_dt)
            continue

        current_ee_pos_b, current_ee_quat_b = get_ee_pose_in_base(robot, ee_body_id)
        if command_kind == "jog_start":
            if bridge.jog_stop_event.is_set():
                bridge.clear_jog_stop()
                full_joint_target[:, arm_joint_ids] = robot.data.joint_pos[
                    :, arm_joint_ids
                ]
                full_joint_target[:, gripper_joint_id] = held_gripper_target
                bridge.update_state(ik="ok", ee_pos=vector_list(current_ee_pos_b))
                details = {
                    "command_frame": command_frame,
                    "coordinates": "base",
                    "start_m": vector_list(jog_start_ee_pos_b),
                    "end_m": vector_list(current_ee_pos_b),
                    "actual_delta_m": vector_list(
                        current_ee_pos_b - jog_start_ee_pos_b
                    ),
                    "sim_steps": motion_step,
                    "wall_time_s": time.monotonic() - motion_started_at,
                }
                bridge.finish(active, "ok", "jog_stopped", details)
                print(f"S4 JOG STOP seq={active['seq']}", details, flush=True)
                active = None
                time.sleep(sim_dt)
                continue

            requested_delta_command = torch.zeros_like(current_ee_pos_b)
            requested_delta_command[:, AXIS_INDEX[axis]] = (
                direction * jog_speed_m_s * sim_dt
            )
            if command_frame == "tool":
                rotation_b_tool = matrix_from_quat(current_ee_quat_b)
                jog_delta_b = torch.bmm(
                    rotation_b_tool,
                    requested_delta_command.unsqueeze(-1),
                ).squeeze(-1)
            else:
                jog_delta_b = requested_delta_command
            requested_jog_target = jog_command_ee_pos_b + jog_delta_b
            lower = initial_ee_pos_b - WORKSPACE_RADIUS_M
            upper = initial_ee_pos_b + WORKSPACE_RADIUS_M
            command_ee_pos_b = torch.clamp(
                requested_jog_target,
                min=lower,
                max=upper,
            )
            if torch.allclose(
                command_ee_pos_b,
                jog_command_ee_pos_b,
                atol=1e-7,
            ):
                bridge.clear_jog_stop()
                bridge.update_state(ik="ok", ee_pos=vector_list(current_ee_pos_b))
                details = {
                    "command_frame": command_frame,
                    "coordinates": "base",
                    "start_m": vector_list(jog_start_ee_pos_b),
                    "end_m": vector_list(current_ee_pos_b),
                    "actual_delta_m": vector_list(
                        current_ee_pos_b - jog_start_ee_pos_b
                    ),
                    "sim_steps": motion_step,
                    "wall_time_s": time.monotonic() - motion_started_at,
                }
                bridge.finish(active, "clamped", "workspace_limit", details)
                print(f"S4 JOG LIMIT seq={active['seq']}", details, flush=True)
                active = None
                time.sleep(sim_dt)
                continue
            jog_command_ee_pos_b = command_ee_pos_b.clone()
        else:
            alpha = min(float(motion_step + 1) / trajectory_steps, 1.0)
            command_ee_pos_b = start_ee_pos_b + alpha * (
                goal_ee_pos_b - start_ee_pos_b
            )

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
        full_joint_target[:, arm_joint_ids] = arm_joint_target
        full_joint_target[:, gripper_joint_id] = held_gripper_target
        step_simulation(sim, robot, full_joint_target, sim_dt, (task_object,))

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

        if command_kind == "jog_start":
            motion_step += 1
            if motion_step % 60 == 0:
                print(
                    f"[S4 JOG] seq={active['seq']} steps={motion_step} "
                    f"ee_pos={vector_list(final_ee_pos_b)}",
                    flush=True,
                )
            time.sleep(sim_dt)
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
                command_frame,
                requested_delta_b,
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
        _S3_BEFORE_STEP = None
        _S3_CAMERA = None
        if _S3_RECORDER is not None:
            _S3_RECORDER.close()
            _S3_RECORDER = None
        bridge.close()
        simulation_app.close()
