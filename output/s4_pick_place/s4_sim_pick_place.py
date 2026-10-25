#!/usr/bin/env python3
"""S-4 固定場景 pick-and-place 主程式。

閱讀這支程式時，先把它想成「一個 60 Hz 的工廠主迴圈」，不要逐行硬讀。
它同時串起四個部分：

1. 網路入口：``Bridge`` 在另一條 thread 接收手機 WebSocket 命令。
2. 模擬控制：本檔主 thread 從 queue 取命令，計算 IK／關節目標並推進 Isaac Sim。
3. 任務判定：``success_sample`` 與 ``StableSuccess`` 判斷方塊是否穩定放入目標區。
4. 資料錄製：可選擇啟用 ``S3Recorder``，以 30 Hz 寫入 HDF5。

最重要的資料流：

    Phone JSON
        -> Bridge 驗證並放入 queue
        -> main() 取出命令
        -> IK／關節／夾爪控制
        -> step_simulation() 推進一次物理
        -> 更新 state、成功條件與 recorder
        -> Bridge 回傳 ack/state

建議閱讀順序：

    main() 場景建立
        -> while simulation_app.is_running() 主迴圈
        -> step_simulation()
        -> reset_episode()
        -> 最後再看各種 command_kind 的細節

目前使用原始 base-frame 控制：WebXR 位移映射到 robot base XYZ，
並以 gripper link 原點作為 IK 追蹤點。
"""

import argparse
import math
import os
import struct
import time
import zlib
from pathlib import Path

# 本地模組：
# - Bridge：WebSocket thread 與 Isaac Sim 主 thread 之間的通訊橋。
# - STEP_METERS：把 coarse/fine 轉成實際公尺數。
# - success_sample / StableSuccess：不依賴 Isaac Sim 的純任務判定邏輯。
from a3_transport import Bridge, STEP_METERS
from s4_task_logic import (
    PitchOnlyClutch,
    StableSuccess,
    map_calibrated_webxr_translation_to_base as map_webxr_translation_to_base,
    quaternion_xyzw_to_rotation_vector,
    success_sample,
)

# Isaac Lab 載入 LeIsaac SO-101 asset 前，需要先知道 asset 根目錄。
# setdefault 表示：外部若已設定就尊重外部值，否則才使用這個預設路徑。
os.environ.setdefault(
    "LEISAAC_ASSETS_ROOT",
    "/root/Documents/robotics/upstream/leisaac/assets",
)

# AppLauncher 必須先建立 Isaac Sim 應用程式，之後才能安全 import torch/Isaac Lab 模組。
from isaaclab.app import AppLauncher


# -----------------------------------------------------------------------------
# [啟動參數]
# 這些參數只在程式啟動時決定一次，例如 WebSocket port、錄製位置與測試模式。
# -----------------------------------------------------------------------------
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
    "--operator-multiview",
    action="store_true",
    help=(
        "create S-8 side and gripper-follow cameras plus two operator viewports; "
        "phone control remains unchanged"
    ),
)
parser.add_argument("--operator-camera-width", type=int, default=480)
parser.add_argument("--operator-camera-height", type=int, default=360)
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
    "--record-session-id",
    default=None,
    help="D-5 session identifier; required with --record-demos",
)
parser.add_argument("--record-teleop-mode", choices=("buttons", "imu", "webxr"), default="buttons")
parser.add_argument("--record-operator-id", default="operator-001")
parser.add_argument("--record-attempt-index", type=int, default=1)
parser.add_argument("--record-condition-order", type=int, default=1)
parser.add_argument("--record-step-mode", choices=("fine", "coarse"), default="fine")
parser.add_argument("--record-robot-id", default="so101-sim-0")
parser.add_argument("--record-app-version", default="unknown")
parser.add_argument("--record-protocol-version", default="1")
parser.add_argument("--record-controller-version", default="s4-hold-control-v1")
parser.add_argument("--record-recorder-version", default="d5-recorder-v1")
parser.add_argument("--record-git-commit", default="unknown")
parser.add_argument("--record-rotation-gain", type=float, default=0.7)
parser.add_argument("--record-rotation-deadband-deg", type=float, default=2.5)
parser.add_argument("--record-rotation-smoothing", type=float, default=0.3)
parser.add_argument("--record-position-gain", type=float, default=None)
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
    or args_cli.operator_camera_width < 1
    or args_cli.operator_camera_height < 1
):
    parser.error("motion and task step counts must be positive")
if args_cli.record_demos and args_cli.replay_hdf5:
    parser.error("--record-demos and --replay-hdf5 cannot be used together")
if args_cli.record_demos and not args_cli.record_session_id:
    parser.error("--record-demos requires --record-session-id for D-5 metadata")
if args_cli.record_attempt_index < 1 or args_cli.record_condition_order < 1:
    parser.error("record attempt and condition order must be >= 1")
if args_cli.record_teleop_mode == "webxr" and args_cli.record_position_gain is None:
    parser.error("--record-teleop-mode webxr requires --record-position-gain")
if args_cli.operator_multiview and not args_cli.enable_cameras:
    parser.error("--operator-multiview requires --enable_cameras")

# 這裡只是「建立」兩個物件：
# - bridge 還沒開始監聽，真正啟動在 main() 裡的 bridge.start()。
# - AppLauncher 會啟動 Isaac Sim，simulation_app 用來判斷視窗是否仍在運作。
bridge = Bridge(args_cli.ws_host, args_cli.ws_port, args_cli.log_file)
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app


# -----------------------------------------------------------------------------
# [Isaac Sim 啟動後才能載入的模組]
# -----------------------------------------------------------------------------
import torch
import isaaclab.sim as sim_utils

from isaaclab.assets import Articulation, RigidObject, RigidObjectCfg
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.sensors.camera import Camera, CameraCfg
from isaaclab.utils.math import matrix_from_quat, quat_inv, subtract_frame_transforms
from leisaac.assets.robots.lerobot import SO101_FOLLOWER_CFG
# -----------------------------------------------------------------------------
# [控制與安全常數]
# -----------------------------------------------------------------------------
# Cartesian IK 目前只直接解三個主要手臂關節；wrist 與 gripper 由其他命令控制。
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

# move_delta 完成時允許的末端位置誤差。
PASS_ERROR_M = {"coarse": 0.005, "fine": 0.0020}
MOVE_STABLE_STEPS = 5
GRIPPER_STABLE_STEPS = 10
# DLS（damped least squares）IK 的阻尼；用來降低奇異點附近的不穩定。
DLS_LAMBDA = 0.05
# 每個 physics step 最多只允許單一 IK 關節改變 0.01 rad，避免瞬間跳動。
MAX_JOINT_DELTA_RAD = 0.01
# 若「命令末端位置」與「實際末端位置」相差超過 10 cm，視為危險並 Estop。
SAFETY_COMMAND_ERROR_M = 0.10
# 所有 Cartesian 命令都限制在初始 EE 位置周圍半徑 8 cm 的盒狀工作區。
WORKSPACE_RADIUS_M = 0.08
AXIS_INDEX = {"x": 0, "y": 1, "z": 2}
GRIPPER_TARGET_DEG = {"close": 0.0, "open": 100.0}
GRIPPER_TOLERANCE_DEG = 2.0
GRIPPER_CLOSE_COMPLETE_DEG = 10.0
# 夾爪關閉時，如果實際角度持續跟不上目標角度，推測手指已碰到物體。
GRIPPER_CONTACT_TRACKING_ERROR_DEG = 1.0
GRIPPER_CONTACT_CONFIRM_STEPS = 2
GRIPPER_CONTACT_PRELOAD_DEG = 0.25
GRIPPER_JOG_CONTACT_ERROR_DEG = 5.0
GRIPPER_JOG_CONTACT_CONFIRM_STEPS = 5
JOINT_STEP_DEG = {"fine": 5.0, "coarse": 15.0}
JOINT_TRAJECTORY_STEPS = {"fine": 30, "coarse": 90}
JOINT_TOLERANCE_DEG = 1.0
JOINT_STABLE_STEPS = 5
# carrying 版本刻意比較慢，降低夾住方塊後的晃動與碰撞。
JOINT_JOG_SPEED_DEG_S = {"fine": 45.0, "coarse": 90.0}
JOINT_JOG_CARRY_SPEED_DEG_S = {"fine": 15.0, "coarse": 30.0}
JOG_SPEED_M_S = {"fine": 0.090, "coarse": 0.180}
JOG_CARRY_SPEED_M_S = {"fine": 0.030, "coarse": 0.060}
GRIPPER_JOG_SPEED_DEG_S = {"fine": 90.0, "coarse": 180.0}
GRIPPER_JOG_CLOSE_SPEED_DEG_S = {"fine": 30.0, "coarse": 60.0}
# IMU 姿態模式先做受限的兩軸映射：手機 x 軸旋轉控 wrist_flex，
# 手機 y 軸旋轉控 wrist_roll，z 軸先忽略。這些上限是單次按住相對於
# 起始關節角的額外限制，關節本身的 soft limit 仍會再夾一次。
# XRViewer live calibration: pitching the phone upward should flex the wrist
# upward.  Roll keeps its previously calibrated sign.
ORIENT_FLEX_SIGN = 1.0
ORIENT_ROLL_SIGN = -1.0
ORIENT_MAX_FLEX_DELTA_DEG = 35.0
ORIENT_MAX_ROLL_DELTA_DEG = 60.0
# After direction calibration, use a 1:1 phone-to-EE position gain.  The
# workspace still clamps every axis to +/-8 cm around the initial pose.
POSE_POSITION_GAIN = 1.00
POSE_MAX_SPEED_M_S = 0.40
POSE_CARRY_MAX_SPEED_M_S = 0.12
# Pose tracking gets a slightly larger per-step joint budget than button jog;
# it remains bounded and does not alter the existing button/orientation modes.
POSE_MAX_JOINT_DELTA_RAD = 0.025

# -----------------------------------------------------------------------------
# [固定場景與成功條件]
# 名稱尾端 _W 代表 world frame；_M 代表單位為公尺。
# -----------------------------------------------------------------------------
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
# -----------------------------------------------------------------------------
# [S-3 錄製相關設定]
# operator command 會編碼成整數，方便寫入固定 dtype 的 HDF5 欄位。
# -----------------------------------------------------------------------------
S3_CAMERA_EYE_W = (0.45, 0.20, 0.55)
S3_CAMERA_TARGET_W = (0.035, -0.19, 0.12)
# S-8 操作輔助視角。Side Camera 固定在桌面側邊，重點是看清楚方塊
# 與桌面間有沒有空隙。Gripper Camera 會跟著末端位置移動，但保持世界座標
# 的水平方向，避免 wrist roll 讓操作畫面整張旋轉。
S8_SIDE_CAMERA_PRIM_PATH = "/World/S8SideCamera"
S8_GRIPPER_CAMERA_PRIM_PATH = "/World/S8GripperCamera"
S8_SIDE_CAMERA_EYE_W = (0.32, -0.20, 0.15)
S8_SIDE_CAMERA_TARGET_W = (0.02, -0.20, 0.11)
# Eye-in-hand 擺位：鏡頭從夾爪後上方移到「前上方」，視線對準
# 兩根手指之間的前下方。這樣不會讓 wrist 外殼擋住被夾物。
S8_GRIPPER_CAMERA_EYE_OFFSET_W = (0.0, -0.015, 0.065)
S8_GRIPPER_CAMERA_TARGET_OFFSET_W = (0.0, -0.120, -0.055)
# WebRTC 操作版面：兩個額外 viewport 疊在主 viewport 的左側，主視角自然
# 保留在右半邊。數值依目前 1828x977 操作畫面校準，啟動時每次套用。
S8_OPERATOR_WINDOW_X = 10
S8_OPERATOR_TOP_Y = 30
S8_OPERATOR_WINDOW_WIDTH = 750
S8_OPERATOR_WINDOW_HEIGHT = 440
S8_OPERATOR_WINDOW_GAP = 10
S3_COMMAND_TYPE = {
    "move_delta": 1,
    "jog_start": 1,
    "joint_delta": 2,
    "joint_jog_start": 2,
    "orient_start": 2,
    "pose_start": 4,
    "gripper": 3,
    "gripper_jog_start": 3,
}
S3_STEP_MODE = {"fine": 1, "coarse": 2}
# 這三個全域 hook 讓 step_simulation() 不用知道 recorder 的實作細節：
# 啟用錄製時會被 main() 填入；沒啟用時保持 None，不會做額外工作。
_S3_CAMERA = None
_S3_BEFORE_STEP = None
_S3_RECORDER = None
_S8_GRIPPER_CAMERA = None
_S8_EE_BODY_ID = None
_S8_TASK_OBJECT = None
_S8_GRIPPER_EYE_OFFSET_EE = None
_S8_GRIPPER_TARGET_OFFSET_EE = None


def update_s8_gripper_camera(robot):
    """讓 S-8 夾爪視角像真實 eye-in-hand 一樣剛性固定在末端。

    啟動時只利用任務方塊校準一次鏡頭擺位，並把鏡頭與看向點的
    位移轉成 EE local frame。之後每個 frame 只由 EE pose 重建鏡頭
    pose，不再追蹤物體的微小物理抖動。
    """
    global _S8_GRIPPER_EYE_OFFSET_EE, _S8_GRIPPER_TARGET_OFFSET_EE
    if _S8_GRIPPER_CAMERA is None or _S8_EE_BODY_ID is None:
        return
    ee_pose_w = robot.data.body_pose_w[:, _S8_EE_BODY_ID]
    ee_pos_w = ee_pose_w[:, :3]
    rotation_w_ee = matrix_from_quat(ee_pose_w[:, 3:7])
    fallback_eye_offset_w = ee_pos_w.new_tensor(
        S8_GRIPPER_CAMERA_EYE_OFFSET_W
    ).unsqueeze(0)
    fallback_target_offset_w = ee_pos_w.new_tensor(
        S8_GRIPPER_CAMERA_TARGET_OFFSET_W
    ).unsqueeze(0)
    camera_eye_w = ee_pos_w + fallback_eye_offset_w
    camera_target_w = ee_pos_w + fallback_target_offset_w

    # 只在第一幀用方塊對準；後續不再以方塊位置驅動鏡頭。
    if _S8_GRIPPER_EYE_OFFSET_EE is None:
        object_pos_w = (
            _S8_TASK_OBJECT.data.root_state_w[:, :3]
            if _S8_TASK_OBJECT is not None
            else camera_target_w
        )
        ee_to_object_w = object_pos_w - ee_pos_w
        horizontal_direction_w = ee_to_object_w.clone()
        horizontal_direction_w[:, 2] = 0.0
        horizontal_distance_m = torch.linalg.vector_norm(
            horizontal_direction_w,
            dim=1,
            keepdim=True,
        )
        object_distance_m = torch.linalg.vector_norm(
            ee_to_object_w,
            dim=1,
            keepdim=True,
        )
        object_is_near = torch.logical_and(
            torch.logical_and(
                torch.isfinite(object_pos_w).all(dim=1, keepdim=True),
                object_distance_m < 0.35,
            ),
            horizontal_distance_m > 0.01,
        )
        safe_horizontal_direction_w = horizontal_direction_w / torch.clamp(
            horizontal_distance_m,
            min=1.0e-6,
        )
        # 鏡頭從 EE 朝方塊方向前移 5.5 cm，再往上 7.5 cm。
        # 這會越過 wrist 外殼，模擬實體 eye-in-hand 支架的鏡頭位置。
        tracked_eye_w = ee_pos_w + safe_horizontal_direction_w * 0.055
        tracked_eye_w[:, 2] += 0.075
        camera_eye_w = torch.where(object_is_near, tracked_eye_w, camera_eye_w)
        camera_target_w = torch.where(
            object_is_near,
            object_pos_w,
            camera_target_w,
        )

        rotation_ee_w = rotation_w_ee.transpose(1, 2)
        _S8_GRIPPER_EYE_OFFSET_EE = torch.bmm(
            rotation_ee_w,
            (camera_eye_w - ee_pos_w).unsqueeze(-1),
        ).squeeze(-1)
        _S8_GRIPPER_TARGET_OFFSET_EE = torch.bmm(
            rotation_ee_w,
            (camera_target_w - ee_pos_w).unsqueeze(-1),
        ).squeeze(-1)

    camera_eye_w = ee_pos_w + torch.bmm(
        rotation_w_ee,
        _S8_GRIPPER_EYE_OFFSET_EE.unsqueeze(-1),
    ).squeeze(-1)
    camera_target_w = ee_pos_w + torch.bmm(
        rotation_w_ee,
        _S8_GRIPPER_TARGET_OFFSET_EE.unsqueeze(-1),
    ).squeeze(-1)

    _S8_GRIPPER_CAMERA.set_world_poses_from_view(
        camera_eye_w,
        camera_target_w,
    )


def create_s8_operator_viewports():
    """用兩個額外 viewport 顯示側視與夾爪視角。

    Camera prim 即使建立成功，不同 Isaac Sim/Kit 版本仍可能沒有
    ``create_viewport_window``。這裡只讓自動開視窗失敗，不影響手機控制與
    模擬主迴圈；使用者仍可從 Isaac Sim GUI 手動開 Viewport 2/3。
    """
    try:
        from omni.kit.viewport.utility import create_viewport_window
        from pxr import Sdf

        side_window = create_viewport_window(
            name="S8 Side View",
            width=S8_OPERATOR_WINDOW_WIDTH,
            height=S8_OPERATOR_WINDOW_HEIGHT,
            position_x=S8_OPERATOR_WINDOW_X,
            position_y=S8_OPERATOR_TOP_Y,
            camera_path=Sdf.Path(S8_SIDE_CAMERA_PRIM_PATH),
        )
        gripper_window = create_viewport_window(
            name="S8 Gripper View",
            width=S8_OPERATOR_WINDOW_WIDTH,
            height=S8_OPERATOR_WINDOW_HEIGHT,
            position_x=S8_OPERATOR_WINDOW_X,
            position_y=(
                S8_OPERATOR_TOP_Y
                + S8_OPERATOR_WINDOW_HEIGHT
                + S8_OPERATOR_WINDOW_GAP
            ),
            camera_path=Sdf.Path(S8_GRIPPER_CAMERA_PRIM_PATH),
        )
        if side_window is None or gripper_window is None:
            raise RuntimeError("create_viewport_window returned None")
        print(
            "S8 MULTIVIEW READY "
            f"side={S8_SIDE_CAMERA_PRIM_PATH} "
            f"gripper={S8_GRIPPER_CAMERA_PRIM_PATH} "
            f"layout=left-stack-{S8_OPERATOR_WINDOW_WIDTH}x"
            f"{S8_OPERATOR_WINDOW_HEIGHT}",
            flush=True,
        )
        # 必須保留 Python reference，否則某些 Kit 版本會回收 viewport proxy。
        return [side_window, gripper_window]
    except Exception as exc:
        print(
            "S8 CAMERAS READY BUT AUTO VIEWPORT UNAVAILABLE "
            f"error={type(exc).__name__}: {exc}",
            flush=True,
        )
        print(
            "Open Isaac Sim Window > Viewports > Viewport 2/3, then select "
            f"{S8_SIDE_CAMERA_PRIM_PATH} and {S8_GRIPPER_CAMERA_PRIM_PATH}.",
            flush=True,
        )
        return []


def write_rgb_png(path, rgb):
    """把 HWC 排列的 uint8 RGB 影像寫成 PNG；不依賴 Pillow/OpenCV。"""
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
    """相機冒煙測試：暖機、擷取一張影像、排除空白畫面，最後存成 PNG。"""
    # 先跑幾個 frame，避免剛啟動 renderer 時讀到尚未完成的畫面。
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
    """推進一個 1/60 秒 physics step，是整支程式最核心的小循環。"""
    # 錄製 hook 要在寫入本 step 的控制目標前取樣，才能對齊 observation/action。
    if _S3_BEFORE_STEP is not None:
        _S3_BEFORE_STEP(target)

    # 1. 把本 step 的 joint position target 寫進 simulator。
    robot.set_joint_position_target(target)
    robot.write_data_to_sim()
    for rigid_object in rigid_objects:
        rigid_object.write_data_to_sim()
    # 2. 真正讓 PhysX 向前走一個 dt。
    sim.step()

    # 3. 把 PhysX 計算後的新狀態讀回 Isaac Lab 的 data buffer。
    robot.update(sim_dt)
    for rigid_object in rigid_objects:
        rigid_object.update(sim_dt)
    # S-8 夾爪視角的 Camera prim 要在讀回最新 EE pose 後才移動。
    update_s8_gripper_camera(robot)
    # 相機也必須在 physics step 後更新，recorder 才能取得最新 RGB。
    if _S3_CAMERA is not None:
        _S3_CAMERA.update(sim_dt)


def get_ee_pose_in_base(robot, ee_body_id):
    """把末端執行器從 world frame 轉成 robot base frame 的 pose。"""
    root_pose_w = robot.data.root_pose_w
    ee_pose_w = robot.data.body_pose_w[:, ee_body_id]
    return subtract_frame_transforms(
        root_pose_w[:, 0:3],
        root_pose_w[:, 3:7],
        ee_pose_w[:, 0:3],
        ee_pose_w[:, 3:7],
    )


def get_position_jacobian_in_base(robot, ee_jacobian_index, arm_joint_ids):
    """取得位置 Jacobian，並從 world frame 旋轉到 robot base frame。"""
    position_jacobian_w = robot.root_physx_view.get_jacobians()[
        :, ee_jacobian_index, 0:3, arm_joint_ids
    ]
    rotation_b_w = matrix_from_quat(quat_inv(robot.data.root_quat_w))
    return torch.bmm(rotation_b_w, position_jacobian_w)


def vector_list(tensor):
    """把 GPU tensor 轉成可寫入 JSON 的 Python list；非有限值回傳 None。"""
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
    """整理一次 Cartesian 移動的驗收證據，供 log 與 ack 診斷使用。"""
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
    """以 30 Hz 重播 HDF5 joint target，最後仍須通過 S-4 成功條件。"""
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
    # HDF5 是 30 Hz、physics 是 60 Hz，所以每筆 target 連續跑兩個 physics steps。
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

    # 重播完不能只說「程式跑完」；還要檢查方塊真的穩定留在目標區。
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
    """建立場景並執行 60 Hz 控制主迴圈。"""
    global _S3_CAMERA, _S3_BEFORE_STEP, _S3_RECORDER # 跟,main說這三個變數是定義在外面的共用變數，不是重新宣告的變數
    global _S8_GRIPPER_CAMERA, _S8_EE_BODY_ID, _S8_TASK_OBJECT

    # -------------------------------------------------------------------------
    # [main 1/8] 啟動網路橋與 60 Hz 物理模擬
    # camera smoke test／HDF5 replay 都是離線模式，不需要接受手機命令。
    # -------------------------------------------------------------------------

    #如果現在不是在重播HDF5或是測試相機，就啟動websocket準備接收手機命令
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

    # -------------------------------------------------------------------------
    # [main 2/8] 建立固定場景：地面、燈光、支座、桌面、目標區、紅方塊、SO-101
    # -------------------------------------------------------------------------
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

    # 綠色 goal marker 只是視覺標記；真正的成功範圍由 GOAL_HALF_EXTENTS_M 判斷。
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

    # 紅色方塊是會受重力、摩擦與碰撞影響的 RigidObject。
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

    # 載入 LeIsaac 提供的 SO-101 articulation 設定，再覆寫本任務的初始姿勢。
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

    # -------------------------------------------------------------------------
    # [main 3/8] 可選相機：只有 smoke test 或正式錄製時才建立，減少平常負擔。
    # -------------------------------------------------------------------------
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

    # S-8 多視角只服務操作者，不改動 S-3 錄影相機、HDF5 schema
    # 或手機 WebSocket protocol。夾爪相機的 near clipping 較短，避免近距離
    # 的夾爪／方塊被 5 cm 的預設裁切面消掉。
    s8_side_camera = None
    s8_gripper_camera = None
    s8_viewport_windows = []
    if args_cli.operator_multiview:
        side_camera_cfg = CameraCfg(
            prim_path=S8_SIDE_CAMERA_PRIM_PATH,
            update_period=1.0 / 30.0,
            height=args_cli.operator_camera_height,
            width=args_cli.operator_camera_width,
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=20.0,
                focus_distance=0.35,
                horizontal_aperture=20.955,
                clipping_range=(0.02, 10.0),
            ),
        )
        gripper_camera_cfg = CameraCfg(
            prim_path=S8_GRIPPER_CAMERA_PRIM_PATH,
            update_period=1.0 / 30.0,
            height=args_cli.operator_camera_height,
            width=args_cli.operator_camera_width,
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(
                # 用較廣視角同時容納兩根手指與中間物體。
                focal_length=12.0,
                focus_distance=0.12,
                horizontal_aperture=20.955,
                clipping_range=(0.005, 2.0),
            ),
        )
        s8_side_camera = Camera(cfg=side_camera_cfg)
        s8_gripper_camera = Camera(cfg=gripper_camera_cfg)

    # -------------------------------------------------------------------------
    # [main 4/8] 第一次 reset，取得 joint/body ID，並讓初始姿勢先穩定下來。
    # Isaac Lab 後續多數 API 使用整數 ID，所以在這裡由名稱查一次並保存。
    # -------------------------------------------------------------------------
    sim.reset()
    robot.reset()
    task_object.reset()
    sim_dt = sim.get_physics_dt()
    if s3_camera is not None:
        camera_eyes = torch.tensor([S3_CAMERA_EYE_W], device=sim.device)
        camera_targets = torch.tensor([S3_CAMERA_TARGET_W], device=sim.device)
        s3_camera.set_world_poses_from_view(camera_eyes, camera_targets)
        s3_camera.reset()
    if s8_side_camera is not None:
        side_camera_eyes = torch.tensor([S8_SIDE_CAMERA_EYE_W], device=sim.device)
        side_camera_targets = torch.tensor(
            [S8_SIDE_CAMERA_TARGET_W], device=sim.device
        )
        s8_side_camera.set_world_poses_from_view(
            side_camera_eyes,
            side_camera_targets,
        )
        s8_side_camera.reset()
        s8_gripper_camera.reset()
    arm_joint_ids = [
        robot.joint_names.index(name) for name in CARTESIAN_JOINT_NAMES
    ]
    gripper_joint_id = robot.joint_names.index(GRIPPER_JOINT_NAME)
    gripper_joint_limits = robot.data.soft_joint_pos_limits[
        :, gripper_joint_id, :
    ].clone()
    wrist_flex_joint_id = robot.joint_names.index("wrist_flex")
    wrist_roll_joint_id = robot.joint_names.index("wrist_roll")
    wrist_body_id = robot.body_names.index("wrist")
    orient_joint_ids = [wrist_flex_joint_id, wrist_roll_joint_id]
    orient_joint_limits = robot.data.soft_joint_pos_limits[
        :, orient_joint_ids, :
    ].clone()
    ee_body_id = robot.body_names.index(EE_BODY_NAME)
    ee_jacobian_index = ee_body_id - 1

    # default_joint_pos 是之後 reset_episode() 恢復機器人時的基準。
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

    # 必須等機器人完成初始姿態穩定後才校準 eye-in-hand。
    # 若在上面 120 幀 settle 之前記錄 EE local offset，後續關節回到
    # default pose 時會把鏡頭帶進 wrist 外殼。
    if s8_gripper_camera is not None:
        _S8_GRIPPER_CAMERA = s8_gripper_camera
        _S8_EE_BODY_ID = ee_body_id
        _S8_TASK_OBJECT = task_object
        update_s8_gripper_camera(robot)
        s8_viewport_windows = create_s8_operator_viewports()

    # smoke test 模式只拍一張圖並退出，不進入控制主迴圈。
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

    # 保存方塊初始 pose/velocity，後續每個 episode 都回到完全相同的起點。
    object_initial_state = task_object.data.default_root_state.clone()
    reset_object_velocity = torch.zeros_like(object_initial_state[:, 7:])
    reset_object_staging_pose = object_initial_state[:, :7].clone()
    # Keep the cube away from the work cell while the position-driven robot is
    # teleported and settles.  Otherwise the reset impulse can hit a cube that
    # was just released at the goal and throw it away from the fixed start.
    reset_object_staging_pose[:, 0] = TABLE_POS_W[0] + 1.0
    reset_object_staging_pose[:, 1] = TABLE_POS_W[1] + 1.0
    reset_object_staging_pose[:, 2] = TABLE_TOP_Z_M + 0.5

    # replay 模式直接餵錄好的 joint target，不接受 WebSocket 命令。
    if args_cli.replay_hdf5:
        replay_hdf5_episode(
            sim,
            robot,
            task_object,
            sim_dt,
            gripper_joint_id,
        )
        return

    # -------------------------------------------------------------------------
    # [main 5/8] 建立 IK controller 與執行期狀態變數
    # -------------------------------------------------------------------------
    initial_ee_pos_b, _ = get_ee_pose_in_base(robot, ee_body_id)
    initial_ee_pos_b = initial_ee_pos_b.clone()
    print(
        "S4 BASE CONTROL READY "
        f"body={EE_BODY_NAME} frame=robot_base mapping=phone_axes_20261025_v1 "
        "control_modes=explicit_v2 default=translation "
        f"initial_ee_b={vector_list(initial_ee_pos_b)}",
        flush=True,
    )
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
    # position-driven 手臂在重力下可能有量測偏差；這個 offset 用來維持初始姿態。
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

    # full_joint_target 是每個 physics step 實際寫入 SO-101 的六維關節目標。
    # active 是目前唯一一筆正在執行的 Bridge request；None 代表閒置。
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
    orient_start_joint_pos = None
    orient_last_q_rel = [0.0, 0.0, 0.0, 1.0]
    orient_was_clamped = False
    pose_start_ee_pos_b = None
    pose_command_ee_pos_b = None
    pose_start_orient_joint_pos = None
    pose_start_gripper_pos = None
    pose_last_p_rel_m = [0.0, 0.0, 0.0]
    pose_last_q_rel = [0.0, 0.0, 0.0, 1.0]
    pose_last_gripper_delta = 0.0
    pose_wrist_clutch = PitchOnlyClutch(
        max_flex_deg=ORIENT_MAX_FLEX_DELTA_DEG,
    )
    pose_wrist_entry_arm_pos = None
    pose_was_workspace_clamped = False
    pose_was_joint_clamped = False
    pose_gripper_contact_count = 0
    pose_gripper_stalled = False
    pose_max_speed_m_s = POSE_MAX_SPEED_M_S
    episode_index = 0
    success_latch = StableSuccess(args_cli.success_stable_steps)
    success_display_remaining = 0
    self_test_countdown = 60 if args_cli.task_self_test else -1
    self_test_active = False

    # -------------------------------------------------------------------------
    # [main 6/8] 可選 HDF5 recorder 與每-step 錄製 hook
    # -------------------------------------------------------------------------
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
        control_params = {
            "rotation_gain": None,
            "rotation_deadband_deg": None,
            "rotation_smoothing": None,
            "step_mode": args_cli.record_step_mode,
        }
        if args_cli.record_teleop_mode != "buttons":
            control_params.update(
                {
                    "rotation_gain": args_cli.record_rotation_gain,
                    "rotation_deadband_deg": args_cli.record_rotation_deadband_deg,
                    "rotation_smoothing": args_cli.record_rotation_smoothing,
                }
            )
        if args_cli.record_teleop_mode == "webxr":
            control_params["position_gain"] = args_cli.record_position_gain
        source_devices = [
            {
                "device_id": "thomas-phone",
                "role": "command_source",
                "device_type": "phone",
                "software_version": args_cli.record_app_version,
                "clock_domain": "unix_epoch_ms",
            },
            {
                "device_id": "leo-mac-relay",
                "role": "bridge",
                "device_type": "mac-relay",
                "software_version": "mac-phone-relay-v1",
                "clock_domain": "monotonic_s",
            },
            {
                "device_id": args_cli.record_robot_id,
                "role": "robot",
                "device_type": "so101-sim",
                "software_version": "leisaac-0.4.0",
                "clock_domain": "sim_time_s",
            },
            {
                "device_id": "s3-front-camera",
                "role": "camera",
                "device_type": "isaac-sim-camera",
                "software_version": "unknown",
                "clock_domain": "sim_time_s",
            },
            {
                "device_id": "rtxws-recorder",
                "role": "recorder",
                "device_type": "server-container",
                "software_version": args_cli.record_recorder_version,
                "clock_domain": "sim_time_s",
            },
        ]
        s3_recorder = S3Recorder(
            output_path=args_cli.record_output,
            joint_names=robot.joint_names,
            image_shape=rgb_shape,
            task="S4 fixed red-cube pick and place",
            recording_metadata={
                "session_id": args_cli.record_session_id,
                "teleop_mode": args_cli.record_teleop_mode,
                "operator_id": args_cli.record_operator_id,
                "attempt_index": args_cli.record_attempt_index,
                "condition_order": args_cli.record_condition_order,
                "environment_kind": "simulation",
                "environment_name": "S4FixedPickPlace",
                "robot_instance_id": args_cli.record_robot_id,
                "control_params": control_params,
                "source_devices": source_devices,
                "software": {
                    "app_version": args_cli.record_app_version,
                    "recorder_version": args_cli.record_recorder_version,
                    "controller_version": args_cli.record_controller_version,
                    "protocol_version": args_cli.record_protocol_version,
                    "git_commit": args_cli.record_git_commit,
                },
            },
        )
        _S3_RECORDER = s3_recorder

    def s3_control_context():
        """把目前 active command 轉成 recorder 要保存的五維 operator intent。"""
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
        """在每個 physics step 前收集 observation/action；recorder 內部降採樣到 30 Hz。"""
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

    # -------------------------------------------------------------------------
    # [main 7/8] Episode reset：先移開方塊，再 reset 手臂，最後放回方塊。
    # 不能同時瞬移手臂與方塊，否則接觸衝量可能把方塊彈飛。
    # -------------------------------------------------------------------------
    def reset_episode(reason):
        """結束目前 episode，恢復固定初始狀態並重新開放命令。"""
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
    if args_cli.operator_multiview:
        print(
            "S8 operator views enabled: main + side + gripper-follow; "
            f"extra_viewports={len(s8_viewport_windows)}",
            flush=True,
        )
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

    # -------------------------------------------------------------------------
    # [main 8/8] 60 Hz 主迴圈
    # 每一圈只走一條主要分支：Estop、成功展示、取新命令，或執行 active 命令。
    # -------------------------------------------------------------------------
    while simulation_app.is_running():
        # [主迴圈 1] 先讀最新的 EE 與方塊狀態，提供 App state 與成功判定使用。
        current_ee_pos_b, current_ee_quat_b = get_ee_pose_in_base(robot, ee_body_id)
        object_state_w = task_object.data.root_state_w[0]
        object_pos_w = object_state_w[:3].detach().cpu().tolist()
        bridge.update_state(
            ee_pos=vector_list(current_ee_pos_b),
            object_pos=object_pos_w,
            goal_pos=list(GOAL_POS_W),
        )

        # [主迴圈 2] Estop 優先權最高：保持最後 actuator target，不再接受一般移動。
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

        # [主迴圈 3] task self-test 直接把方塊放到 goal，只測成功判定與 reset。
        # 這不能當作機器人真的完成抓取的證據。
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

        # [主迴圈 4] 成功後保留畫面 90 steps（預設 1.5 秒），再自動 reset。
        if success_display_remaining > 0:
            step_simulation(sim, robot, full_joint_target, sim_dt, (task_object,))
            success_display_remaining -= 1
            if success_display_remaining == 0:
                reset_episode("success")
            time.sleep(sim_dt)
            continue

        # [主迴圈 5] 每個 physics sample 都檢查：位置、速度、角速度、夾爪已開。
        # StableSuccess 只有在條件連續成立 30 steps 時才 latch 成功。
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
                bridge.clear_orient_stop()
                bridge.clear_pose_stop()
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

        # 方塊掉到桌面以下且目前沒有命令執行時，自動 reset；這不是成功。
        if active is None and object_pos_w[2] < FALLEN_OBJECT_Z_M:
            reset_episode("object_fallen")
            time.sleep(sim_dt)
            continue

        # [主迴圈 6] 閒置時才從 Bridge queue 取下一筆命令。
        # queue 容量為 1，因此系統不會堆積一長串過時動作。
        if active is None:
            request = bridge.take()
            if request is None:
                step_simulation(sim, robot, full_joint_target, sim_dt, (task_object,))
                time.sleep(sim_dt)
                continue
            if time.monotonic() > request["deadline"]:
                bridge.finish(request, "rejected", "expired_before_execution")
                continue

            # 取出後放進 active；後續每圈都持續執行，直到完成、停止或失敗。
            active = request
            command_kind = request["message"]["type"]
            payload = request["message"]["payload"]
            motion_step = 0
            stable_count = 0
            gripper_contact_count = 0
            gripper_jog_stalled = False
            motion_started_at = time.monotonic()
            # reset_scene 不需要 IK，收到後立刻執行完整 episode reset。
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
            # jog_start：按住期間以 m/s 連續更新 EE target，鬆開時由 jog_stop 結束。
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
            # move_delta：一次走固定距離；coarse=20 mm、fine=5 mm。
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
            # joint_jog_start／joint_delta：目前只允許直接調整 wrist_flex。
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
            # orient_start：手機相對旋轉只控制兩個 wrist joint。
            # 原本的 XYZ／手腕按鍵／夾爪分支保持不變。
            elif command_kind == "orient_start":
                orient_start_joint_pos = robot.data.joint_pos[
                    :, orient_joint_ids
                ].clone()
                full_joint_target[:, orient_joint_ids] = orient_start_joint_pos
                orient_last_q_rel = [0.0, 0.0, 0.0, 1.0]
                orient_was_clamped = False
                bridge.update_state(ik="orienting", orient_deg=0.0)
                print(
                    f"[S4 ORIENT START] seq={active['seq']} "
                    f"frame={payload['frame']} rate_hz={payload['rate_hz']} "
                    f"gain={payload['gain']:.3f} "
                    f"deadband_deg={payload['deadband_deg']:.3f} "
                    f"smoothing={payload['smoothing']:.3f} "
                    f"wrist_flex_deg="
                    f"{math.degrees(orient_start_joint_pos[0, 0].item()):.3f} "
                    f"wrist_roll_deg="
                    f"{math.degrees(orient_start_joint_pos[0, 1].item()):.3f}",
                    flush=True,
                )
            # pose_start：回到原始 base-frame 控制。WebXR 位移直接
            # 映射到 robot base XYZ，IK 追蹤 gripper link 原點（非 TCP offset）。
            elif command_kind == "pose_start":
                pose_wrist_clutch.reset()
                pose_wrist_entry_arm_pos = None
                pose_start_ee_pos_b = current_ee_pos_b.clone()
                pose_command_ee_pos_b = current_ee_pos_b.clone()
                pose_start_orient_joint_pos = robot.data.joint_pos[
                    :, orient_joint_ids
                ].clone()
                pose_start_gripper_pos = robot.data.joint_pos[
                    :, gripper_joint_id
                ].clone()
                held_gripper_target = pose_start_gripper_pos.clone()
                full_joint_target[:, orient_joint_ids] = pose_start_orient_joint_pos
                full_joint_target[:, gripper_joint_id] = held_gripper_target
                pose_last_p_rel_m = [0.0, 0.0, 0.0]
                pose_last_q_rel = [0.0, 0.0, 0.0, 1.0]
                pose_last_gripper_delta = 0.0
                pose_was_workspace_clamped = False
                pose_was_joint_clamped = False
                pose_gripper_contact_count = 0
                pose_gripper_stalled = False
                carrying = bridge.state_snapshot()["gripper"] == "close"
                pose_max_speed_m_s = (
                    POSE_CARRY_MAX_SPEED_M_S if carrying else POSE_MAX_SPEED_M_S
                )
                ik_controller.reset()
                bridge.update_state(
                    ik="pose_tracking",
                    orient_deg=0.0,
                    gripper_delta=0.0,
                )
                print(
                    f"[S4 POSE START] seq={active['seq']} "
                    f"frame={payload['frame']} rate_hz={payload['rate_hz']} "
                    f"position_gain={POSE_POSITION_GAIN:.2f} "
                    f"max_speed={pose_max_speed_m_s:.3f}m/s "
                    f"profile={'carry' if carrying else 'free'} "
                    "frame_mapping=webxr_to_robot_base "
                    f"control_mode={payload.get('control_mode', 'translation')} "
                    f"start_ee={vector_list(pose_start_ee_pos_b)}",
                    flush=True,
                )
            # gripper_jog_start／gripper：控制開合，關閉時包含接觸停止保護。
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
                "orient_start": "orient_watchdog_timeout",
                "pose_start": "pose_watchdog_timeout",
            }.get(command_kind, "execution_timeout")
            continuous_command = command_kind in {
                "jog_start",
                "joint_jog_start",
                "gripper_jog_start",
                "orient_start",
                "pose_start",
            }
            bridge.update_state(ik="stopped" if continuous_command else "no_solution")
            bridge.finish(active, "rejected", timeout_reason)
            if continuous_command:
                # A missed keepalive means "release the clutch": hold the
                # measured pose immediately, but do not latch the global
                # emergency stop.  Manual estop and non-finite math remain
                # hard faults.
                full_joint_target = robot.data.joint_pos.clone()
            else:
                bridge.engage_estop()
            print(
                f"[A3 {'HOLD STOP' if continuous_command else 'REJECT'}] "
                f"seq={active['seq']} reason={timeout_reason}",
                flush=True,
            )
            active = None
            continue

        # ---------------------------------------------------------------------
        # [active 執行 A] base-frame WebXR；每次按住明確選平移或 wrist-only。
        # ---------------------------------------------------------------------
        if command_kind == "pose_start":
            current_ee_pos_b, current_ee_quat_b = get_ee_pose_in_base(
                robot, ee_body_id
            )
            if bridge.pose_stop_event.is_set():
                bridge.clear_pose_stop()
                final_arm_joint_pos = robot.data.joint_pos[:, arm_joint_ids].clone()
                final_orient_joint_pos = robot.data.joint_pos[
                    :, orient_joint_ids
                ].clone()
                final_gripper_pos = robot.data.joint_pos[
                    :, gripper_joint_id
                ].clone()
                full_joint_target[:, arm_joint_ids] = final_arm_joint_pos
                full_joint_target[:, orient_joint_ids] = final_orient_joint_pos
                held_gripper_target = final_gripper_pos.clone()
                full_joint_target[:, gripper_joint_id] = held_gripper_target
                rotation_vector = quaternion_xyzw_to_rotation_vector(
                    pose_last_q_rel
                )
                details = {
                    "frame": payload["frame"],
                    "position_gain": POSE_POSITION_GAIN,
                    "coordinates": "gripper_link_in_robot_base",
                    "control_mode": payload.get("control_mode", "translation"),
                    "pitch_mode": "wrist_only" if pose_wrist_clutch.active else "translation",
                    "pitch_x_deg": math.degrees(rotation_vector[0]),
                    "wrist_flex_actual_deg": math.degrees(final_orient_joint_pos[0, 0].item()),
                    "arm_delta_since_wrist_entry_deg": (
                        torch.rad2deg(final_arm_joint_pos - pose_wrist_entry_arm_pos)[0].tolist()
                        if pose_wrist_entry_arm_pos is not None else None
                    ),
                    "webxr_delta_m": list(pose_last_p_rel_m),
                    "start_m": vector_list(pose_start_ee_pos_b),
                    "end_m": vector_list(current_ee_pos_b),
                    "actual_delta_m": vector_list(
                        current_ee_pos_b - pose_start_ee_pos_b
                    ),
                    "phone_rotation_deg": math.degrees(
                        math.sqrt(sum(value * value for value in rotation_vector))
                    ),
                    "gripper_delta": pose_last_gripper_delta,
                    "workspace_clamped": pose_was_workspace_clamped,
                    "joint_limit_clamped": pose_was_joint_clamped,
                    "sim_steps": motion_step,
                    "wall_time_s": time.monotonic() - motion_started_at,
                }
                bridge.update_state(
                    ik=(
                        "pose_clamped"
                        if pose_was_workspace_clamped or pose_was_joint_clamped
                        else "ok"
                    ),
                    ee_pos=vector_list(current_ee_pos_b),
                    orient_deg=details["phone_rotation_deg"],
                    gripper_delta=pose_last_gripper_delta,
                )
                bridge.finish(active, "ok", "pose_stopped", details)
                print(f"S4 POSE STOP seq={active['seq']}", details, flush=True)
                active = None
                time.sleep(sim_dt)
                continue

            pose = bridge.pose_snapshot(active)
            if pose is None:
                time.sleep(sim_dt)
                continue
            pose_last_p_rel_m = pose["p_rel_m"]
            pose_last_q_rel = pose["q_rel"]
            pose_last_gripper_delta = pose["gripper_delta"]

            # The UI chooses a mode once per press; never infer it from tilt.
            was_wrist_only = pose_wrist_clutch.active
            try:
                wrist_targets = pose_wrist_clutch.update(
                    pose_last_q_rel, full_joint_target[0].tolist(),
                    flex_index=orient_joint_ids[0],
                    start_flex=pose_start_orient_joint_pos[0, 0].item(),
                    flex_sign=ORIENT_FLEX_SIGN,
                    lower=orient_joint_limits[0, 0, 0].item(),
                    upper=orient_joint_limits[0, 0, 1].item(),
                    max_step=MAX_JOINT_DELTA_RAD,
                    control_mode=payload.get("control_mode", "translation"),
                )
            except ValueError as exc:
                bridge.finish(active, "rejected", "pose_wrist_invalid_input")
                bridge.engage_estop()
                full_joint_target = robot.data.joint_pos.clone()
                print(f"[S4 POSE REJECT] invalid wrist input: {exc}", flush=True)
                active = None
                continue
            if wrist_targets is not None:
                if not was_wrist_only:
                    pose_wrist_entry_arm_pos = robot.data.joint_pos[:, arm_joint_ids].clone()
                    print(
                        f"[S4 POSE WRIST-ONLY START] seq={active['seq']} "
                        f"pitch_x_deg={math.degrees(pose_wrist_clutch.pitch_rad):.2f} "
                        f"held_arm_targets_deg={torch.rad2deg(full_joint_target[:, arm_joint_ids])[0].tolist()}",
                        flush=True,
                    )
                full_joint_target = torch.tensor(
                    [wrist_targets], device=robot.device, dtype=full_joint_target.dtype
                )
                held_gripper_target = full_joint_target[:, gripper_joint_id].clone()
                pose_last_gripper_delta = 0.0  # Gripper is held in wrist-only mode.
                pose_was_joint_clamped |= pose_wrist_clutch.clamped
                step_simulation(sim, robot, full_joint_target, sim_dt, (task_object,))
                final_ee_pos_b, _ = get_ee_pose_in_base(robot, ee_body_id)
                if not (torch.isfinite(robot.data.joint_pos).all().item()
                        and torch.isfinite(final_ee_pos_b).all().item()):
                    bridge.finish(active, "rejected", "pose_wrist_nonfinite_state")
                    bridge.engage_estop()
                    active = None
                    continue
                motion_step += 1
                bridge.update_state(
                    ik="orient_clamped" if pose_wrist_clutch.clamped else "orienting",
                    ee_pos=vector_list(final_ee_pos_b),
                    orient_deg=math.degrees(pose_wrist_clutch.pitch_rad),
                    gripper_delta=0.0,
                )
                if motion_step % 30 == 0:
                    arm_drift = torch.rad2deg(
                        robot.data.joint_pos[:, arm_joint_ids] - pose_wrist_entry_arm_pos
                    )[0].tolist()
                    print(
                        f"[S4 POSE WRIST-ONLY] seq={active['seq']} step={motion_step} "
                        f"pitch_x_deg={math.degrees(pose_wrist_clutch.pitch_rad):.2f} "
                        f"wrist_flex_target_deg={math.degrees(wrist_targets[orient_joint_ids[0]]):.2f} "
                        f"wrist_flex_actual_deg={math.degrees(robot.data.joint_pos[0, orient_joint_ids[0]].item()):.2f} "
                        f"arm_actual_delta_deg={arm_drift} translation_ignored=True",
                        flush=True,
                    )
                time.sleep(sim_dt)
                continue  # Never run translation IK for this latched gesture.

            mapped_delta_b = map_webxr_translation_to_base(
                pose_last_p_rel_m,
                POSE_POSITION_GAIN,
            )
            requested_pose_target = pose_start_ee_pos_b + torch.tensor(
                [mapped_delta_b],
                device=robot.device,
                dtype=pose_start_ee_pos_b.dtype,
            )
            workspace_lower = initial_ee_pos_b - WORKSPACE_RADIUS_M
            workspace_upper = initial_ee_pos_b + WORKSPACE_RADIUS_M
            desired_pose_target = torch.clamp(
                requested_pose_target,
                min=workspace_lower,
                max=workspace_upper,
            )
            pose_workspace_clamped_now = not torch.allclose(
                desired_pose_target,
                requested_pose_target,
                atol=1.0e-7,
            )
            pose_was_workspace_clamped = (
                pose_was_workspace_clamped or pose_workspace_clamped_now
            )

            target_delta = desired_pose_target - pose_command_ee_pos_b
            target_delta_norm = torch.linalg.vector_norm(target_delta, dim=1)[0]
            max_position_step_m = pose_max_speed_m_s * sim_dt
            if target_delta_norm.item() > max_position_step_m:
                target_delta = target_delta * (
                    max_position_step_m / target_delta_norm.item()
                )
            pose_command_ee_pos_b = pose_command_ee_pos_b + target_delta

            rotation_vector = quaternion_xyzw_to_rotation_vector(
                pose_last_q_rel
            )
            flex_delta_rad = 0.0  # Translation mode ignores all phone rotation.
            roll_delta_rad = 0.0  # Translation mode holds both wrist targets.
            requested_orient_target = pose_start_orient_joint_pos + torch.tensor(
                [[flex_delta_rad, roll_delta_rad]],
                device=robot.device,
                dtype=pose_start_orient_joint_pos.dtype,
            )
            desired_orient_target = torch.clamp(
                requested_orient_target,
                min=orient_joint_limits[:, :, 0],
                max=orient_joint_limits[:, :, 1],
            )
            pose_joint_clamped_now = not torch.allclose(
                desired_orient_target,
                requested_orient_target,
                atol=1.0e-7,
            )
            pose_was_joint_clamped = (
                pose_was_joint_clamped or pose_joint_clamped_now
            )
            orient_target_step = torch.clamp(
                desired_orient_target - full_joint_target[:, orient_joint_ids],
                min=-MAX_JOINT_DELTA_RAD,
                max=MAX_JOINT_DELTA_RAD,
            )
            full_joint_target[:, orient_joint_ids] += orient_target_step

            gripper_range = (
                gripper_joint_limits[:, 1] - gripper_joint_limits[:, 0]
            )
            requested_gripper_target = (
                pose_start_gripper_pos
                + pose_last_gripper_delta * gripper_range
            )
            desired_gripper_target = torch.clamp(
                requested_gripper_target,
                min=gripper_joint_limits[:, 0],
                max=gripper_joint_limits[:, 1],
            )
            gripper_opening = (
                desired_gripper_target[0].item()
                >= held_gripper_target[0].item()
            )
            gripper_speed_deg_s = (
                GRIPPER_JOG_SPEED_DEG_S["fine"]
                if gripper_opening
                else GRIPPER_JOG_CLOSE_SPEED_DEG_S["fine"]
            )
            max_gripper_step_rad = math.radians(gripper_speed_deg_s) * sim_dt
            held_gripper_target += torch.clamp(
                desired_gripper_target - held_gripper_target,
                min=-max_gripper_step_rad,
                max=max_gripper_step_rad,
            )

            ik_controller.set_command(
                pose_command_ee_pos_b,
                ee_quat=current_ee_quat_b,
            )
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
                bridge.finish(active, "rejected", "pose_nonfinite_ik")
                bridge.engage_estop()
                full_joint_target = robot.data.joint_pos.clone()
                print(
                    f"[S4 POSE REJECT] seq={active['seq']} "
                    "reason=pose_nonfinite_ik",
                    flush=True,
                )
                active = None
                continue

            raw_joint_delta = desired_arm_joint_pos - current_arm_joint_pos
            limited_joint_delta = torch.clamp(
                raw_joint_delta,
                min=-POSE_MAX_JOINT_DELTA_RAD,
                max=POSE_MAX_JOINT_DELTA_RAD,
            )
            arm_joint_target = (
                current_arm_joint_pos + gravity_hold_offset + limited_joint_delta
            )
            arm_joint_target = torch.clamp(
                arm_joint_target,
                min=arm_joint_limits[:, :, 0],
                max=arm_joint_limits[:, :, 1],
            )
            full_joint_target[:, arm_joint_ids] = arm_joint_target
            full_joint_target[:, gripper_joint_id] = held_gripper_target
            pose_step_started_at = time.monotonic()
            step_simulation(sim, robot, full_joint_target, sim_dt, (task_object,))

            final_ee_pos_b, _ = get_ee_pose_in_base(robot, ee_body_id)
            command_error_m = torch.linalg.vector_norm(
                pose_command_ee_pos_b - final_ee_pos_b,
                dim=1,
            )[0].item()
            if (
                not math.isfinite(command_error_m)
                or command_error_m > SAFETY_COMMAND_ERROR_M
            ):
                bridge.update_state(ik="no_solution")
                bridge.finish(active, "rejected", "pose_tracking_safety_stop")
                bridge.engage_estop()
                full_joint_target = robot.data.joint_pos.clone()
                print(
                    f"[S4 POSE REJECT] seq={active['seq']} "
                    "reason=pose_tracking_safety_stop "
                    f"tracking_error_m={command_error_m}",
                    flush=True,
                )
                active = None
                continue

            motion_step += 1
            orient_deg = math.degrees(
                math.sqrt(sum(value * value for value in rotation_vector))
            )
            if pose_last_gripper_delta > 0.02:
                pose_gripper_state = "open"
            elif pose_last_gripper_delta < -0.02:
                pose_gripper_state = "close"
            else:
                pose_gripper_state = bridge.state_snapshot()["gripper"]
            bridge.update_state(
                ee_pos=vector_list(final_ee_pos_b),
                gripper=pose_gripper_state,
                ik=(
                    "pose_clamped"
                    if pose_workspace_clamped_now or pose_joint_clamped_now
                    else "pose_tracking"
                ),
                orient_deg=orient_deg,
                gripper_delta=pose_last_gripper_delta,
            )
            if motion_step % 30 == 0:
                print(
                    f"[S4 POSE] seq={active['seq']} step={motion_step} "
                    f"webxr={pose_last_p_rel_m} "
                    f"target={vector_list(pose_command_ee_pos_b)} "
                    f"actual={vector_list(final_ee_pos_b)} "
                    f"phone_deg={orient_deg:.2f} "
                    f"gripper_delta={pose_last_gripper_delta:+.3f}",
                    flush=True,
                )
            pose_step_remaining_s = sim_dt - (
                time.monotonic() - pose_step_started_at
            )
            if pose_step_remaining_s > 0.0:
                time.sleep(pose_step_remaining_s)
            continue

        # ---------------------------------------------------------------------
        # [active 執行 B] 手機姿態控制 wrist_flex 與 wrist_roll
        # ---------------------------------------------------------------------
        if command_kind == "orient_start":
            if bridge.orient_stop_event.is_set():
                bridge.clear_orient_stop()
                final_orient_joint_pos = robot.data.joint_pos[
                    :, orient_joint_ids
                ].clone()
                full_joint_target[:, orient_joint_ids] = final_orient_joint_pos
                rotation_vector = quaternion_xyzw_to_rotation_vector(
                    orient_last_q_rel
                )
                details = {
                    "frame": payload["frame"],
                    "start_wrist_flex_deg": math.degrees(
                        orient_start_joint_pos[0, 0].item()
                    ),
                    "end_wrist_flex_deg": math.degrees(
                        final_orient_joint_pos[0, 0].item()
                    ),
                    "start_wrist_roll_deg": math.degrees(
                        orient_start_joint_pos[0, 1].item()
                    ),
                    "end_wrist_roll_deg": math.degrees(
                        final_orient_joint_pos[0, 1].item()
                    ),
                    "phone_rotation_deg": math.degrees(
                        math.sqrt(sum(value * value for value in rotation_vector))
                    ),
                    "joint_limit_clamped": orient_was_clamped,
                    "sim_steps": motion_step,
                    "wall_time_s": time.monotonic() - motion_started_at,
                }
                bridge.update_state(
                    ik="orient_clamped" if orient_was_clamped else "ok",
                    orient_deg=details["phone_rotation_deg"],
                )
                bridge.finish(active, "ok", "orient_stopped", details)
                print(
                    f"S4 ORIENT STOP seq={active['seq']}",
                    details,
                    flush=True,
                )
                active = None
                time.sleep(sim_dt)
                continue

            latest_q_rel = bridge.orient_snapshot(active)
            if latest_q_rel is None:
                time.sleep(sim_dt)
                continue
            orient_last_q_rel = latest_q_rel
            rotation_vector = quaternion_xyzw_to_rotation_vector(latest_q_rel)
            flex_delta_rad = max(
                -math.radians(ORIENT_MAX_FLEX_DELTA_DEG),
                min(
                    math.radians(ORIENT_MAX_FLEX_DELTA_DEG),
                    ORIENT_FLEX_SIGN * rotation_vector[0],
                ),
            )
            roll_delta_rad = max(
                -math.radians(ORIENT_MAX_ROLL_DELTA_DEG),
                min(
                    math.radians(ORIENT_MAX_ROLL_DELTA_DEG),
                    ORIENT_ROLL_SIGN * rotation_vector[1],
                ),
            )
            requested_orient_target = orient_start_joint_pos + torch.tensor(
                [[flex_delta_rad, roll_delta_rad]],
                device=robot.device,
                dtype=orient_start_joint_pos.dtype,
            )
            desired_orient_target = torch.clamp(
                requested_orient_target,
                min=orient_joint_limits[:, :, 0],
                max=orient_joint_limits[:, :, 1],
            )
            orient_is_clamped = not torch.allclose(
                desired_orient_target,
                requested_orient_target,
                atol=1.0e-7,
            )
            orient_was_clamped = orient_was_clamped or orient_is_clamped
            # 手機單幀旋轉即使很大，actuator target 也只能緩慢追上。
            target_step = torch.clamp(
                desired_orient_target - full_joint_target[:, orient_joint_ids],
                min=-MAX_JOINT_DELTA_RAD,
                max=MAX_JOINT_DELTA_RAD,
            )
            full_joint_target[:, orient_joint_ids] += target_step
            full_joint_target[:, gripper_joint_id] = held_gripper_target
            step_simulation(sim, robot, full_joint_target, sim_dt, (task_object,))
            motion_step += 1
            orient_deg = math.degrees(
                math.sqrt(sum(value * value for value in rotation_vector))
            )
            bridge.update_state(
                ik="orient_clamped" if orient_is_clamped else "orienting",
                orient_deg=orient_deg,
            )
            if motion_step % 30 == 0:
                current_orient_joint_pos = robot.data.joint_pos[
                    :, orient_joint_ids
                ]
                print(
                    f"[S4 ORIENT] seq={active['seq']} step={motion_step} "
                    f"phone_deg={orient_deg:.2f} "
                    f"wrist_flex_deg="
                    f"{math.degrees(current_orient_joint_pos[0, 0].item()):.2f} "
                    f"wrist_roll_deg="
                    f"{math.degrees(current_orient_joint_pos[0, 1].item()):.2f}",
                    flush=True,
                )
            time.sleep(sim_dt)
            continue

        # ---------------------------------------------------------------------
        # [active 執行 B] wrist_flex 連續 jog
        # 每個 step 依 deg/s 增加 target，直到 App 送 joint_jog_stop。
        # ---------------------------------------------------------------------
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

        # ---------------------------------------------------------------------
        # [active 執行 B] wrist_flex 固定角度移動
        # 使用線性插值走到目標，並要求誤差連續數步在容差內。
        # ---------------------------------------------------------------------
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

        # ---------------------------------------------------------------------
        # [active 執行 C] 夾爪連續 jog
        # 關閉時若 tracking error 持續變大，推測已接觸方塊並停止繼續硬夾。
        # ---------------------------------------------------------------------
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

        # ---------------------------------------------------------------------
        # [active 執行 D] 夾爪一次性 open/close
        # 逐步插值到 0° 或 100°；close 遇到物體時可提前 contact stop。
        # ---------------------------------------------------------------------
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
                # 目標仍在關閉、實際手指卻跟不上，代表碰到阻力。
                # 不再追逐 hard-close，只保留極小 preload 以維持抓持。
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

        # ---------------------------------------------------------------------
        # [active 執行 E] Cartesian jog／move_delta 共用的 Differential IK 路徑
        # 上面四種 joint/gripper 命令都已 continue，因此走到這裡的只剩
        # jog_start 或 move_delta。
        # ---------------------------------------------------------------------
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

        # IK 輸入是 base frame 下的目標位置；orientation 保持目前值不變。
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

        # IK 輸出先做每-step 變化限制，再做 soft joint limit，最後才寫入 simulator。
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
        # 這是即時安全誤差，不是 move_delta 的最終驗收誤差。
        # 超過 10 cm 代表控制已嚴重偏離，立即 Estop。
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

        # move_delta 到達目標後還要連續穩定 MOVE_STABLE_STEPS，才回傳 completed。
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


# 只有直接執行這個檔案時才呼叫 main()；若被其他測試 import，不會自動跑模擬。
if __name__ == "__main__":
    try:
        main() # 479
    except KeyboardInterrupt:
        print("A3 move_delta stopped by user.", flush=True)
    finally:
        # 不論正常結束、Ctrl-C 或例外，都關閉 recorder、Bridge 與 Isaac Sim。
        _S3_BEFORE_STEP = None
        _S3_CAMERA = None
        _S8_GRIPPER_CAMERA = None
        _S8_EE_BODY_ID = None
        _S8_TASK_OBJECT = None
        if _S3_RECORDER is not None:
            _S3_RECORDER.close()
            _S3_RECORDER = None
        bridge.close()
        simulation_app.close()
