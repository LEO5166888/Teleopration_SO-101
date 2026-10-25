#!/usr/bin/env python3
"""Small 30 Hz HDF5 recorder for the direct-workflow S-4 simulation."""

import json
from datetime import datetime, timezone
from pathlib import Path

import h5py
import numpy as np


SCHEMA_VERSION = "s3-v1"
FPS = 30
PHYSICS_STEPS_PER_SAMPLE = 2
D5_RECORDING_SPEC_VERSION = "d5-v1"
D5_JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


class S3Recorder:
    """Write one temporary episode and publish it only after task success."""

    def __init__(
        self,
        output_path,
        joint_names,
        image_shape,
        task,
        *,
        task_id="s4-fixed-pick-place-v1",
        recording_metadata=None,
    ):
        self.output_path = Path(output_path)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.joint_names = list(joint_names)
        self.image_shape = tuple(image_shape)
        self.task = task
        self.task_id = task_id
        self.recording_metadata = dict(recording_metadata or {})
        if self.recording_metadata and tuple(self.joint_names) != D5_JOINT_NAMES:
            raise ValueError(
                "D-5 joint order mismatch: "
                f"expected {D5_JOINT_NAMES}, got {tuple(self.joint_names)}"
            )
        self._validate_recording_metadata()
        self.stream = h5py.File(self.output_path, "a")
        self.data = self.stream.require_group("data")
        self.data.attrs["env_args"] = json.dumps(
            {
                "env_name": "S4FixedPickPlace",
                "type": "IsaacLab direct workflow",
                "recording_fps": FPS,
            },
            separators=(",", ":"),
        )
        if "_tmp_episode" in self.data:
            del self.data["_tmp_episode"]
        self.episode = None
        self.sample_count = 0
        self.physics_phase = 0
        self.capture_started = False
        self.source_episode = None
        self.created_at_utc = None

    def _validate_recording_metadata(self):
        if not self.recording_metadata:
            return
        required = {
            "session_id",
            "teleop_mode",
            "operator_id",
            "attempt_index",
            "condition_order",
            "environment_kind",
            "environment_name",
            "robot_instance_id",
            "control_params",
            "source_devices",
            "software",
        }
        missing = sorted(required - set(self.recording_metadata))
        if missing:
            raise ValueError(f"D-5 recording metadata missing fields: {missing}")
        if self.recording_metadata["teleop_mode"] not in {"buttons", "imu", "webxr"}:
            raise ValueError("teleop_mode must be buttons, imu, or webxr")
        if int(self.recording_metadata["attempt_index"]) < 1:
            raise ValueError("attempt_index must be >= 1")
        if int(self.recording_metadata["condition_order"]) < 1:
            raise ValueError("condition_order must be >= 1")
        controls = self.recording_metadata["control_params"]
        if self.recording_metadata["teleop_mode"] == "buttons":
            for field in (
                "rotation_gain",
                "rotation_deadband_deg",
                "rotation_smoothing",
            ):
                if controls.get(field) is not None:
                    raise ValueError(f"buttons requires control_params.{field}=None")
        if controls.get("step_mode") not in {"fine", "coarse"}:
            raise ValueError("control_params.step_mode must be fine or coarse")

    @property
    def active(self):
        return self.episode is not None

    def start(self, source_episode):
        if self.active:
            self.discard("restarted")
        episode = self.data.create_group("_tmp_episode")
        episode.attrs["num_samples"] = 0
        episode.attrs["success"] = False
        episode.attrs["schema_version"] = SCHEMA_VERSION
        episode.attrs["fps"] = FPS
        episode.attrs["task"] = self.task
        episode.attrs["joint_names"] = json.dumps(self.joint_names)
        episode.attrs["ee_frame"] = "base"
        episode.attrs["quaternion_order"] = "wxyz"
        episode.attrs["source_episode"] = int(source_episode)
        self.source_episode = int(source_episode)
        self.created_at_utc = datetime.now(timezone.utc).isoformat().replace(
            "+00:00", "Z"
        )
        if self.recording_metadata:
            self._write_d5_flat_attrs(episode)

        timestamps = episode.create_group("timestamps")
        observations = episode.create_group("obs")
        actions = episode.create_group("actions")
        status = episode.create_group("status")

        self._dataset(timestamps, "sim_time_s", (), np.float64)
        self._dataset(timestamps, "frame_index", (), np.int64)
        self._dataset(timestamps, "source_command_ts_ms", (), np.int64)
        self._dataset(timestamps, "source_command_seq", (), np.int64)
        self._dataset(observations, "joint_pos", (6,), np.float32)
        self._dataset(observations, "joint_vel", (6,), np.float32)
        self._dataset(observations, "ee_pose", (7,), np.float32)
        observations.create_dataset(
            "front_rgb",
            shape=(0, *self.image_shape),
            maxshape=(None, *self.image_shape),
            chunks=(1, *self.image_shape),
            dtype=np.uint8,
            compression="lzf",
        )
        self._dataset(actions, "operator_delta", (5,), np.float32)
        self._dataset(actions, "joint_target", (6,), np.float32)
        self._dataset(status, "estop", (), np.bool_)
        self._dataset(status, "command_type", (), np.int8)
        self._dataset(status, "step_mode", (), np.int8)
        self.episode = episode
        self.sample_count = 0
        self.physics_phase = 0
        self.capture_started = False
        self.stream.flush()

    def _write_d5_flat_attrs(self, episode):
        metadata = self.recording_metadata
        episode.attrs["recording_spec_version"] = D5_RECORDING_SPEC_VERSION
        episode.attrs["session_id"] = metadata["session_id"]
        episode.attrs["episode_id"] = self.source_episode
        episode.attrs["created_at_utc"] = self.created_at_utc
        episode.attrs["teleop_mode"] = metadata["teleop_mode"]
        episode.attrs["operator_id"] = metadata["operator_id"]
        episode.attrs["attempt_index"] = int(metadata["attempt_index"]) + self.source_episode
        episode.attrs["condition_order"] = int(metadata["condition_order"])
        episode.attrs["task_id"] = self.task_id
        episode.attrs["environment"] = metadata["environment_kind"]
        episode.attrs["robot_id"] = metadata["robot_instance_id"]
        episode.attrs["control_params"] = json.dumps(
            metadata["control_params"],
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        for field, value in metadata["software"].items():
            episode.attrs[field] = value

    @staticmethod
    def _dataset(group, name, tail_shape, dtype):
        return group.create_dataset(
            name,
            shape=(0, *tail_shape),
            maxshape=(None, *tail_shape),
            chunks=(256, *tail_shape),
            dtype=dtype,
        )

    @staticmethod
    def _append(dataset, index, value):
        dataset.resize(index + 1, axis=0)
        dataset[index] = value

    def before_physics_step(
        self,
        *,
        joint_pos,
        joint_vel,
        ee_pose,
        front_rgb,
        operator_delta,
        joint_target,
        source_command_ts_ms,
        source_command_seq,
        estop,
        command_type,
        step_mode,
    ):
        if not self.active:
            return False

        operator_delta = np.asarray(operator_delta, dtype=np.float32)
        capture_started_now = False
        if not self.capture_started:
            # Arm at scene reset, but make the first effective operator command
            # frame zero. This removes arbitrary operator setup time while still
            # preserving every pause after the demonstration has begun.
            if not np.any(np.abs(operator_delta) > 1.0e-8):
                return False
            self.capture_started = True
            self.physics_phase = 0
            capture_started_now = True

        should_record = self.physics_phase % PHYSICS_STEPS_PER_SAMPLE == 0
        self.physics_phase += 1
        if not should_record:
            return capture_started_now

        index = self.sample_count
        values = {
            "timestamps/sim_time_s": np.float64(index / FPS),
            "timestamps/frame_index": np.int64(index),
            "timestamps/source_command_ts_ms": np.int64(source_command_ts_ms),
            "timestamps/source_command_seq": np.int64(source_command_seq),
            "obs/joint_pos": np.asarray(joint_pos, dtype=np.float32),
            "obs/joint_vel": np.asarray(joint_vel, dtype=np.float32),
            "obs/ee_pose": np.asarray(ee_pose, dtype=np.float32),
            "obs/front_rgb": np.asarray(front_rgb, dtype=np.uint8),
            "actions/operator_delta": operator_delta,
            "actions/joint_target": np.asarray(joint_target, dtype=np.float32),
            "status/estop": np.bool_(estop),
            "status/command_type": np.int8(command_type),
            "status/step_mode": np.int8(step_mode),
        }
        for path, value in values.items():
            self._append(self.episode[path], index, value)
        self.sample_count += 1
        self.episode.attrs.modify("num_samples", self.sample_count)
        return capture_started_now

    def finish_success(self):
        if not self.active:
            return None
        if self.sample_count < 2:
            self.discard("too_few_samples")
            return None
        self.episode.attrs.modify("num_samples", self.sample_count)
        self.episode.attrs.modify("success", True)
        self.episode.attrs["termination_reason"] = "task_success"
        if self.recording_metadata:
            self.episode.attrs["d5_metadata_json"] = self._build_d5_metadata_json()
        demo_index = self._next_demo_index()
        demo_name = f"demo_{demo_index}"
        self.data.move("_tmp_episode", demo_name)
        self.episode = None
        self.data.attrs["total"] = sum(
            int(group.attrs["num_samples"])
            for name, group in self.data.items()
            if name.startswith("demo_")
        )
        self.stream.flush()
        return demo_name, self.sample_count

    def _build_d5_metadata_json(self):
        metadata = self.recording_metadata
        record = {
            "schema_version": D5_RECORDING_SPEC_VERSION,
            "session_id": metadata["session_id"],
            "episode_id": self.source_episode,
            "created_at_utc": self.created_at_utc,
            "teleop_mode": metadata["teleop_mode"],
            "task": {"id": self.task_id, "instruction": self.task},
            "recording": {
                "fps": FPS,
                "primary_clock": "sim_time_s",
                "num_frames": self.sample_count,
                "nominal_duration_s": self.sample_count / FPS,
            },
            "environment": {
                "kind": metadata["environment_kind"],
                "name": metadata["environment_name"],
            },
            "robot": {
                "model": "so101",
                "instance_id": metadata["robot_instance_id"],
                "joint_names": self.joint_names,
                "ee_frame": "base",
                "quaternion_order": "wxyz",
            },
            "operator": {
                "operator_id": metadata["operator_id"],
                "attempt_index": int(metadata["attempt_index"]) + self.source_episode,
                "condition_order": int(metadata["condition_order"]),
            },
            "control_params": metadata["control_params"],
            "source_devices": metadata["source_devices"],
            "software": metadata["software"],
            "outcome": {
                "success": True,
                "termination_reason": "task_success",
            },
        }
        return json.dumps(
            record,
            ensure_ascii=False,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        )

    def _next_demo_index(self):
        indices = []
        for name in self.data:
            if name.startswith("demo_"):
                try:
                    indices.append(int(name.removeprefix("demo_")))
                except ValueError:
                    continue
        return max(indices, default=-1) + 1

    def discard(self, reason):
        if not self.active:
            return 0
        count = self.sample_count
        self.episode.attrs["discard_reason"] = reason
        del self.data["_tmp_episode"]
        self.episode = None
        self.sample_count = 0
        self.physics_phase = 0
        self.capture_started = False
        self.source_episode = None
        self.created_at_utc = None
        self.stream.flush()
        return count

    def close(self):
        if self.stream is None:
            return
        self.discard("simulation_closed")
        self.stream.close()
        self.stream = None
