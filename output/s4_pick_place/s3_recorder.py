#!/usr/bin/env python3
"""Small 30 Hz HDF5 recorder for the direct-workflow S-4 simulation."""

import json
from pathlib import Path

import h5py
import numpy as np


SCHEMA_VERSION = "s3-v1"
FPS = 30
PHYSICS_STEPS_PER_SAMPLE = 2


class S3Recorder:
    """Write one temporary episode and publish it only after task success."""

    def __init__(self, output_path, joint_names, image_shape, task):
        self.output_path = Path(output_path)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.joint_names = list(joint_names)
        self.image_shape = tuple(image_shape)
        self.task = task
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
        self.stream.flush()
        return count

    def close(self):
        if self.stream is None:
            return
        self.discard("simulation_closed")
        self.stream.close()
        self.stream = None
