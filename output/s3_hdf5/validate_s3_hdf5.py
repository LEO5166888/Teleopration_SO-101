#!/usr/bin/env python3
"""Validate the synchronized S-3 HDF5 draft contract."""

import argparse
import json
import math
from pathlib import Path

import h5py
import numpy as np


REQUIRED_DATASETS = {
    "timestamps/sim_time_s": (1, np.floating),
    "timestamps/frame_index": (1, np.integer),
    "timestamps/source_command_ts_ms": (1, np.integer),
    "timestamps/source_command_seq": (1, np.integer),
    "obs/joint_pos": (2, np.floating),
    "obs/joint_vel": (2, np.floating),
    "obs/ee_pose": (2, np.floating),
    "obs/front_rgb": (4, np.integer),
    "actions/operator_delta": (2, np.floating),
    "actions/joint_target": (2, np.floating),
    "status/estop": (1, np.bool_),
    "status/command_type": (1, np.integer),
    "status/step_mode": (1, np.integer),
}


def fail(message):
    raise ValueError(message)


def finite(name, values):
    if not np.isfinite(values).all():
        fail(f"{name}: contains NaN or Inf")


def validate_episode(name, episode):
    missing_attrs = {
        "num_samples",
        "success",
        "schema_version",
        "fps",
        "task",
        "joint_names",
        "ee_frame",
        "quaternion_order",
    } - set(episode.attrs)
    if missing_attrs:
        fail(f"{name}: missing attrs {sorted(missing_attrs)}")

    sample_count = int(episode.attrs["num_samples"])
    if sample_count < 2:
        fail(f"{name}: num_samples must be at least 2")
    if not bool(episode.attrs["success"]):
        fail(f"{name}: success must be true for an accepted demonstration")

    fps = float(episode.attrs["fps"])
    if not math.isfinite(fps) or fps <= 0:
        fail(f"{name}: fps must be finite and positive")

    arrays = {}
    for path, (rank, dtype_class) in REQUIRED_DATASETS.items():
        if path not in episode:
            fail(f"{name}: missing dataset {path}")
        dataset = episode[path]
        if dataset.ndim != rank:
            fail(f"{name}/{path}: expected rank {rank}, got {dataset.ndim}")
        if dataset.shape[0] != sample_count:
            fail(
                f"{name}/{path}: first dimension {dataset.shape[0]} "
                f"!= num_samples {sample_count}"
            )
        if not np.issubdtype(dataset.dtype, dtype_class):
            fail(f"{name}/{path}: unexpected dtype {dataset.dtype}")
        arrays[path] = dataset[...]

    expected_shapes = {
        "obs/joint_pos": (sample_count, 6),
        "obs/joint_vel": (sample_count, 6),
        "obs/ee_pose": (sample_count, 7),
        "actions/operator_delta": (sample_count, 5),
        "actions/joint_target": (sample_count, 6),
    }
    for path, expected in expected_shapes.items():
        if arrays[path].shape != expected:
            fail(f"{name}/{path}: expected shape {expected}, got {arrays[path].shape}")

    rgb = arrays["obs/front_rgb"]
    if rgb.shape[-1] != 3 or rgb.dtype != np.uint8:
        fail(f"{name}/obs/front_rgb: expected uint8 [N,H,W,3], got {rgb.dtype} {rgb.shape}")
    if rgb.shape[1] < 2 or rgb.shape[2] < 2:
        fail(f"{name}/obs/front_rgb: image resolution is invalid")
    if np.all(rgb == 0) or np.all(rgb == 255):
        fail(f"{name}/obs/front_rgb: every image is uniformly black or white")

    for path in (
        "timestamps/sim_time_s",
        "obs/joint_pos",
        "obs/joint_vel",
        "obs/ee_pose",
        "actions/operator_delta",
        "actions/joint_target",
    ):
        finite(f"{name}/{path}", arrays[path])

    sim_time = arrays["timestamps/sim_time_s"]
    if not np.all(np.diff(sim_time) > 0):
        fail(f"{name}/timestamps/sim_time_s: timestamps are not strictly increasing")

    frame_index = arrays["timestamps/frame_index"]
    if not np.array_equal(frame_index, np.arange(sample_count, dtype=frame_index.dtype)):
        fail(f"{name}/timestamps/frame_index: expected consecutive 0..N-1")

    print(
        f"PASS {name}: samples={sample_count}, fps={fps:g}, "
        f"rgb={rgb.shape[2]}x{rgb.shape[1]}"
    )
    return sample_count


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("file", type=Path)
    args = parser.parse_args()

    if not args.file.is_file():
        fail(f"file not found: {args.file}")

    with h5py.File(args.file, "r") as stream:
        if "data" not in stream:
            fail("missing root group: data")
        data = stream["data"]
        episode_names = sorted(name for name in data if name.startswith("demo_"))
        if not episode_names:
            fail("no data/demo_N episodes found")

        total = sum(validate_episode(name, data[name]) for name in episode_names)
        if "total" in data.attrs and int(data.attrs["total"]) != total:
            fail(f"data.attrs['total']={data.attrs['total']} != validated total {total}")

        env_args = data.attrs.get("env_args")
        if env_args is not None:
            try:
                json.loads(env_args)
            except (TypeError, json.JSONDecodeError) as exc:
                fail(f"data.attrs['env_args'] is not valid JSON: {exc}")

    print(f"S3 HDF5 VALIDATION PASS: episodes={len(episode_names)}, total={total}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError) as exc:
        print(f"S3 HDF5 VALIDATION FAIL: {exc}")
        raise SystemExit(1)
