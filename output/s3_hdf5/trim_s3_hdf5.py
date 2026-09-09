#!/usr/bin/env python3
"""Copy one successful S-3 episode while removing leading operator idle time."""

import argparse
from pathlib import Path

import h5py
import numpy as np


def copy_attrs(source, destination):
    for key, value in source.attrs.items():
        destination.attrs[key] = value


def copy_dataset(source, destination_group, name, start, sample_count):
    new_shape = (sample_count, *source.shape[1:])
    options = {}
    if source.chunks is not None:
        options["chunks"] = source.chunks
    if source.compression is not None:
        options["compression"] = source.compression
        options["compression_opts"] = source.compression_opts
    if source.shuffle:
        options["shuffle"] = True
    if source.fletcher32:
        options["fletcher32"] = True

    destination = destination_group.create_dataset(
        name,
        shape=new_shape,
        dtype=source.dtype,
        **options,
    )
    copy_attrs(source, destination)
    block_size = 32 if source.ndim >= 4 else 1024
    for output_start in range(0, sample_count, block_size):
        output_stop = min(output_start + block_size, sample_count)
        source_start = start + output_start
        source_stop = start + output_stop
        destination[output_start:output_stop] = source[source_start:source_stop]
    return destination


def copy_group(source, destination, start, sample_count, original_count):
    copy_attrs(source, destination)
    for name, item in source.items():
        if isinstance(item, h5py.Group):
            child = destination.create_group(name)
            copy_group(item, child, start, sample_count, original_count)
        elif item.ndim >= 1 and item.shape[0] == original_count:
            copy_dataset(item, destination, name, start, sample_count)
        else:
            source.file.copy(item, destination, name=name)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--episode", default="demo_0")
    parser.add_argument("--pre-roll-samples", type=int, default=15)
    args = parser.parse_args()

    if args.pre_roll_samples < 0:
        raise ValueError("--pre-roll-samples must be non-negative")
    if args.destination.exists():
        raise FileExistsError(f"destination already exists: {args.destination}")
    args.destination.parent.mkdir(parents=True, exist_ok=True)

    with h5py.File(args.source, "r") as source:
        source_episode = source[f"data/{args.episode}"]
        operator_delta = source_episode["actions/operator_delta"]
        original_count = int(source_episode.attrs["num_samples"])
        active = np.any(np.abs(operator_delta[...]) > 1.0e-8, axis=1)
        active_indices = np.flatnonzero(active)
        if active_indices.size == 0:
            raise ValueError(f"{args.episode}: no non-zero operator action")

        first_active = int(active_indices[0])
        trim_start = max(0, first_active - args.pre_roll_samples)
        sample_count = original_count - trim_start

        with h5py.File(args.destination, "w") as destination:
            copy_attrs(source, destination)
            destination_data = destination.create_group("data")
            copy_attrs(source["data"], destination_data)
            destination_episode = destination_data.create_group(args.episode)
            copy_group(
                source_episode,
                destination_episode,
                trim_start,
                sample_count,
                original_count,
            )
            destination_episode.attrs.modify("num_samples", sample_count)
            destination_episode.attrs["trimmed_leading_samples"] = trim_start
            destination_episode.attrs["source_file"] = str(args.source)
            fps = float(destination_episode.attrs["fps"])
            destination_episode["timestamps/frame_index"][:] = np.arange(
                sample_count, dtype=np.int64
            )
            destination_episode["timestamps/sim_time_s"][:] = (
                np.arange(sample_count, dtype=np.float64) / fps
            )
            destination_data.attrs.modify("total", sample_count)

    print(
        f"S3 HDF5 TRIM PASS episode={args.episode} "
        f"original={original_count} first_active={first_active} "
        f"trim_start={trim_start} output_samples={sample_count} "
        f"output={args.destination}"
    )


if __name__ == "__main__":
    try:
        main()
    except (KeyError, OSError, ValueError) as exc:
        print(f"S3 HDF5 TRIM FAIL: {exc}")
        raise SystemExit(1)
