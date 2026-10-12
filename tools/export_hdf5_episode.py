#!/usr/bin/env python3
"""Export camera videos and diagnostics from a Cobot Magic HDF5 episode."""

import argparse
import json
from pathlib import Path

import cv2
import h5py
import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


JOINT_NAMES = [f"joint{i}" for i in range(6)] + ["gripper"]


def dataset_info(dataset):
    array = dataset[()]
    finite = np.isfinite(array)
    finite_values = array[finite]
    info = {
        "shape": list(dataset.shape),
        "dtype": str(dataset.dtype),
        "nan_count": int(np.isnan(array).sum()),
        "inf_count": int(np.isinf(array).sum()),
        "zero_fraction": float(np.mean(array == 0)),
    }
    if finite_values.size:
        info.update(
            {
                "min": float(finite_values.min()),
                "max": float(finite_values.max()),
                "mean": float(finite_values.mean()),
                "std": float(finite_values.std()),
            }
        )
    if array.ndim == 2:
        info["per_dimension_min"] = np.nanmin(array, axis=0).astype(float).tolist()
        info["per_dimension_max"] = np.nanmax(array, axis=0).astype(float).tolist()
        info["per_dimension_range"] = np.ptp(array, axis=0).astype(float).tolist()
        info["per_dimension_std"] = np.nanstd(array, axis=0).astype(float).tolist()
    return info


def to_bgr(frame, source_color):
    if source_color == "rgb":
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    return frame


def add_label(frame, label):
    result = frame.copy()
    cv2.rectangle(result, (0, 0), (250, 36), (0, 0, 0), -1)
    cv2.putText(
        result,
        label,
        (10, 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return result


def open_writer(path, fps, size):
    writer = cv2.VideoWriter(
        str(path), cv2.VideoWriter_fourcc(*"mp4v"), float(fps), size
    )
    if not writer.isOpened():
        raise RuntimeError(f"Could not open MP4 writer for {path}")
    return writer


def export_videos(images, output_dir, stem, fps, source_color):
    camera_names = sorted(images.keys())
    if not camera_names:
        raise ValueError("No camera datasets found")

    shapes = {name: images[name].shape for name in camera_names}
    frame_count = min(shape[0] for shape in shapes.values())
    heights = {shape[1] for shape in shapes.values()}
    if len(heights) != 1:
        raise ValueError(f"Camera heights differ: {shapes}")
    height = heights.pop()

    writers = {}
    output_paths = {}
    for name in camera_names:
        _, camera_height, camera_width, channels = shapes[name]
        if channels != 3:
            raise ValueError(f"Expected three color channels for {name}, got {shapes[name]}")
        path = output_dir / f"{stem}_{name}.mp4"
        writers[name] = open_writer(path, fps, (camera_width, camera_height))
        output_paths[name] = path

    combined_width = sum(shapes[name][2] for name in camera_names)
    combined_path = output_dir / f"{stem}_combined.mp4"
    combined_writer = open_writer(combined_path, fps, (combined_width, height))

    frame_delta_sum = {name: 0.0 for name in camera_names}
    previous_frames = {}
    try:
        for index in range(frame_count):
            combined_frames = []
            for name in camera_names:
                frame = to_bgr(images[name][index], source_color)
                writers[name].write(frame)
                combined_frames.append(add_label(frame, name))
                if name in previous_frames:
                    delta = cv2.absdiff(frame, previous_frames[name])
                    frame_delta_sum[name] += float(delta.mean())
                previous_frames[name] = frame
            combined_writer.write(np.concatenate(combined_frames, axis=1))
            if (index + 1) % 50 == 0 or index + 1 == frame_count:
                print(f"Exported {index + 1}/{frame_count} frames")
    finally:
        for writer in writers.values():
            writer.release()
        combined_writer.release()

    mean_frame_delta = {
        name: frame_delta_sum[name] / max(frame_count - 1, 1) for name in camera_names
    }
    return output_paths, combined_path, mean_frame_delta


def save_contact_sheet(images, output_path, source_color):
    camera_names = sorted(images.keys())
    frame_count = min(images[name].shape[0] for name in camera_names)
    indices = [0, frame_count // 2, frame_count - 1]
    rows = []
    for name in camera_names:
        row = []
        for index in indices:
            label = f"{name} | frame {index}"
            row.append(add_label(to_bgr(images[name][index], source_color), label))
        rows.append(np.concatenate(row, axis=1))
    sheet = np.concatenate(rows, axis=0)
    if not cv2.imwrite(str(output_path), sheet, [cv2.IMWRITE_JPEG_QUALITY, 92]):
        raise RuntimeError(f"Could not write contact sheet to {output_path}")


def plot_qpos_action(qpos, action, output_path):
    fig, axes = plt.subplots(7, 2, figsize=(14, 18), sharex=True)
    for side_index, side in enumerate(("left", "right")):
        offset = side_index * 7
        for joint_index, joint_name in enumerate(JOINT_NAMES):
            axis = axes[joint_index, side_index]
            axis.plot(qpos[:, offset + joint_index], label="qpos", linewidth=1.4)
            axis.plot(action[:, offset + joint_index], label="action", linewidth=1.0, alpha=0.8)
            axis.set_title(f"{side} {joint_name}")
            axis.grid(alpha=0.25)
            if joint_index == 0:
                axis.legend()
    axes[-1, 0].set_xlabel("frame")
    axes[-1, 1].set_xlabel("frame")
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def plot_effort(effort, output_path):
    fig, axes = plt.subplots(7, 2, figsize=(14, 18), sharex=True)
    for side_index, side in enumerate(("left", "right")):
        offset = side_index * 7
        for joint_index, joint_name in enumerate(JOINT_NAMES):
            axis = axes[joint_index, side_index]
            axis.plot(effort[:, offset + joint_index], linewidth=1.2)
            axis.set_title(f"{side} {joint_name} effort")
            axis.grid(alpha=0.25)
    axes[-1, 0].set_xlabel("frame")
    axes[-1, 1].set_xlabel("frame")
    fig.tight_layout()
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def write_text_summary(summary, output_path):
    action = summary["datasets"]["action"]
    qpos = summary["datasets"]["observations/qpos"]
    effort = summary["datasets"]["observations/effort"]
    lines = [
        f"Source: {summary['source']}",
        f"File size: {summary['file_size_bytes']} bytes",
        f"Frames: {summary['frame_count']}",
        f"FPS: {summary['fps']}",
        f"Duration: {summary['duration_seconds']:.3f} seconds",
        f"Cameras: {', '.join(summary['camera_names'])}",
        "",
        f"action zero fraction: {action['zero_fraction']:.6f}",
        f"action range by dimension: {action['per_dimension_range']}",
        f"qpos range by dimension: {qpos['per_dimension_range']}",
        f"effort zero fraction: {effort['zero_fraction']:.6f}",
        f"effort range by dimension: {effort['per_dimension_range']}",
        "",
        "Mean absolute pixel change between adjacent frames:",
    ]
    for name, value in summary["mean_absolute_frame_delta"].items():
        lines.append(f"  {name}: {value:.6f}")
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("episode", type=Path)
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--fps", type=float, default=20.0)
    parser.add_argument("--source-color", choices=("rgb", "bgr"), default="rgb")
    args = parser.parse_args()

    episode = args.episode.expanduser().resolve()
    output_dir = (args.output_dir or episode.parent).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = episode.stem

    with h5py.File(episode, "r") as root:
        images = root["observations/images"]
        camera_names = sorted(images.keys())
        frame_count = min(images[name].shape[0] for name in camera_names)
        video_paths, combined_path, frame_delta = export_videos(
            images, output_dir, stem, args.fps, args.source_color
        )

        contact_sheet_path = output_dir / f"{stem}_contact_sheet.jpg"
        save_contact_sheet(images, contact_sheet_path, args.source_color)

        qpos = root["observations/qpos"][()]
        action = root["action"][()]
        effort = root["observations/effort"][()]
        qpos_plot_path = output_dir / f"{stem}_qpos_action.png"
        effort_plot_path = output_dir / f"{stem}_effort.png"
        plot_qpos_action(qpos, action, qpos_plot_path)
        plot_effort(effort, effort_plot_path)

        numeric_paths = [
            "action",
            "base_action",
            "observations/qpos",
            "observations/qvel",
            "observations/effort",
        ]
        summary = {
            "source": str(episode),
            "file_size_bytes": episode.stat().st_size,
            "attributes": {
                key: value.item() if isinstance(value, np.generic) else value
                for key, value in root.attrs.items()
            },
            "fps": args.fps,
            "frame_count": frame_count,
            "duration_seconds": frame_count / args.fps,
            "camera_names": camera_names,
            "camera_shapes": {name: list(images[name].shape) for name in camera_names},
            "mean_absolute_frame_delta": frame_delta,
            "datasets": {path: dataset_info(root[path]) for path in numeric_paths},
            "outputs": {
                "combined_video": str(combined_path),
                "camera_videos": {name: str(path) for name, path in video_paths.items()},
                "contact_sheet": str(contact_sheet_path),
                "qpos_action_plot": str(qpos_plot_path),
                "effort_plot": str(effort_plot_path),
            },
        }

    json_path = output_dir / f"{stem}_summary.json"
    text_path = output_dir / f"{stem}_summary.txt"
    json_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    write_text_summary(summary, text_path)
    print(f"Summary: {text_path}")


if __name__ == "__main__":
    main()
