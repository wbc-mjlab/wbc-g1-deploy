#!/usr/bin/env python3
"""Slice motion clips and blend to/from the arms_down pose.

Reads trajectory definitions from COPY_TRAJECTORIES.txt, slices source NPZ
files by frame range, optionally prepends/appends a 0.5 s smooth blend through
arms_down (yaw-aligned to the trajectory at each boundary). Skips blending for
liedown/getup trajectories.

Usage:
  python scripts/slice_trajectories.py
  python scripts/slice_trajectories.py --trajectories /path/to/COPY_TRAJECTORIES.txt
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CLIPS_DIR = REPO_ROOT / "config" / "clips"
DEFAULT_TRAJECTORIES = REPO_ROOT.parents[1] / "COPY_TRAJECTORIES.txt"

BLEND_SEC = 0.5
ANCHOR_BODY = "torso_link"
PELVIS_IDX = 0
SKIP_BLEND_SUBSTRINGS = ("liedown", "getup")

G1_BODY_NAMES = [
    "pelvis",
    "left_hip_pitch_link",
    "left_hip_roll_link",
    "left_hip_yaw_link",
    "left_knee_link",
    "left_ankle_pitch_link",
    "left_ankle_roll_link",
    "right_hip_pitch_link",
    "right_hip_roll_link",
    "right_hip_yaw_link",
    "right_knee_link",
    "right_ankle_pitch_link",
    "right_ankle_roll_link",
    "waist_yaw_link",
    "waist_roll_link",
    "torso_link",
    "left_shoulder_pitch_link",
    "left_shoulder_roll_link",
    "left_shoulder_yaw_link",
    "left_elbow_link",
    "left_wrist_roll_link",
    "left_wrist_pitch_link",
    "left_wrist_yaw_link",
    "right_shoulder_pitch_link",
    "right_shoulder_roll_link",
    "right_shoulder_yaw_link",
    "right_elbow_link",
    "right_wrist_roll_link",
    "right_wrist_pitch_link",
    "right_wrist_yaw_link",
]

TRAJ_RE = re.compile(
    r"^(?P<name>[A-Za-z0-9_]+)\s*:\s*(?P<file>\S+\.npz)\s*,?\s*(?P<start>\d+)\s*,\s*(?P<end>\d+)\s*$"
)
POSE_RE = re.compile(
    r"^(?P<name>[A-Za-z0-9_]+)\s*:\s*(?P<file>\S+\.npz)\s*:\s*(?P<frame>\d+)\s*$"
)


def quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=np.float64)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return q / n


def quat_conj(q: np.ndarray) -> np.ndarray:
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return quat_normalize(
        np.array(
            [
                w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
                w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            ],
            dtype=np.float64,
        )
    )


def quat_apply(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    q = quat_normalize(q)
    xyz = q[1:]
    t = np.cross(xyz, v) * 2.0
    return v + q[0] * t + np.cross(xyz, t)


def yaw_quat(q: np.ndarray) -> np.ndarray:
    q = quat_normalize(q)
    w, x, y, z = q
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.array([np.cos(yaw * 0.5), 0.0, 0.0, np.sin(yaw * 0.5)], dtype=np.float64)


def quat_slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        out = q0 + t * (q1 - q0)
        return quat_normalize(out)
    theta_0 = np.arccos(np.clip(dot, -1.0, 1.0))
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * t
    s0 = np.sin(theta_0 - theta) / sin_theta_0
    s1 = np.sin(theta) / sin_theta_0
    return quat_normalize(s0 * q0 + s1 * q1)


def smoothstep(t: float) -> float:
    t = float(np.clip(t, 0.0, 1.0))
    return t * t * (3.0 - 2.0 * t)


def resolve_anchor_index(body_names: np.ndarray | None) -> int:
    if body_names is not None:
        names = [str(x) for x in body_names.tolist()]
        if ANCHOR_BODY in names:
            return names.index(ANCHOR_BODY)
    if ANCHOR_BODY in G1_BODY_NAMES:
        return G1_BODY_NAMES.index(ANCHOR_BODY)
    raise ValueError(f"Anchor body '{ANCHOR_BODY}' not found")


def load_npz(path: Path) -> dict:
    with np.load(path, allow_pickle=True) as data:
        out = {key: np.asarray(data[key]) for key in data.files}
    return out


def extract_frame(data: dict, idx: int) -> dict[str, np.ndarray]:
    return {
        "joint_pos": np.asarray(data["joint_pos"][idx], dtype=np.float32),
        "joint_vel": np.asarray(data["joint_vel"][idx], dtype=np.float32),
        "body_pos_w": np.asarray(data["body_pos_w"][idx], dtype=np.float32),
        "body_quat_w": np.asarray(data["body_quat_w"][idx], dtype=np.float32),
        "body_lin_vel_w": np.asarray(data["body_lin_vel_w"][idx], dtype=np.float32),
        "body_ang_vel_w": np.asarray(data["body_ang_vel_w"][idx], dtype=np.float32),
    }


def zero_velocities(frame: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    out = {k: v.copy() for k, v in frame.items()}
    out["joint_vel"] = np.zeros_like(out["joint_vel"])
    out["body_lin_vel_w"] = np.zeros_like(out["body_lin_vel_w"])
    out["body_ang_vel_w"] = np.zeros_like(out["body_ang_vel_w"])
    return out


def joint_names_list(data: dict) -> list[str] | None:
    if "joint_names" not in data:
        return None
    return [str(x) for x in data["joint_names"].tolist()]


def align_pose_to_boundary(
    pose: dict[str, np.ndarray],
    ref: dict[str, np.ndarray],
    yaw_body_idx: int,
    pelvis_idx: int = PELVIS_IDX,
) -> dict[str, np.ndarray]:
    """Place a joint-space pose with ref anchor yaw and pelvis position.

    Yaw is taken from ``yaw_body_idx`` (torso_link in deploy). The whole clip
    frame is rotated in world space about the reference pelvis; joint angles are
    left unchanged.
    """
    out = {k: v.copy() for k, v in pose.items()}

    ref_yaw_q = ref["body_quat_w"][yaw_body_idx]
    pose_yaw_q = out["body_quat_w"][yaw_body_idx]
    pose_pelvis_p = out["body_pos_w"][pelvis_idx]
    pivot = ref["body_pos_w"][pelvis_idx]

    delta_q = quat_mul(yaw_quat(ref_yaw_q), quat_conj(yaw_quat(pose_yaw_q)))

    n_bodies = out["body_pos_w"].shape[0]
    for i in range(n_bodies):
        rel = out["body_pos_w"][i] - pose_pelvis_p
        out["body_pos_w"][i] = (pivot + quat_apply(delta_q, rel)).astype(np.float32)
        out["body_quat_w"][i] = quat_mul(delta_q, out["body_quat_w"][i]).astype(
            np.float32
        )

    out["body_lin_vel_w"] = np.stack(
        [quat_apply(delta_q, v).astype(np.float32) for v in out["body_lin_vel_w"]]
    )
    out["body_ang_vel_w"] = np.stack(
        [quat_apply(delta_q, v).astype(np.float32) for v in out["body_ang_vel_w"]]
    )
    return out


def blend_frames(
    a: dict[str, np.ndarray],
    b: dict[str, np.ndarray],
    alpha: float,
) -> dict[str, np.ndarray]:
    alpha = float(alpha)
    beta = 1.0 - alpha
    n_bodies = a["body_quat_w"].shape[0]
    return {
        "joint_pos": (beta * a["joint_pos"] + alpha * b["joint_pos"]).astype(np.float32),
        "joint_vel": (beta * a["joint_vel"] + alpha * b["joint_vel"]).astype(np.float32),
        "body_pos_w": (beta * a["body_pos_w"] + alpha * b["body_pos_w"]).astype(np.float32),
        "body_lin_vel_w": (
            beta * a["body_lin_vel_w"] + alpha * b["body_lin_vel_w"]
        ).astype(np.float32),
        "body_ang_vel_w": (
            beta * a["body_ang_vel_w"] + alpha * b["body_ang_vel_w"]
        ).astype(np.float32),
        "body_quat_w": np.stack(
            [
                quat_slerp(a["body_quat_w"][i], b["body_quat_w"][i], alpha).astype(
                    np.float32
                )
                for i in range(n_bodies)
            ]
        ),
    }


def build_blend_segment(
    pose_a: dict[str, np.ndarray],
    pose_b: dict[str, np.ndarray],
    n_frames: int,
) -> list[dict[str, np.ndarray]]:
    if n_frames <= 0:
        return []
    return [
        blend_frames(pose_a, pose_b, smoothstep((i + 1) / n_frames))
        for i in range(n_frames)
    ]


def stack_frames(frames: list[dict[str, np.ndarray]]) -> dict[str, np.ndarray]:
    if not frames:
        raise ValueError("No frames to stack")
    return {
        "joint_pos": np.stack([f["joint_pos"] for f in frames]),
        "joint_vel": np.stack([f["joint_vel"] for f in frames]),
        "body_pos_w": np.stack([f["body_pos_w"] for f in frames]),
        "body_quat_w": np.stack([f["body_quat_w"] for f in frames]),
        "body_lin_vel_w": np.stack([f["body_lin_vel_w"] for f in frames]),
        "body_ang_vel_w": np.stack([f["body_ang_vel_w"] for f in frames]),
    }


def save_clip(path: Path, arrays: dict[str, np.ndarray], meta: dict) -> None:
    payload: dict = {
        "fps": np.asarray(meta["fps"], dtype=np.float32),
        "joint_pos": arrays["joint_pos"].astype(np.float32),
        "joint_vel": arrays["joint_vel"].astype(np.float32),
        "body_pos_w": arrays["body_pos_w"].astype(np.float32),
        "body_quat_w": arrays["body_quat_w"].astype(np.float32),
        "body_lin_vel_w": arrays["body_lin_vel_w"].astype(np.float32),
        "body_ang_vel_w": arrays["body_ang_vel_w"].astype(np.float32),
    }
    if "joint_names" in meta:
        payload["joint_names"] = meta["joint_names"]
    if "robot" in meta:
        payload["robot"] = meta["robot"]
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(path, **payload)


def parse_trajectories(path: Path) -> tuple[list[dict], dict[str, tuple[str, int]]]:
    trajectories: list[dict] = []
    poses: dict[str, tuple[str, int]] = {}
    in_poses = False

    for raw in path.read_text().splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line.upper().startswith("POSES"):
            in_poses = True
            continue

        if in_poses:
            m = POSE_RE.match(line)
            if not m:
                raise ValueError(f"Could not parse pose line: {raw}")
            poses[m.group("name")] = (m.group("file"), int(m.group("frame")))
            continue

        m = TRAJ_RE.match(line)
        if not m:
            raise ValueError(f"Could not parse trajectory line: {raw}")
        trajectories.append(
            {
                "name": m.group("name"),
                "file": m.group("file"),
                "start": int(m.group("start")),
                "end": int(m.group("end")),
            }
        )

    return trajectories, poses


def should_blend(name: str) -> bool:
    lower = name.lower()
    return not any(token in lower for token in SKIP_BLEND_SUBSTRINGS)


def process_trajectory(
    spec: dict,
    clips_dir: Path,
    arms_down: dict[str, np.ndarray],
    yaw_body_idx: int,
    blend_sec: float,
) -> tuple[str, str, int]:
    source_path = clips_dir / spec["file"]
    if not source_path.exists():
        raise FileNotFoundError(f"Source clip not found: {source_path}")

    data = load_npz(source_path)
    start = spec["start"]
    end = spec["end"]
    if start < 0 or end < start:
        raise ValueError(
            f"{spec['name']}: invalid frame range [{start}, {end}] in {source_path.name}"
        )
    if end >= data["joint_pos"].shape[0]:
        raise ValueError(
            f"{spec['name']}: end frame {end} out of range for {source_path.name} "
            f"(length {data['joint_pos'].shape[0]})"
        )

    fps = float(np.asarray(data["fps"]).reshape(-1)[0])
    blend_frames_n = max(1, int(round(blend_sec * fps)))

    core = [
        extract_frame(data, i) for i in range(start, end + 1)
    ]

    if should_blend(spec["name"]):
        stationary = zero_velocities(arms_down)
        start_ref = core[0]
        end_ref = core[-1]

        arms_at_start = align_pose_to_boundary(stationary, start_ref, yaw_body_idx)
        arms_at_end = align_pose_to_boundary(stationary, end_ref, yaw_body_idx)

        blend_in = build_blend_segment(arms_at_start, start_ref, blend_frames_n)
        blend_out = build_blend_segment(end_ref, arms_at_end, blend_frames_n)
        frames = blend_in + core + blend_out
    else:
        frames = core
        blend_frames_n = 0

    arrays = stack_frames(frames)
    out_name = f"{spec['name']}.npz"
    out_path = clips_dir / out_name
    meta = {"fps": data["fps"]}
    if "joint_names" in data:
        meta["joint_names"] = data["joint_names"]
    if "robot" in data:
        meta["robot"] = data["robot"]
    save_clip(out_path, arrays, meta)

    total_frames = arrays["joint_pos"].shape[0]
    print(
        f"[OK] {spec['name']}: {spec['file']}[{start}:{end}] "
        f"-> {out_name} ({total_frames} frames, blend={blend_frames_n if should_blend(spec['name']) else 0})"
    )
    return spec["name"], out_name, total_frames


def write_manifest(clips_dir: Path, entries: list[tuple[str, str]], default: str) -> None:
    lines = [
        "# Motion clips generated by slice_trajectories.py",
        "clips:",
    ]
    for name, fname in entries:
        lines.append(f"  - name: {name}")
        lines.append(f"    file: {fname}")
    lines.append(f"default: {default}")
    (clips_dir / "manifest.yaml").write_text("\n".join(lines) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--trajectories",
        type=Path,
        default=DEFAULT_TRAJECTORIES,
        help=f"Trajectory list file (default: {DEFAULT_TRAJECTORIES})",
    )
    parser.add_argument(
        "--clips-dir",
        type=Path,
        default=DEFAULT_CLIPS_DIR,
        help=f"Directory with source NPZ clips (default: {DEFAULT_CLIPS_DIR})",
    )
    parser.add_argument(
        "--blend-sec",
        type=float,
        default=BLEND_SEC,
        help=f"Blend duration in seconds (default: {BLEND_SEC})",
    )
    parser.add_argument(
        "--no-manifest",
        action="store_true",
        help="Do not rewrite config/clips/manifest.yaml",
    )
    args = parser.parse_args()

    if not args.trajectories.exists():
        print(f"Trajectory file not found: {args.trajectories}", file=sys.stderr)
        return 1
    if not args.clips_dir.is_dir():
        print(f"Clips directory not found: {args.clips_dir}", file=sys.stderr)
        return 1

    trajectories, poses = parse_trajectories(args.trajectories)
    if not trajectories:
        print("No trajectories found.", file=sys.stderr)
        return 1
    if "arms_down" not in poses:
        print("POSES section must define arms_down.", file=sys.stderr)
        return 1

    pose_file, pose_frame = poses["arms_down"]
    pose_path = args.clips_dir / pose_file
    if not pose_path.exists():
        print(f"arms_down source not found: {pose_path}", file=sys.stderr)
        return 1

    pose_data = load_npz(pose_path)
    if pose_frame >= pose_data["joint_pos"].shape[0]:
        print(
            f"arms_down frame {pose_frame} out of range for {pose_path.name}",
            file=sys.stderr,
        )
        return 1

    yaw_body_idx = resolve_anchor_index(pose_data.get("body_names"))
    arms_down = extract_frame(pose_data, pose_frame)

    manifest_entries: list[tuple[str, str]] = []
    for spec in trajectories:
        name, fname, _ = process_trajectory(
            spec,
            args.clips_dir,
            arms_down,
            yaw_body_idx,
            args.blend_sec,
        )
        manifest_entries.append((name, fname))

    if not args.no_manifest:
        write_manifest(args.clips_dir, manifest_entries, manifest_entries[0][0])
        print(f"[OK] Wrote manifest with {len(manifest_entries)} clips")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
