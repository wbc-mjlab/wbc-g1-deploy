#!/usr/bin/env python3
"""Register a wbc_mjlab motion NPZ in config/clips/manifest.yaml."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import yaml

_REQUIRED_NUMERIC_KEYS = (
  "joint_pos",
  "joint_vel",
  "body_pos_w",
  "body_quat_w",
  "body_lin_vel_w",
  "body_ang_vel_w",
)


def validate_motion_npz(path: Path) -> None:
  """Ensure the clip has the arrays wbc_g1_ctrl reads (wbc_mjlab export format)."""
  data = np.load(path, allow_pickle=True)
  missing = [k for k in _REQUIRED_NUMERIC_KEYS if k not in data.files]
  if missing:
    raise ValueError(f"{path} missing required motion fields: {missing}")
  joint_pos = np.asarray(data["joint_pos"])
  body_pos = np.asarray(data["body_pos_w"])
  if joint_pos.ndim != 2:
    raise ValueError(f"{path}: joint_pos must be 2-D, got shape {joint_pos.shape}")
  if body_pos.ndim != 3:
    raise ValueError(f"{path}: body_pos_w must be 3-D, got shape {body_pos.shape}")


def load_manifest(path: Path) -> dict:
  if path.is_file():
    return yaml.safe_load(path.read_text()) or {}
  return {"clips": []}


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--motion-file", type=Path, required=True)
  parser.add_argument("--name", required=True, help="clip id in manifest")
  parser.add_argument(
    "--clips-dir",
    type=Path,
    default=Path("robots/g1/config/clips"),
    help="clips directory containing manifest.yaml",
  )
  parser.add_argument("--set-default", action="store_true")
  args = parser.parse_args()

  src = args.motion_file.resolve()
  validate_motion_npz(src)

  clips_dir = args.clips_dir.resolve()
  clips_dir.mkdir(parents=True, exist_ok=True)
  manifest_path = clips_dir / "manifest.yaml"

  dest_name = f"{args.name}.npz"
  dest_path = clips_dir / dest_name
  dest_path.write_bytes(src.read_bytes())

  doc = load_manifest(manifest_path)
  clips = doc.setdefault("clips", [])
  clips = [c for c in clips if c.get("name") != args.name]
  clips.append({"name": args.name, "file": dest_name})
  doc["clips"] = clips
  if args.set_default or "default" not in doc:
    doc["default"] = args.name

  manifest_path.write_text(yaml.safe_dump(doc, sort_keys=False))
  print(f"Registered clip '{args.name}' → {dest_path}")
  print(f"Updated {manifest_path}")


if __name__ == "__main__":
  main()
