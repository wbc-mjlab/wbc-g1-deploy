#!/usr/bin/env python3
"""Copy an NPZ clip into config/clips and update manifest.yaml."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import yaml

from export.g1_body_names import G1_FULL_BODY_NAMES

# Arrays read by wbc_g1_ctrl (cnpy cannot load numpy object/pickled fields).
_DEPLOY_NUMERIC_KEYS = (
  "fps",
  "joint_pos",
  "joint_vel",
  "body_pos_w",
  "body_quat_w",
  "body_lin_vel_w",
  "body_ang_vel_w",
)


def sanitize_motion_npz(src: Path, dest: Path) -> None:
  """Write a deploy-safe NPZ with only numeric arrays cnpy can load."""
  data = np.load(src, allow_pickle=True)
  missing = [k for k in _DEPLOY_NUMERIC_KEYS if k not in data.files]
  if missing:
    raise ValueError(f"{src} missing required motion fields: {missing}")

  payload: dict[str, np.ndarray] = {
    key: np.asarray(data[key]) for key in _DEPLOY_NUMERIC_KEYS
  }
  if "body_names" in data.files:
    names = data["body_names"]
    if names.dtype == object:
      names = [str(x) for x in names.tolist()]
    else:
      names = [n.decode() if isinstance(n, (bytes, bytearray)) else str(n) for n in names]
  else:
    num_bodies = int(np.asarray(data["body_pos_w"]).shape[1])
    if num_bodies != len(G1_FULL_BODY_NAMES):
      raise ValueError(
        f"{src} has {num_bodies} bodies but G1 expects {len(G1_FULL_BODY_NAMES)}; "
        "re-export the clip from wbc_mjlab or provide body_names in the NPZ."
      )
    names = list(G1_FULL_BODY_NAMES)
  max_len = max((len(n) for n in names), default=1)
  payload["body_names"] = np.asarray(names, dtype=f"|S{max_len}")

  dest.parent.mkdir(parents=True, exist_ok=True)
  # np.savez appends ".npz" when the path does not already end with ".npz".
  np.savez(str(dest), **payload)


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

  clips_dir = args.clips_dir.resolve()
  clips_dir.mkdir(parents=True, exist_ok=True)
  manifest_path = clips_dir / "manifest.yaml"

  dest_name = f"{args.name}.npz"
  dest_path = clips_dir / dest_name
  sanitize_motion_npz(args.motion_file.resolve(), dest_path)

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
