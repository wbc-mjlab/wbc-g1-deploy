#!/usr/bin/env python3
"""Deploy helpers for wbc_g1_deploy: pack a trained policy, register motion clips."""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_POLICY_DIR = REPO_ROOT / "config" / "policy" / "wbc"
DEFAULT_CLIPS_DIR = REPO_ROOT / "config" / "clips"

_PARAMS = Path("params") / "config.yaml"
_LEGACY_PARAMS = Path("params") / "wbc_tracking_params.yaml"
_ONNX = "policy.onnx"
_LEGACY_ONNX = "latest.onnx"
_ONNX_CANDIDATES = (_ONNX, _LEGACY_ONNX)

_REQUIRED_NUMERIC_KEYS = (
  "joint_pos",
  "joint_vel",
  "body_pos_w",
  "body_quat_w",
  "body_lin_vel_w",
  "body_ang_vel_w",
)


def _find_training_params(checkpoint: Path, explicit: Path | None) -> Path:
  if explicit is not None:
    if not explicit.is_file():
      raise FileNotFoundError(explicit)
    return explicit
  for candidate in (checkpoint / _PARAMS, checkpoint / _LEGACY_PARAMS):
    if candidate.is_file():
      return candidate
  raise FileNotFoundError(
    f"No training params under {checkpoint} (expected {_PARAMS} or {_LEGACY_PARAMS})"
  )


def _find_onnx(params_dir: Path) -> Path:
  for name in _ONNX_CANDIDATES:
    path = params_dir / name
    if path.is_file():
      return path
  raise FileNotFoundError(f"No ONNX under {params_dir} (tried {_ONNX_CANDIDATES})")


def cmd_pack(args: argparse.Namespace) -> None:
  checkpoint = args.checkpoint.resolve()
  out = args.out.resolve()
  params_dir = checkpoint / "params"

  params_src = _find_training_params(checkpoint, args.params_yaml)
  src_onnx = _find_onnx(params_dir)

  out_params = out / "params"
  out_params.mkdir(parents=True, exist_ok=True)

  shutil.copy2(params_src, out_params / "config.yaml")
  shutil.copy2(src_onnx, out_params / _ONNX)
  if (params_dir / "policy.onnx.data").is_file():
    shutil.copy2(params_dir / "policy.onnx.data", out_params / "policy.onnx.data")

  print(f"Policy bundle: {out}")
  print("  params/config.yaml")
  print(f"  params/{_ONNX}")


def validate_motion_npz(path: Path) -> None:
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


def cmd_add_clip(args: argparse.Namespace) -> None:
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


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  sub = parser.add_subparsers(dest="command", required=True)

  pack = sub.add_parser("pack", help="copy checkpoint params/ into config/policy/wbc/")
  pack.add_argument("--checkpoint", type=Path, required=True)
  pack.add_argument(
    "--out",
    type=Path,
    default=DEFAULT_POLICY_DIR,
    help=f"policy directory (default: {DEFAULT_POLICY_DIR.relative_to(REPO_ROOT)})",
  )
  pack.add_argument(
    "--params-yaml",
    type=Path,
    default=None,
    help="override training config.yaml path",
  )
  pack.set_defaults(func=cmd_pack)

  clip = sub.add_parser("add-clip", help="register a motion NPZ in config/clips/")
  clip.add_argument("--motion-file", type=Path, required=True)
  clip.add_argument("--name", required=True, help="clip id in manifest.yaml")
  clip.add_argument(
    "--clips-dir",
    type=Path,
    default=DEFAULT_CLIPS_DIR,
    help=f"clips directory (default: {DEFAULT_CLIPS_DIR.relative_to(REPO_ROOT)})",
  )
  clip.add_argument("--set-default", action="store_true")
  clip.set_defaults(func=cmd_add_clip)

  args = parser.parse_args()
  args.func(args)


if __name__ == "__main__":
  main()
