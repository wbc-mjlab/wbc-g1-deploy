#!/usr/bin/env python3
"""Pack checkpoint ONNX + deploy.yaml for wbc_g1_ctrl."""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(_REPO_ROOT))

from export.convert_tracking_params import write_deploy_yaml  # noqa: E402

_TRAINING_PARAMS = Path("params") / "wbc_tracking_params.yaml"
_LEGACY_SUBDIR_PARAMS = Path("params") / "wbc_tracking" / "wbc_tracking_params.yaml"
_LEGACY_PARAMS = Path("params") / "policy_export" / "policy_export.yaml"


def _find_training_params(checkpoint: Path, explicit: Path | None) -> Path:
  if explicit is not None:
    if not explicit.is_file():
      raise FileNotFoundError(explicit)
    return explicit
  for candidate in (
    checkpoint / _TRAINING_PARAMS,
    checkpoint / _LEGACY_SUBDIR_PARAMS,
    checkpoint / _LEGACY_PARAMS,
  ):
    if candidate.is_file():
      return candidate
  raise FileNotFoundError(
    f"No training params under {checkpoint} (expected {_TRAINING_PARAMS}, "
    f"{_LEGACY_SUBDIR_PARAMS}, or {_LEGACY_PARAMS})"
  )


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--checkpoint", type=Path, required=True)
  parser.add_argument(
    "--out",
    type=Path,
    required=True,
    help="policy dir, e.g. robots/g1/config/policy/wbc",
  )
  parser.add_argument("--onnx-name", default="policy.onnx")
  parser.add_argument(
    "--params-yaml",
    type=Path,
    default=None,
    help="training wbc_tracking_params.yaml",
  )
  parser.add_argument("--defaults", type=Path, default=None)
  args = parser.parse_args()

  params = args.checkpoint / "params"
  src_onnx = params / args.onnx_name
  if not src_onnx.is_file():
    src_onnx = params / "latest.onnx"
  if not src_onnx.is_file():
    raise FileNotFoundError(f"No ONNX under {params}")

  params_src = _find_training_params(args.checkpoint, args.params_yaml)

  out_params = args.out / "params"
  out_exported = args.out / "exported"
  out_params.mkdir(parents=True, exist_ok=True)
  out_exported.mkdir(parents=True, exist_ok=True)

  deploy_doc = write_deploy_yaml(
    params_src, out_params / "deploy.yaml", defaults_path=args.defaults
  )
  shutil.copy2(src_onnx, out_exported / "policy.onnx")
  if (params / "policy.onnx.data").is_file():
    shutil.copy2(params / "policy.onnx.data", out_exported / "policy.onnx.data")

  wbc = deploy_doc["wbc_tracking"]
  action_key = next(iter(deploy_doc["actions"]))
  print(f"Policy bundle: {args.out}")
  print(f"  params/deploy.yaml (from {params_src.name})")
  print(f"    action={action_key}, action_mode={wbc.get('action_mode')}")
  print(f"    actor_history_length={wbc.get('actor_history_length')}")
  print("  exported/policy.onnx")


if __name__ == "__main__":
  main()
