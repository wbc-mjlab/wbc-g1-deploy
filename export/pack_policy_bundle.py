#!/usr/bin/env python3
"""Pack a wbc_mjlab checkpoint into a wbc_g1_ctrl policy directory."""

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
_ONNX_CANDIDATES = ("latest.onnx", "policy.onnx")


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


def _find_onnx(params_dir: Path, onnx_name: str | None) -> Path:
  if onnx_name:
    path = params_dir / onnx_name
    if path.is_file():
      return path
    raise FileNotFoundError(path)
  for name in _ONNX_CANDIDATES:
    path = params_dir / name
    if path.is_file():
      return path
  raise FileNotFoundError(f"No ONNX under {params_dir} (tried {_ONNX_CANDIDATES})")


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--checkpoint", type=Path, required=True)
  parser.add_argument(
    "--out",
    type=Path,
    required=True,
    help="policy dir, e.g. robots/g1/config/policy/wbc",
  )
  parser.add_argument("--onnx-name", default=None, help="default: latest.onnx or policy.onnx")
  parser.add_argument(
    "--params-yaml",
    type=Path,
    default=None,
    help="training wbc_tracking_params.yaml",
  )
  parser.add_argument(
    "--defaults",
    type=Path,
    default=None,
    help="optional robots/g1/config/policy_defaults.yaml for real-robot PD",
  )
  parser.add_argument(
    "--write-deploy-yaml",
    action="store_true",
    help="also write params/deploy.yaml (runtime reads wbc_tracking_params.yaml directly)",
  )
  args = parser.parse_args()

  params_dir = args.checkpoint / "params"
  src_onnx = _find_onnx(params_dir, args.onnx_name)
  params_src = _find_training_params(args.checkpoint, args.params_yaml)

  out_params = args.out / "params"
  out_params.mkdir(parents=True, exist_ok=True)

  shutil.copy2(params_src, out_params / "wbc_tracking_params.yaml")
  shutil.copy2(src_onnx, out_params / src_onnx.name)
  if (params_dir / "policy.onnx.data").is_file():
    shutil.copy2(params_dir / "policy.onnx.data", out_params / "policy.onnx.data")

  defaults = args.defaults
  if defaults is None:
    candidate = _REPO_ROOT / "robots" / "g1" / "config" / "policy_defaults.yaml"
    if candidate.is_file():
      defaults = candidate

  if args.write_deploy_yaml:
    deploy_doc = write_deploy_yaml(
      out_params / "wbc_tracking_params.yaml",
      out_params / "deploy.yaml",
      defaults_path=defaults,
    )
    wbc = deploy_doc["wbc_tracking"]
    action_key = next(iter(deploy_doc["actions"]))
    print(f"  params/deploy.yaml (action={action_key}, history={wbc.get('actor_history_length')})")

  print(f"Policy bundle: {args.out}")
  print(f"  params/wbc_tracking_params.yaml")
  print(f"  params/{src_onnx.name}")
  print("Runtime loads training params directly; ONNX resolved from params/latest.onnx")


if __name__ == "__main__":
  main()
