#!/usr/bin/env python3
"""Convert training ``wbc_tracking_params.yaml`` → runtime ``deploy.yaml`` for wbc_g1_ctrl.

Reads [wbc_mjlab](https://github.com/simeon-ned/wbc_mjlab) ``wbc_tracking_params_v1``:

- ``action.action_mode`` → deploy ``actions`` key (``ReferenceJointPositionAction`` or
  ``JointPositionAction``)
- ``tracking.actor_history_length`` → ``history_length`` on every observation term
- Legacy ``action.type`` / ``tracking.action_mode`` are still accepted
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

import yaml

_OBS_RENAME = {
  "joint_pos": "joint_pos_rel",
  "joint_vel": "joint_vel_rel",
  "actions": "last_action",
}

_ACTION_MODE_TO_DEPLOY_CLASS = {
  "reference_residual": "ReferenceJointPositionAction",
  "default_relative": "JointPositionAction",
}

_LEGACY_ACTION_TYPE_TO_MODE = {
  "ReferenceJointPositionAction": "reference_residual",
  "JointPositionAction": "default_relative",
}

DEFAULT_ACTION_MODE = "reference_residual"
DEFAULT_ACTOR_HISTORY_LENGTH = 1


def _load_yaml(path: Path) -> dict[str, Any]:
  text = path.read_text(encoding="utf-8")
  if text.startswith("#"):
    text = text.split("\n", 1)[1]
  return yaml.safe_load(text) or {}


def load_robot_defaults(robot_id: str, defaults_path: Path | None) -> dict[str, Any]:
  if defaults_path is not None:
    return _load_yaml(defaults_path)
  path = Path(__file__).resolve().parents[1] / "robots" / robot_id / "config" / "policy_defaults.yaml"
  if path.is_file():
    return _load_yaml(path)
  return {}


def resolve_action_mode(doc: dict[str, Any]) -> str:
  """Resolve semantic action mode from current or legacy training export."""
  action = doc.get("action") or {}
  if "action_mode" in action:
    return str(action["action_mode"])
  tracking = doc.get("tracking") or {}
  if "action_mode" in tracking:
    return str(tracking["action_mode"])
  legacy_type = action.get("type")
  if legacy_type in _LEGACY_ACTION_TYPE_TO_MODE:
    return _LEGACY_ACTION_TYPE_TO_MODE[legacy_type]
  return DEFAULT_ACTION_MODE


def resolve_actor_history_length(
  doc: dict[str, Any],
  *,
  robot_defaults: dict[str, Any] | None = None,
) -> int:
  """Resolve actor observation history length (1 = no stacking)."""
  tracking = doc.get("tracking") or {}
  defaults = robot_defaults or {}
  tracking_defaults = defaults.get("wbc_tracking") or defaults.get("tracking") or {}
  return int(
    tracking.get("actor_history_length")
    or tracking_defaults.get("actor_history_length")
    or DEFAULT_ACTOR_HISTORY_LENGTH
  )


def deploy_action_class(action_mode: str) -> str:
  try:
    return _ACTION_MODE_TO_DEPLOY_CLASS[action_mode]
  except KeyError as exc:
    supported = ", ".join(sorted(_ACTION_MODE_TO_DEPLOY_CLASS))
    raise ValueError(
      f"Unsupported action_mode {action_mode!r} (expected one of: {supported})"
    ) from exc


def _normalize_training_doc(doc: dict[str, Any]) -> dict[str, Any]:
  """Accept current and legacy training export schemas."""
  version = doc.get("schema_version", "")
  if version == "wbc_tracking_params_v1":
    return doc
  if version == "wbc_policy_export_v1":
    action_block = doc.get("actions") or {}
    action_type = next(iter(action_block.keys()), "ReferenceJointPositionAction")
    action_mode = _LEGACY_ACTION_TYPE_TO_MODE.get(action_type, "reference_residual")
    return {
      "schema_version": "wbc_tracking_params_v1",
      "robot_id": doc["robot_id"],
      "policy_step_dt": doc.get("policy_step_dt", doc.get("step_dt")),
      "joint_names": doc["joint_names"],
      "default_joint_pos": doc["default_joint_pos"],
      "stiffness": doc["stiffness"],
      "damping": doc["damping"],
      "action": {
        "action_mode": action_mode,
        **action_block[action_type],
      },
      "actor_observations": doc.get("observations", {}),
      "tracking": doc.get("wbc_tracking", doc.get("tracking", {})),
    }
  raise ValueError(f"Unsupported training params schema: {version!r}")


def tracking_params_to_deploy(
  params_doc: dict[str, Any],
  *,
  robot_defaults: dict[str, Any] | None = None,
) -> dict[str, Any]:
  """Build isaaclab ``deploy.yaml`` consumed by ``wbc_g1_ctrl``."""
  doc = _normalize_training_doc(params_doc)
  defaults = dict(robot_defaults or {})
  robot_id = doc.get("robot_id", defaults.get("robot_id", "g1"))
  tracking = dict(doc.get("tracking") or {})
  tracking_defaults = dict(defaults.get("wbc_tracking") or defaults.get("tracking") or {})
  actor_history_length = resolve_actor_history_length(doc, robot_defaults=defaults)

  observations: dict[str, Any] = {}
  for name, block in (doc.get("actor_observations") or {}).items():
    deploy_name = _OBS_RENAME.get(name, name)
    observations[deploy_name] = {
      "params": block.get("params", {}),
      "clip": None,
      "scale": block.get("scale"),
      "history_length": actor_history_length,
    }

  action = doc.get("action") or {}
  action_mode = resolve_action_mode(doc)
  action_type = deploy_action_class(action_mode)
  n = len(doc.get("joint_names") or [])

  deploy_tracking = {
    **tracking_defaults,
    **tracking,
    "robot_id": robot_id,
    "action_mode": action_mode,
    "training_observation_names": tracking.get("actor_observation_names", []),
    "deploy_observation_names": list(observations.keys()),
    "actor_history_length": actor_history_length,
  }

  return {
    "joint_ids_map": defaults.get("joint_ids_map") or list(range(n)),
    "step_dt": doc["policy_step_dt"],
    "stiffness": defaults.get("stiffness") or doc.get("stiffness"),
    "damping": defaults.get("damping") or doc.get("damping"),
    "default_joint_pos": defaults.get("default_joint_pos") or doc.get("default_joint_pos"),
    "commands": {},
    "actions": {
      action_type: {
        "clip": action.get("clip"),
        "joint_names": action.get("joint_names", [".*"]),
        "scale": action["scale"],
        "offset": action.get("offset"),
        "joint_ids": action.get("joint_ids"),
        "command_name": action.get("command_name", "motion"),
      }
    },
    "observations": observations,
    "wbc_tracking": deploy_tracking,
  }


def write_deploy_yaml(
  params_path: Path,
  deploy_path: Path,
  *,
  defaults_path: Path | None = None,
) -> dict[str, Any]:
  params_doc = _load_yaml(params_path)
  robot_id = str(params_doc.get("robot_id", "g1"))
  defaults = load_robot_defaults(robot_id, defaults_path)
  doc = tracking_params_to_deploy(params_doc, robot_defaults=defaults)
  deploy_path.parent.mkdir(parents=True, exist_ok=True)
  header = f"# From training {params_path.name}\n"
  deploy_path.write_text(
    header + yaml.safe_dump(doc, sort_keys=False, default_flow_style=None),
    encoding="utf-8",
  )
  return doc


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--params", type=Path, required=True, help="wbc_tracking_params.yaml")
  parser.add_argument("--out", type=Path, required=True, help="deploy.yaml for wbc_g1_ctrl")
  parser.add_argument("--defaults", type=Path, default=None, help="robot policy_defaults.yaml")
  args = parser.parse_args()

  doc = write_deploy_yaml(args.params, args.out, defaults_path=args.defaults)
  wbc = doc["wbc_tracking"]
  action_key = next(iter(doc["actions"]))
  print(
    f"Wrote {args.out} "
    f"(action={action_key}, action_mode={wbc.get('action_mode')}, "
    f"actor_history_length={wbc.get('actor_history_length')}, "
    f"wbc_command_dim={wbc.get('wbc_command_dim')})"
  )


if __name__ == "__main__":
  main()
