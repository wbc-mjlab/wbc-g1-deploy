#!/usr/bin/env python3
"""Smoke tests for training → deploy YAML conversion."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parents[1]
if str(_REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(_REPO_ROOT))

from export.convert_tracking_params import (  # noqa: E402
  deploy_action_class,
  resolve_action_mode,
  resolve_actor_history_length,
  tracking_params_to_deploy,
)

_NEW_FORMAT = {
  "schema_version": "wbc_tracking_params_v1",
  "robot_id": "g1",
  "policy_step_dt": 0.02,
  "joint_names": ["j0", "j1"],
  "default_joint_pos": [0.0, 0.1],
  "stiffness": [1.0, 2.0],
  "damping": [0.1, 0.2],
  "action": {
    "action_mode": "reference_residual",
    "scale": [0.5, 0.5],
    "command_name": "motion",
  },
  "actor_observations": {
    "command": {"dim": 12, "scale": [1.0] * 12, "params": {"command_name": "motion"}},
    "joint_pos": {"dim": 2, "scale": [1.0, 1.0], "params": {}},
  },
  "tracking": {
    "anchor_body_name": "torso_link",
    "has_state_estimation": False,
    "wbc_command_dim": 12,
    "actor_observation_names": ["command", "joint_pos"],
    "actor_history_length": 10,
  },
}

_LEGACY_FORMAT = {
  "schema_version": "wbc_tracking_params_v1",
  "robot_id": "g1",
  "policy_step_dt": 0.02,
  "joint_names": ["j0", "j1"],
  "default_joint_pos": [0.0, 0.1],
  "stiffness": [1.0, 2.0],
  "damping": [0.1, 0.2],
  "action": {
    "type": "ReferenceJointPositionAction",
    "scale": [0.5, 0.5],
    "command_name": "motion",
  },
  "actor_observations": {
    "command": {"dim": 12, "scale": [1.0] * 12, "params": {"command_name": "motion"}},
  },
  "tracking": {
    "anchor_body_name": "torso_link",
    "wbc_command_dim": 12,
    "action_mode": "reference_residual",
    "actor_observation_names": ["command"],
  },
}


class ConvertTrackingParamsTests(unittest.TestCase):
  def test_resolve_action_mode_from_action_block(self) -> None:
    self.assertEqual(resolve_action_mode(_NEW_FORMAT), "reference_residual")

  def test_resolve_action_mode_from_legacy_type(self) -> None:
    self.assertEqual(resolve_action_mode(_LEGACY_FORMAT), "reference_residual")

  def test_resolve_actor_history_length(self) -> None:
    self.assertEqual(resolve_actor_history_length(_NEW_FORMAT), 10)
    self.assertEqual(resolve_actor_history_length(_LEGACY_FORMAT), 1)

  def test_deploy_action_class_mapping(self) -> None:
    self.assertEqual(
      deploy_action_class("reference_residual"), "ReferenceJointPositionAction"
    )
    self.assertEqual(deploy_action_class("default_relative"), "JointPositionAction")

  def test_tracking_params_to_deploy_history_and_action(self) -> None:
    doc = tracking_params_to_deploy(_NEW_FORMAT)
    self.assertIn("ReferenceJointPositionAction", doc["actions"])
    self.assertEqual(doc["wbc_tracking"]["action_mode"], "reference_residual")
    self.assertEqual(doc["wbc_tracking"]["actor_history_length"], 10)
    for term in doc["observations"].values():
      self.assertEqual(term["history_length"], 10)

  def test_observation_name_mapping(self) -> None:
    doc = tracking_params_to_deploy(_NEW_FORMAT)
    self.assertIn("joint_pos_rel", doc["observations"])
    self.assertNotIn("joint_pos", doc["observations"])


if __name__ == "__main__":
  unittest.main()
