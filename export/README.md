# export/

Tools that bridge [wbc_mjlab](../../wbc_mjlab) training artifacts to `wbc_g1_ctrl`.

| Script | Role |
|--------|------|
| `convert_tracking_params.py` | `wbc_tracking_params.yaml` → `deploy.yaml` |
| `pack_policy_bundle.py` | Checkpoint ONNX + convert → `config/policy/wbc/` |
| `add_clip.py` | NPZ clips → `config/clips/manifest.yaml` |
| `test_convert_tracking_params.py` | Unit tests for the converter |

## `wbc_tracking_params_v1` (training)

Written by wbc_mjlab on checkpoint save or via `wbc-mjlab-export-tracking-params`.

Key fields consumed here:

```yaml
action:
  action_mode: reference_residual   # or default_relative
  scale: [...]
  command_name: motion
tracking:
  actor_history_length: 10          # 1 for no history (Zest / NoSE)
  wbc_command_dim: 39
  anchor_body_name: torso_link
  actor_observation_names: [...]
```

## `deploy.yaml` (runtime)

Generated for `ManagerBasedRLEnv` + ONNX runner:

```yaml
actions:
  ReferenceJointPositionAction:     # from action_mode
    scale: [...]
    command_name: motion
observations:
  command:
    history_length: 10              # from actor_history_length
  joint_pos_rel:
    history_length: 10
wbc_tracking:
  action_mode: reference_residual
  actor_history_length: 10
```

## Examples

Pack full bundle from a training run:

```bash
python export/pack_policy_bundle.py \
  --checkpoint ../wbc_mjlab/logs/rsl_rl/wbc_g1/2026-06-07_22-52-43 \
  --out robots/g1/config/policy/wbc
```

Convert params only:

```bash
python export/convert_tracking_params.py \
  --params ../wbc_mjlab/logs/rsl_rl/wbc_g1/<run>/params/wbc_tracking_params.yaml \
  --out robots/g1/config/policy/wbc/params/deploy.yaml \
  --defaults robots/g1/config/policy_defaults.yaml
```
