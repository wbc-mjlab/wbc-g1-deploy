# G1 WBC Deploy

Onboard deployment for Unitree G1 whole-body tracking. **One policy, many motion clips** — swap NPZ trajectories at runtime with the joystick.

This stack pairs with [wbc-mjlab](https://github.com/wbc-mjlab/wbc-mjlab), which extends mjlab's original **single-clip** tracking example into a **clip-library** training pipeline (multi-clip datasets, RSI, NPZ conversion). Deploy mirrors that model: one exported ONNX policy plus a `config/clips/` library selected at runtime (`manifest.yaml`).

The controller subscribes to Unitree SDK2 `LowState`, runs the bundled ONNX policy, and publishes motor commands.

## Demo

[![Simulation demo](https://img.youtube.com/vi/CSyczObERIc/maxresdefault.jpg)](https://www.youtube.com/watch?v=CSyczObERIc)

Bundled policy and clip library on loopback (`--network=lo`). [Watch on YouTube](https://www.youtube.com/watch?v=CSyczObERIc).

## Bundled policy

The included `policy.onnx` was trained in [wbc-mjlab](https://github.com/wbc-mjlab/wbc-mjlab) (`Wbc-G1`) mainly on the **LAFAN1 retarget** library, plus a **small subset of BONES-SEED** clips (mostly **back and side flips**). It is a reference controller for the deploy stack — not a universal foundation model.

For motions or skills outside that training mix, **train your own policy** on your clips and drop in the exported artifacts:

```bash
# in wbc-mjlab
uv run wbc-mjlab-train --task Wbc-G1 --dataset lafan
# play exports params/policy.onnx + params/config.yaml → copy into config/policy/
```

See [wbc-mjlab docs](https://wbc-mjlab.github.io/wbc-mjlab/) for datasets, motion conversion, and export. Bundled runtime clips live under `config/clips/` (`manifest.yaml`).

**Standing:** Passive → FixStand (L2 + D-pad Up) → WBC tracking (R2 + A)

**From floor:** Passive → FloorReady (L2 + D-pad Down) → WBC + getup (R2 + Y)

## Build and run

```bash
scripts/bootstrap_thirdparty.sh
mkdir -p build && cd build && cmake .. && make -j
./wbc_g1_ctrl --network=lo   # or your robot NIC (e.g. eth0)
```

`--network` selects the Unitree DDS interface for `rt/lowstate` / `rt/lowcmd` (domain 0).

## Joystick

| Action | Buttons |
|--------|---------|
| FixStand (standing) | L2 + D-pad Up |
| Floor pose (getup frame 0) | L2 + D-pad Down |
| WBC tracking (standing) | R2 + A (from FixStand) |
| WBC + getup (from floor) | R2 + Y (from FloorReady) |
| Passive | L2 + B |
| Select clip | RT + D-pad right / left (standing only) |
| Play selected clip | A |
| Liedown | D-pad down (when idle: select mode or after clip finishes; auto-plays) |
| Getup | D-pad up (when down and idle; auto-plays) |

## Dependencies

- [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2)
- ONNX Runtime 1.22.0 (`scripts/bootstrap_thirdparty.sh`)
- yaml-cpp, Boost, Eigen3, fmt, spdlog, zlib

Ubuntu packages (typical dev machine):

```bash
sudo apt install -y \
  cmake g++ build-essential \
  libyaml-cpp-dev libboost-all-dev libeigen3-dev \
  libspdlog-dev libfmt-dev zlib1g-dev
```

If unitree_sdk2 is installed to a user prefix:

```bash
export UNITREE_SDK_PREFIX=$HOME/.local/unitree_robotics
```

CMake also checks `/opt/unitree_robotics`.

## Config layout

| File | Purpose |
|------|---------|
| `config/config.yaml` | Active `policy_dir`, `clips_dir`, FSM + joystick |
| `config/robot_defaults.yaml` | Real-robot PD gains, `joint_ids_map`, `default_joint_pos`, PD torque clip catalog |
| `config/policy/wbc/params/config.yaml` | Deploy observation/history and action layout |
| `config/policy/wbc/params/policy.onnx` | Policy weights |
| `config/clips/` | Motion NPZ library + `manifest.yaml` |

Set the active policy bundle in `config/config.yaml`:

```yaml
policy_dir: config/policy/wbc
clips_dir: config/clips
clips_manifest: manifest.yaml
```

Per-state FSM keys (`policy_dir`, `clips_dir`, …) override the root values when present.

### PD torque clipping

Before sending motor position targets, WBC can back-solve clipped positions so implied PD torque stays within motor limits (same scheme as `robot_core/motor_control_cpp`):

```yaml
# config/robot_defaults.yaml
pd_torque_clipping:
  enabled: true
  fraction: 0.98          # soft cap = fraction * joint_max_torque
  joint_max_torque: [...] # per policy joint, N·m
```

`Wbc_Tracking.pd_torque_clipping` overrides these defaults when set.

## Layout

```
wbc_g1_deploy/
  apps/wbc_g1_ctrl/        # main executable
  src/                     # WBC tracking, clips, ONNX runtime, MDP registrations
  include/                 # FSM + Isaac Lab-style obs runtime
  config/
  scripts/
```

## Related repos

| Repo | Role |
|------|------|
| [wbc-mjlab/wbc-mjlab](https://github.com/wbc-mjlab/wbc-mjlab) | Train policies; export `policy.onnx` + `config.yaml` |
| [wbc-mjlab/wbc-g1-deploy](https://github.com/wbc-mjlab/wbc-g1-deploy) | This repo — G1 ONNX runtime |
| [mujocolab/mjlab](https://github.com/mujocolab/mjlab) | Simulation / training stack |
