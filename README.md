# wbc_g1_deploy

Reference **G1** real-robot deploy for [wbc_mjlab](../wbc_mjlab). **One policy, many trajectories** — swap NPZ clips at runtime.

- **One policy** (`config/policy/wbc/`) — same WBC tracker for all motions
- **Many clips** (`config/clips/`) — switch at runtime
- **No walking stack** — Passive → FixStand → tracking only

Future: DDS trajectory + controller split — [docs/architecture.md](docs/architecture.md).

## Layout

```
wbc_g1_deploy/
  include/                 # isaaclab + FSM
  robots/g1/
    wbc_g1_ctrl            # built binary
    config/policy/wbc/     # ONNX + wbc_tracking_params.yaml
    config/policy_defaults.yaml
    config/clips/
  export/                  # training → deploy conversion
```

## Prerequisites

1. Train a policy in **wbc_mjlab** (e.g. `Wbc-G1` with actor history 10, or `Wbc-G1-ZEST` with history 1).
2. Checkpoint folder must contain:
   - `params/latest.onnx` (or `params/policy.onnx`)
   - `params/wbc_tracking_params.yaml` (written on checkpoint save, or export manually)
3. **unitree_sdk2** installed (provides Unitree DDS headers + Cyclone DDS libs).
4. **ONNX Runtime** bundles under `thirdparty/` (see [Build](#build)).
5. On the robot: DDS network interface (e.g. `eth0`).

### System dependencies (Ubuntu)

Same stack as [unitree_rl_mjlab](../unitree_rl_mjlab) deploy:

```bash
sudo apt install -y \
  cmake g++ build-essential \
  libyaml-cpp-dev libboost-all-dev libeigen3-dev \
  libspdlog-dev libfmt-dev zlib1g-dev
```

### unitree_sdk2 (required for build)

The error `unitree/dds_wrapper/robots/go2/go2.h: No such file or directory` means **unitree_sdk2 is not installed**. Cyclone DDS is bundled inside the SDK — you do not install it separately.

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install    # installs to /usr/local by default
```

Or install without sudo to a user prefix:

```bash
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/.local/unitree_robotics
make -j$(nproc) && make install
export UNITREE_SDK_PREFIX=$HOME/.local/unitree_robotics   # before cmake ..
```

CMake also checks `/opt/unitree_robotics` (Unitree’s documented prefix).

## Quick start (train → robot)

### 1. Train in wbc_mjlab

```bash
cd ../wbc_mjlab
pip install -e .
wbc-mjlab-train --task Wbc-G1 --dataset lafan
```

Checkpoint example:

```text
logs/rsl_rl/wbc_g1/<timestamp>/
  model_<iter>.pt
  params/latest.onnx
  params/wbc_tracking_params.yaml
```

If `wbc_tracking_params.yaml` is missing, regenerate from the task:

```bash
wbc-mjlab-export-tracking-params --task Wbc-G1 --out /tmp/wbc_tracking_params.yaml
```

### 2. Pack policy bundle

From **wbc_g1_deploy**:

```bash
python export/pack_policy_bundle.py \
  --checkpoint ../wbc_mjlab/logs/rsl_rl/wbc_g1/<run-dir> \
  --out robots/g1/config/policy/wbc
```

This copies `params/wbc_tracking_params.yaml` and `params/latest.onnx` as-is. At runtime,
`wbc_g1_ctrl` converts training params to the internal deploy layout in C++ (no manual
`deploy.yaml` step required).

Optional: also write `params/deploy.yaml` for inspection:

```bash
python export/pack_policy_bundle.py \
  --checkpoint ../wbc_mjlab/logs/rsl_rl/wbc_g1/<run-dir> \
  --out robots/g1/config/policy/wbc \
  --write-deploy-yaml
```

Manual convert only:

```bash
python export/convert_tracking_params.py \
  --params ../wbc_mjlab/logs/rsl_rl/wbc_g1/<run>/params/wbc_tracking_params.yaml \
  --out robots/g1/config/policy/wbc/params/deploy.yaml
```

### 3. Add motion clips

Clips are wbc_mjlab NPZ exports (`joint_pos`, `body_pos_w`, `body_quat_w`, …). Object
metadata fields (`joint_names`, `robot`) are ignored; anchor body index uses the built-in
G1 body list when `body_names` is absent.

```bash
python export/add_clip.py \
  --motion-file ../wbc_mjlab/data/g1/lafan/npz/walk1_subject1.npz \
  --name walk \
  --clips-dir robots/g1/config/clips \
  --set-default
```

### 4. Build

Install ONNX Runtime if needed:

```bash
./scripts/bootstrap_thirdparty.sh
```

Build (x86_64 for sim/dev machine, aarch64 on G1 onboard PC):

```bash
cd robots/g1
rm -rf build && mkdir build && cd build
cmake ..
make -j
```

If unitree_sdk2 is in a non-default location:

```bash
export UNITREE_SDK_PREFIX=$HOME/.local/unitree_robotics
cmake ..
make -j
```

### 5. Run on robot

```bash
cd robots/g1/build
./wbc_g1_ctrl eth0
```

Replace `eth0` with your DDS network interface.

## Joystick

| Action | Buttons |
|--------|---------|
| FixStand | L2 + Up |
| Tracking | R2 + A |
| Passive | L2 + B |
| Next clip | RT + D-pad right |
| Prev clip | RT + D-pad left |

## Training → deploy mapping

[wbc_mjlab](../wbc_mjlab) writes `wbc_tracking_params_v1`. `wbc_g1_ctrl` loads it directly and
maps it to the internal deploy layout (same rules as `export/convert_tracking_params.py`).
Legacy `params/deploy.yaml` still works if present.

| Training (`wbc_tracking_params.yaml`) | Deploy (`deploy.yaml`) |
|---------------------------------------|-------------------------|
| `action.action_mode: reference_residual` | `actions.ReferenceJointPositionAction` |
| `action.action_mode: default_relative` | `actions.JointPositionAction` |
| `tracking.actor_history_length: N` | `history_length: N` on every observation term |
| `actor_observations.joint_pos` | `observations.joint_pos_rel` |
| `actor_observations.joint_vel` | `observations.joint_vel_rel` |
| `actor_observations.actions` | `observations.last_action` |
| `policy_step_dt` | `step_dt` |
| `tracking.wbc_command_dim` | `wbc_tracking.wbc_command_dim` |

Legacy fields still work: `action.type`, `tracking.action_mode` (without `action.action_mode`).

**History ordering:** mjlab stacks observations **term-major** (`[cmd×N, ang_vel×N, …]`). Deploy uses per-term `history_length` with `use_gym_history: false` (default) — do not enable `use_gym_history` for WBC policies trained in mjlab.

**PD gains:** training stiffness/damping are simulation values. `policy_defaults.yaml` supplies real-robot PD and joint SDK order when packing.

## Verify conversion

```bash
python -m unittest export.test_convert_tracking_params
```

## export/ scripts

See [export/README.md](export/README.md).
