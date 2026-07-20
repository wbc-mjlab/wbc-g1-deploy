# Gen reference + WBC controller

Operator guide for running `wbc_g1_ctrl` (policy / motors) together with
`wbc_reference_node` (clips + Gen Arc on DDS domain 101).

Unitree wireless: **LT/RT ≈ L2/R2**. Config uses `LT` / `RT` names.

## Build

```bash
scripts/bootstrap_thirdparty.sh
mkdir -p build
cd build
cmake ..
make -j
cd ..
```

Pick the DDS network interface:

- `lo` for local / loopback
- robot NIC (e.g. `eth0`, `enp3s0`) on the real robot

## One-command tmux launcher

```bash
scripts/run_gen_controller.sh
```

Default network is `lo`. Examples:

```bash
scripts/run_gen_controller.sh lo
scripts/run_gen_controller.sh -n eth0
scripts/run_gen_controller.sh --network=eth0
NETWORK=eth0 scripts/run_gen_controller.sh
```

Session `wbc_gen`:

- pane 0 (top): `build/wbc_g1_ctrl --network=<iface>`
- pane 1 (bottom): `build/wbc_reference_node --network=<iface>`

Prefer starting the ref node in **clips** (default) so Standing / Down prep works.
The tmux launcher no longer passes `--mode gen`; to boot Gen directly use
`scripts/run_gen_controller.sh -- --mode gen`, then `LT+down` (floor) or `RT+X`
(back to Standing) before enabling the policy.

`--keep-open` is default: after Ctrl+C, logs stay visible and the pane drops to a shell.
Close panes on exit with `--close-on-exit`. Start detached with `--no-attach`, then:

```bash
tmux attach -t wbc_gen
```

Extra args for the reference node after `--`:

```bash
scripts/run_gen_controller.sh --network eth0 -- --hz 100
```

## Manual launch

```bash
build/wbc_g1_ctrl --network=eth0
build/wbc_reference_node --network=eth0
```

Both must use the same Unitree iface for joystick (`rt/lowstate`). Reference Arc uses
isolated domain **101** (`config/config.yaml` → `reference_dds`).

## How the two processes share the stick

| Who | Listens for |
|-----|-------------|
| **Ctrl** | Prep + enable: `LT+up` / `LT+down` / `RT+A` / `LT+B` |
| **Ref node** | Same prep chords (so Arc matches FixStand / FloorReady), plus all clips/Gen UX |

`RT + A` enables the **policy on ctrl only**. It does **not** play a clip on the ref
node (play is `A` without RT).

## Bring-up sequences

### Standing

1. `LT + D-pad up` — ctrl → FixStand; ref → **Standing** (default idle frame 0)
2. `RT + A` — enable WBC; policy tracks idle
3. Use **Clips** or enter **Gen** (`RT + Y`)

### Floor

1. `LT + D-pad down` — ctrl → FloorReady; ref → **Down** (getup frame 0 only)
2. `RT + A` — enable WBC; still tracking getup frame 0 (getup does **not** auto-play)
3. `RT + D-pad up` — play getup → **Standing** → clips / Gen unlocked

### After liedown

From Standing: `RT + D-pad down` plays liedown → ref enters **Down** again.
Same as floor: only `RT + up` can getup; Gen / browse / idle play stay blocked.

## Reference body states

| State | Entered by | Publishes | Allowed |
|-------|------------|-----------|---------|
| **Standing** | boot (`initial_up: true`), `LT+up`, after getup, after clip ends | idle frame 0 (then browse/play) | browse, play, Gen, liedown |
| **Down** | `LT+down`, after liedown | getup frame 0 | **only** `RT+up` getup |

## Controls — Clips mode (Standing)

| Action | Buttons |
|--------|---------|
| Browse library | `RT + D-pad left / right` |
| Play selected clip | `A` (no RT) |
| Liedown | `RT + D-pad down` |
| Enter Gen | `RT + Y` |
| Back from Gen | `RT + X` |

Returning to clips (from Gen, after getup, or after a browsable clip ends) always
republishes **default idle frame 0**, then you can browse again.

## Controls — Down mode

| Action | Buttons |
|--------|---------|
| Policy enable (ctrl) | `RT + A` |
| Play getup | `RT + D-pad up` |
| Blocked | `A`, browse, Gen, liedown, `LT+up` |

## Controls — Gen mode

Enter with `RT + Y` while Standing. Height teleop works **only** in Gen.

| Action | Buttons |
|--------|---------|
| `vx` forward / back | Left stick **Y** |
| `vy` strafe | Left stick **X** |
| `wz` yaw | Right stick **X** |
| Boost lin + ang velocity | Hold **RT** (× cruise, then clamp to play ranges) |
| Height up / down | D-pad up / down **without** RT |
| Height reset (~0.80 m idle) | `RB + Y` |
| Leave Gen → Standing idle | `RT + X` |

Why `!RT` on height: `RT + up` / `RT + down` are reserved for getup / liedown in clips.

Cruise / boost ranges: `reference_node.cruise_lin_vel_*`, `vel_boost_min` /
`vel_boost_max` in `config/config.yaml`. Height step / limits come from Gen
`play_vel_ranges.height` and `height_step`.

## Ctrl-only (always)

| Action | Buttons |
|--------|---------|
| Passive | `LT + B` |
| FixStand | `LT + D-pad up` |
| FloorReady | `LT + D-pad down` |
| Enable WBC | `RT + A` |

## Logging

Quiet by default: mode switches and clip / getup / liedown events at `info`.
Step heartbeats and PD torque-clip spam need `--verbose`:

```bash
build/wbc_g1_ctrl --verbose --network=eth0
build/wbc_reference_node --verbose --network=eth0
```

## Architecture

Process split, DDS episode sync, and design notes: [`architecture.md`](architecture.md).
