# Gen reference + WBC controller

Build and launch for `wbc_g1_ctrl` + `wbc_reference_node`.

**Operator recipes** (stand / floor → Gen → clips): [`operator.md`](operator.md).

Unitree wireless: **LT/RT ≈ L2/R2**.

## Build

```bash
scripts/bootstrap_thirdparty.sh
mkdir -p build
cd build
cmake ..
make -j
cd ..
```

Network: `lo` for loopback, or the robot NIC (e.g. `eth0`).

## One-command tmux launcher

```bash
scripts/run_gen_controller.sh
scripts/run_gen_controller.sh -n eth0
NETWORK=eth0 scripts/run_gen_controller.sh
```

Session `wbc_gen`:

- pane 0: `build/wbc_g1_ctrl --network=<iface>`
- pane 1: `build/wbc_reference_node --network=<iface>` (clips / Standing by default)

Boot Gen directly: `scripts/run_gen_controller.sh -- --mode gen`  
(then `LT+down` for floor, or `RT+X` for Standing, before enabling policy).

`--keep-open` (default): pane returns to a shell after Ctrl+C.  
`--close-on-exit` / `--no-attach` as needed. Attach later: `tmux attach -t wbc_gen`.

Extra ref-node args after `--`:

```bash
scripts/run_gen_controller.sh --network eth0 -- --hz 100
```

## Manual launch

```bash
build/wbc_g1_ctrl --network=eth0
build/wbc_reference_node --network=eth0
```

Same Unitree iface for joystick. Reference Arc: DDS domain **101** (`reference_dds` in config).

## Logging

Quiet by default (mode / clip events). Step heartbeats / PD clip spam:

```bash
build/wbc_g1_ctrl --verbose --network=eth0
build/wbc_reference_node --verbose --network=eth0
```

## See also

- [`operator.md`](operator.md) — stand / floor bring-up, Gen, back to clips
- [`architecture.md`](architecture.md) — process split and DDS sync
