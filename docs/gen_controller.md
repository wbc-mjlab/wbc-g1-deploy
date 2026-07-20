# Gen reference + WBC controller

Build first:

```bash
scripts/bootstrap_thirdparty.sh
mkdir -p build
cd build
cmake ..
make -j
cd ..
```

Pick the DDS network interface:

- use `lo` for local/loopback runs
- use the robot NIC, for example `eth0` or `enp3s0`, when connected to the real robot

## One-command tmux launcher

```bash
scripts/run_gen_controller.sh
```

The default network is `lo`. The network argument is interchangeable:

```bash
scripts/run_gen_controller.sh lo
scripts/run_gen_controller.sh -n eth0
scripts/run_gen_controller.sh --network=eth0
NETWORK=eth0 scripts/run_gen_controller.sh
```

The script opens and attaches to a tmux session named `wbc_gen`:

- pane 0, top: `build/wbc_g1_ctrl --network=<iface>`
- pane 1, bottom: `build/wbc_reference_node --mode gen --network=<iface>`

`--keep-open` is the default behavior: after Ctrl+C or process exit, the command stops,
logs remain visible, and the pane returns to an interactive shell.
To close panes normally when commands exit:

```bash
scripts/run_gen_controller.sh --close-on-exit
```

To start the session without attaching:

```bash
scripts/run_gen_controller.sh --no-attach
```

Attach later with:

```bash
tmux attach -t wbc_gen
```

Pass extra arguments to `wbc_reference_node` after `--`:

```bash
scripts/run_gen_controller.sh --network eth0 -- --hz 100
```

## Manual tmux launch

```bash
tmux new -s wbc_gen
```

In the first pane:

```bash
build/wbc_g1_ctrl --network=lo
```

Open a second pane:

```bash
tmux split-window -v
```

In the second pane:

```bash
build/wbc_reference_node --mode gen --network=lo
```

## Enable on the robot

- Standing start: `L2 + D-pad Up` for FixStand, then `R2 + A` for WBC tracking.
- Floor start: `L2 + D-pad Down` for FloorReady, then `R2 + A` for WBC tracking, then D-pad Up on the reference node for getup.
- Enter Gen locomotion from the reference node with `RT + Y`.
- Return to clip select with `RT + X`.
- Go Passive with `L2 + B`.
