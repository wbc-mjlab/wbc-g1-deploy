#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/run_gen_controller.sh [--network <iface>] [--session <name>] [--no-attach] [--keep-open|--close-on-exit] [-- <extra reference args>]
  scripts/run_gen_controller.sh <iface>
  NETWORK=<iface> scripts/run_gen_controller.sh

Examples:
  scripts/run_gen_controller.sh
  scripts/run_gen_controller.sh --network lo
  scripts/run_gen_controller.sh -n eth0
  scripts/run_gen_controller.sh --no-attach
  scripts/run_gen_controller.sh --keep-open       # default
  scripts/run_gen_controller.sh --close-on-exit
  NETWORK=enp3s0 scripts/run_gen_controller.sh -- --hz 100

Starts and attaches to a tmux session with:
  pane 0: build/wbc_g1_ctrl --network=<iface>
  pane 1: build/wbc_reference_node --mode gen --network=<iface>

By default, --keep-open returns each pane to a shell after Ctrl+C/process exit.
EOF
}

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
network="${NETWORK:-lo}"
session="${SESSION:-wbc_gen}"
attach=true
keep_open=true
extra_ref_args=()

while (($#)); do
  case "$1" in
    -n|--network)
      [[ $# -ge 2 ]] || { echo "missing value for $1" >&2; usage >&2; exit 2; }
      network="$2"
      shift 2
      ;;
    --network=*)
      network="${1#*=}"
      shift
      ;;
    --session)
      [[ $# -ge 2 ]] || { echo "missing value for $1" >&2; usage >&2; exit 2; }
      session="$2"
      shift 2
      ;;
    --session=*)
      session="${1#*=}"
      shift
      ;;
    --no-attach)
      attach=false
      shift
      ;;
    --close-on-exit)
      keep_open=false
      shift
      ;;
    --keep-open|--remain-on-exit)
      keep_open=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      extra_ref_args=("$@")
      break
      ;;
    *)
      if [[ "$network" == "lo" ]]; then
        network="$1"
        shift
      else
        extra_ref_args+=("$1")
        shift
      fi
      ;;
  esac
done

ctrl_bin="$repo_root/build/wbc_g1_ctrl"
ref_bin="$repo_root/build/wbc_reference_node"
for bin in "$ctrl_bin" "$ref_bin"; do
  if [[ ! -x "$bin" ]]; then
    echo "error: missing executable: $bin" >&2
    echo "build first: mkdir -p build && cd build && cmake .. && make -j" >&2
    exit 1
  fi
done

if ! command -v tmux >/dev/null 2>&1; then
  echo "error: tmux is required for this launcher" >&2
  exit 1
fi

if tmux has-session -t "$session" 2>/dev/null; then
  if [[ "$attach" == true ]]; then
    echo "tmux session '$session' already exists; attaching."
    exec tmux attach -t "$session"
  else
    echo "error: tmux session '$session' already exists" >&2
    echo "attach with: tmux attach -t $session" >&2
    exit 1
  fi
fi

ctrl_cmd=( "$ctrl_bin" "--network=$network" )
ref_cmd=( "$ref_bin" --mode gen "--network=$network" "${extra_ref_args[@]}" )
shell_cmd=( "${SHELL:-/bin/bash}" -l )
printf -v ctrl_cmd_q '%q ' "${ctrl_cmd[@]}"
printf -v ref_cmd_q '%q ' "${ref_cmd[@]}"
printf -v shell_cmd_q '%q ' "${shell_cmd[@]}"

if [[ "$keep_open" == true ]]; then
  tmux new-session -d -s "$session" -c "$repo_root" "$shell_cmd_q"
  tmux set-window-option -t "$session:0" remain-on-exit off >/dev/null
  tmux split-window -v -t "$session:0" -c "$repo_root" "$shell_cmd_q"
  tmux send-keys -t "$session:0.0" "$ctrl_cmd_q" C-m
  tmux send-keys -t "$session:0.1" "$ref_cmd_q" C-m
else
  tmux new-session -d -s "$session" -c "$repo_root" "$ctrl_cmd_q"
  tmux set-window-option -t "$session:0" remain-on-exit off >/dev/null
  tmux split-window -v -t "$session:0" -c "$repo_root" "$ref_cmd_q"
fi
tmux select-pane -t "$session:0.0"

echo "Started tmux session '$session' on network '$network'."
if [[ "$attach" == true ]]; then
  exec tmux attach -t "$session"
else
  echo "Attach with: tmux attach -t $session"
fi
