#!/usr/bin/env bash
# Copy ONNX Runtime bundles into wbc_g1_deploy/thirdparty/
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SRC="${1:-}"

if [[ -z "${SRC}" ]]; then
  echo "Usage: $0 /path/to/unitree_rl_mjlab/deploy" >&2
  exit 1
fi

if [[ ! -d "${SRC}/thirdparty" ]]; then
  echo "Missing thirdparty in: ${SRC}" >&2
  exit 1
fi

for ort in onnxruntime-linux-x64-1.22.0 onnxruntime-linux-aarch64-1.22.0; do
  if [[ -d "${SRC}/thirdparty/${ort}" ]]; then
    rm -rf "${ROOT}/thirdparty/${ort}"
    cp -a "${SRC}/thirdparty/${ort}" "${ROOT}/thirdparty/"
    echo "Installed ${ort}"
  fi
done

echo "Done. thirdparty at ${ROOT}/thirdparty"
