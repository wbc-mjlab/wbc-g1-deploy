#!/usr/bin/env bash
# Regenerate C++ types from include/idl/WbcReference.idl (requires cyclonedds idlc + idlcxx).
#
# Prefer this over the isomorphic EstimatorOutput sed copy once idlc works in CI.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IDL_DIR="${REPO_ROOT}/include/idl"
SRC_IDL_DIR="${REPO_ROOT}/src/idl"
IDL_FILE="${IDL_DIR}/WbcReference.idl"

if ! command -v idlc >/dev/null 2>&1; then
  echo "idlc not found. Install cyclonedds 0.10.x with the C++ backend (idlcxx)." >&2
  exit 1
fi

mkdir -p "${IDL_DIR}" "${SRC_IDL_DIR}"
rm -f "${IDL_DIR}/WbcReference.hpp" "${SRC_IDL_DIR}/WbcReference.cpp"
(
  cd "${IDL_DIR}"
  idlc -l cxx -t -o . WbcReference.idl
)
mv "${IDL_DIR}/WbcReference.cpp" "${SRC_IDL_DIR}/"
echo "Generated ${IDL_DIR}/WbcReference.hpp and ${SRC_IDL_DIR}/WbcReference.cpp"
