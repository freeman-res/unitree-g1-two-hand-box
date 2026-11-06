#!/usr/bin/env bash
set -euo pipefail

cd "${ARMS_DIR:-$HOME/ARMS}"

DDS_WRAPPER="${DDS_WRAPPER:-/home/unitree/cyclonedds_py_0_10_fix/run_with_fixed_dds.sh}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
NETWORK_INTERFACE="${NETWORK_INTERFACE:-eth0}"
EXECUTE="${EXECUTE:-0}"
AUTO_HOLD_S="${AUTO_HOLD_S:-0.8}"
SNAPSHOT_FILE="${SNAPSHOT_FILE:-right_thumb_squeeze_probe_start.json}"
LOG_FILE="${LOG_FILE:-right_thumb_squeeze_probe.log}"

args=(
  -B -m arms.task_right_thumb_probe
  --network-interface "$NETWORK_INTERFACE"
  --auto-squeeze
  --auto-hold-s "$AUTO_HOLD_S"
  --snapshot-file "$SNAPSHOT_FILE"
  --log-file "$LOG_FILE"
)

if [[ "$EXECUTE" == "1" ]]; then
  args+=(--execute)
fi

echo "ARMS_DIR=$(pwd)"
echo "EXECUTE=$EXECUTE"
echo "AUTO_SQUEEZE=1"
echo "AUTO_HOLD_S=$AUTO_HOLD_S"
echo "SNAPSHOT_FILE=$SNAPSHOT_FILE"
echo "LOG_FILE=$LOG_FILE"
echo "DDS_ENV=isolated"
echo "NOTE=only Dex3 right thumb squeeze poses; arm is not moved"

exec env -i \
  HOME="$HOME" \
  USER="${USER:-unitree}" \
  LOGNAME="${LOGNAME:-${USER:-unitree}}" \
  PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
  ARMS_DIR="$(pwd)" \
  "$DDS_WRAPPER" "$PYTHON_BIN" "${args[@]}"
