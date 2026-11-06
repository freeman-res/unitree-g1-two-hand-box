#!/usr/bin/env bash
set -euo pipefail

cd "${ARMS_DIR:-$HOME/ARMS}"

DDS_WRAPPER="${DDS_WRAPPER:-/home/unitree/cyclonedds_py_0_10_fix/run_with_fixed_dds.sh}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
NETWORK_INTERFACE="${NETWORK_INTERFACE:-eth0}"
EXECUTE="${EXECUTE:-0}"
TEST_RIGHT_ARM_POSE="${TEST_RIGHT_ARM_POSE:-1}"
ARM_DURATION="${ARM_DURATION:-3.0}"
AUTO="${AUTO:-1}"
AUTO_HOLD_S="${AUTO_HOLD_S:-0.8}"
PROBE_Q="${PROBE_Q:--0.6}"
SNAPSHOT_FILE="${SNAPSHOT_FILE:-right_dex3_motor_probe_start.json}"
LOG_FILE="${LOG_FILE:-right_dex3_motor_probe.log}"

args=(
  -B -m arms.task_right_dex3_motor_probe
  --network-interface "$NETWORK_INTERFACE"
  --arm-duration "$ARM_DURATION"
  --auto-hold-s "$AUTO_HOLD_S"
  --probe-q "$PROBE_Q"
  --snapshot-file "$SNAPSHOT_FILE"
  --log-file "$LOG_FILE"
)

if [[ "$EXECUTE" == "1" ]]; then
  args+=(--execute)
fi

if [[ "$TEST_RIGHT_ARM_POSE" == "1" ]]; then
  args+=(--test-right-arm-pose)
fi

if [[ "$AUTO" == "1" ]]; then
  args+=(--auto)
fi

echo "ARMS_DIR=$(pwd)"
echo "EXECUTE=$EXECUTE"
echo "TEST_RIGHT_ARM_POSE=$TEST_RIGHT_ARM_POSE"
echo "ARM_DURATION=$ARM_DURATION"
echo "AUTO=$AUTO"
echo "AUTO_HOLD_S=$AUTO_HOLD_S"
echo "PROBE_Q=$PROBE_Q"
echo "SNAPSHOT_FILE=$SNAPSHOT_FILE"
echo "LOG_FILE=$LOG_FILE"

exec env -i \
  HOME="$HOME" \
  USER="${USER:-unitree}" \
  LOGNAME="${LOGNAME:-${USER:-unitree}}" \
  PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
  ARMS_DIR="$(pwd)" \
  "$DDS_WRAPPER" "$PYTHON_BIN" "${args[@]}"
