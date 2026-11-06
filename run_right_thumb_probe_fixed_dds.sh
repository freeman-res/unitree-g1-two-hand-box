#!/usr/bin/env bash
set -euo pipefail

cd "${ARMS_DIR:-$HOME/ARMS}"

DDS_WRAPPER="${DDS_WRAPPER:-/home/unitree/cyclonedds_py_0_10_fix/run_with_fixed_dds.sh}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
NETWORK_INTERFACE="${NETWORK_INTERFACE:-eth0}"
EXECUTE="${EXECUTE:-0}"
RAISE_RIGHT_ARM="${RAISE_RIGHT_ARM:-0}"
BEND_RIGHT_ELBOW="${BEND_RIGHT_ELBOW:-0}"
TEST_RIGHT_ARM_POSE="${TEST_RIGHT_ARM_POSE:-1}"
RIGHT_ELBOW_RAD="${RIGHT_ELBOW_RAD:-0.95}"
RIGHT_ARM_PRESET="${RIGHT_ARM_PRESET:-right_cup_side_high_table80}"
ARM_DURATION="${ARM_DURATION:-3.0}"
AUTO="${AUTO:-1}"
AUTO_HOLD_S="${AUTO_HOLD_S:-0.8}"
SNAPSHOT_FILE="${SNAPSHOT_FILE:-right_thumb_probe_start.json}"
LOG_FILE="${LOG_FILE:-right_thumb_probe.log}"

args=(
  -B -m arms.task_right_thumb_probe
  --network-interface "$NETWORK_INTERFACE"
  --right-arm-preset "$RIGHT_ARM_PRESET"
  --right-elbow-rad "$RIGHT_ELBOW_RAD"
  --arm-duration "$ARM_DURATION"
  --auto-hold-s "$AUTO_HOLD_S"
  --snapshot-file "$SNAPSHOT_FILE"
  --log-file "$LOG_FILE"
)

if [[ "$EXECUTE" == "1" ]]; then
  args+=(--execute)
fi

if [[ "$RAISE_RIGHT_ARM" == "1" ]]; then
  args+=(--raise-right-arm)
fi

if [[ "$BEND_RIGHT_ELBOW" == "1" ]]; then
  args+=(--bend-right-elbow)
fi

if [[ "$TEST_RIGHT_ARM_POSE" == "1" ]]; then
  args+=(--test-right-arm-pose)
fi

if [[ "$AUTO" == "1" ]]; then
  args+=(--auto)
fi

echo "ARMS_DIR=$(pwd)"
echo "EXECUTE=$EXECUTE"
echo "RAISE_RIGHT_ARM=$RAISE_RIGHT_ARM"
echo "BEND_RIGHT_ELBOW=$BEND_RIGHT_ELBOW"
echo "TEST_RIGHT_ARM_POSE=$TEST_RIGHT_ARM_POSE"
echo "RIGHT_ELBOW_RAD=$RIGHT_ELBOW_RAD"
echo "RIGHT_ARM_PRESET=$RIGHT_ARM_PRESET"
echo "ARM_DURATION=$ARM_DURATION"
echo "AUTO=$AUTO"
echo "AUTO_HOLD_S=$AUTO_HOLD_S"
echo "SNAPSHOT_FILE=$SNAPSHOT_FILE"
echo "LOG_FILE=$LOG_FILE"

exec env -i \
  HOME="$HOME" \
  USER="${USER:-unitree}" \
  LOGNAME="${LOGNAME:-${USER:-unitree}}" \
  PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
  ARMS_DIR="$(pwd)" \
  "$DDS_WRAPPER" "$PYTHON_BIN" "${args[@]}"
