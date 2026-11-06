#!/usr/bin/env bash
set -euo pipefail

cd "${ARMS_DIR:-$HOME/ARMS}"

DDS_WRAPPER="${DDS_WRAPPER:-/home/unitree/cyclonedds_py_0_10_fix/run_with_fixed_dds.sh}"
NETWORK_INTERFACE="${NETWORK_INTERFACE:-eth0}"
SNAPSHOT_FILE="${SNAPSHOT_FILE:-before_box_two_hand.json}"
HOME_SNAPSHOT_FILE="${HOME_SNAPSHOT_FILE:-baseline_home_snapshot.json}"
PATH_FILE="${PATH_FILE:-runs/box_two_hand_last_path.json}"
RESTORE_DURATION="${RESTORE_DURATION:-5.0}"
HOME_SIDE_OUT_DELTA="${HOME_SIDE_OUT_DELTA:-0.00}"
HAND_SEQUENCE="${HAND_SEQUENCE:-close}"
GRASP_METHOD="${GRASP_METHOD:-wrap}"
LOG_FILE="${LOG_FILE:-restore_both_arms_home_fixed_dds.log}"

echo "ARMS_DIR=$(pwd)"
echo "SNAPSHOT_FILE=$SNAPSHOT_FILE"
echo "HOME_SNAPSHOT_FILE=$HOME_SNAPSHOT_FILE"
echo "PATH_FILE=$PATH_FILE"
echo "RESTORE_DURATION=$RESTORE_DURATION"
echo "HOME_SIDE_OUT_DELTA=$HOME_SIDE_OUT_DELTA"
echo "HAND_SEQUENCE=$HAND_SEQUENCE"
echo "GRASP_METHOD=$GRASP_METHOD"
echo "LOG_FILE=$LOG_FILE"
echo "DDS_ENV=isolated"

exec env -i \
  HOME="$HOME" \
  USER="${USER:-unitree}" \
  LOGNAME="${LOGNAME:-${USER:-unitree}}" \
  PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
  ARMS_DIR="$(pwd)" \
  "$DDS_WRAPPER" python3 -B -m arms.restore_two_hand_path \
  --network-interface "$NETWORK_INTERFACE" \
  --path-file "$PATH_FILE" \
  --snapshot-file "$SNAPSHOT_FILE" \
  --home-snapshot-file "$HOME_SNAPSHOT_FILE" \
  --duration "$RESTORE_DURATION" \
  --home-side-out-delta "$HOME_SIDE_OUT_DELTA" \
  --hand-sequence "$HAND_SEQUENCE" \
  --grasp-method "$GRASP_METHOD" \
  --fallback-direct \
  --log-file "$LOG_FILE"
