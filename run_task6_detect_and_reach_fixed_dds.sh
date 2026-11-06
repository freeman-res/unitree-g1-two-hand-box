#!/usr/bin/env bash
set -euo pipefail

cd "${ARMS_DIR:-$HOME/ARMS}"

DDS_WRAPPER="${DDS_WRAPPER:-/home/unitree/cyclonedds_py_0_10_fix/run_with_fixed_dds.sh}"
NETWORK_INTERFACE="${NETWORK_INTERFACE:-eth0}"

# Vision and target transform.
YOLO_MAX_TIME="${YOLO_MAX_TIME:-8}"
YOLO_MIN_CONFIDENCE="${YOLO_MIN_CONFIDENCE:-0.05}"
YOLO_TARGET_CLASS="${YOLO_TARGET_CLASS:-cup}"
YOLO_MODEL="${YOLO_MODEL:-}"
STOP_DISTANCE="${STOP_DISTANCE:-0.60}"
CAMERA_PITCH_DOWN_DEG="${CAMERA_PITCH_DOWN_DEG:-42}"

# Grasp target offset in torso coordinates.
GRASP_FORWARD_OFFSET="${GRASP_FORWARD_OFFSET:--0.08}"
GRASP_LEFT_OFFSET="${GRASP_LEFT_OFFSET:--0.155}"
GRASP_UP_OFFSET="${GRASP_UP_OFFSET:-0.16}"

# Obstacle clearance. Required distance = clearance + extra clearance.
OBSTACLE_CLEARANCE="${OBSTACLE_CLEARANCE:-0.12}"
OBSTACLE_EXTRA_CLEARANCE="${OBSTACLE_EXTRA_CLEARANCE:-0.03}"
OBSTACLE_EARLY_ACCEPT_DISTANCE="${OBSTACLE_EARLY_ACCEPT_DISTANCE:-0.20}"
OBSTACLE_STRIDE_PX="${OBSTACLE_STRIDE_PX:-10}"
OBSTACLE_CAPTURE_S="${OBSTACLE_CAPTURE_S:-0.3}"
OBSTACLE_EXCLUDE_TARGET_RADIUS="${OBSTACLE_EXCLUDE_TARGET_RADIUS:-0.04}"

# Route search limits.
AVOIDANCE_REPLANS="${AVOIDANCE_REPLANS:-2}"
AVOIDANCE_REPLAN_SCALE="${AVOIDANCE_REPLAN_SCALE:-1.5}"
AVOIDANCE_ITERATIONS="${AVOIDANCE_ITERATIONS:-35}"
AVOIDANCE_MAX_WAYPOINT_TRIES="${AVOIDANCE_MAX_WAYPOINT_TRIES:-24}"
AVOIDANCE_MAX_CANDIDATES="${AVOIDANCE_MAX_CANDIDATES:-28}"
AVOIDANCE_UP_VALUES="${AVOIDANCE_UP_VALUES:-0.12,0.22,0.32}"
AVOIDANCE_LEFT_VALUES="${AVOIDANCE_LEFT_VALUES:-0.0,-0.15,0.15}"
AVOIDANCE_FORWARD_VALUES="${AVOIDANCE_FORWARD_VALUES:--0.10,-0.20,0.0}"
AVOIDANCE_APPROACH_UP_VALUES="${AVOIDANCE_APPROACH_UP_VALUES:-0.08,0.12,0.18}"

# Execution controls. Default is dry-run. Use EXECUTE=1 only for a real arm test.
EXECUTE="${EXECUTE:-0}"
GRIP="${GRIP:-none}"
LOG_FILE="${LOG_FILE:-detect_reach_fixed_dds.log}"
SNAPSHOT_FILE="${SNAPSHOT_FILE:-before_detect_and_reach.json}"

args=(
  -B -m arms.task6_detect_and_reach
  --hand right
  --network-interface "$NETWORK_INTERFACE"
  --yolo-max-time "$YOLO_MAX_TIME"
  --yolo-min-confidence "$YOLO_MIN_CONFIDENCE"
  --yolo-target-class "$YOLO_TARGET_CLASS"
  --stop-distance "$STOP_DISTANCE"
  --camera-pitch-down-deg "$CAMERA_PITCH_DOWN_DEG"
  --grasp-forward-offset "$GRASP_FORWARD_OFFSET"
  --grasp-left-offset "$GRASP_LEFT_OFFSET"
  --grasp-up-offset "$GRASP_UP_OFFSET"
  --obstacle-check realsense
  --obstacle-clearance "$OBSTACLE_CLEARANCE"
  --obstacle-extra-clearance "$OBSTACLE_EXTRA_CLEARANCE"
  --obstacle-early-accept-distance "$OBSTACLE_EARLY_ACCEPT_DISTANCE"
  --obstacle-stride-px "$OBSTACLE_STRIDE_PX"
  --obstacle-capture-s "$OBSTACLE_CAPTURE_S"
  --obstacle-exclude-target-radius "$OBSTACLE_EXCLUDE_TARGET_RADIUS"
  --avoidance-mode basic
  --avoidance-replans "$AVOIDANCE_REPLANS"
  --avoidance-replan-scale "$AVOIDANCE_REPLAN_SCALE"
  --avoidance-iterations "$AVOIDANCE_ITERATIONS"
  --avoidance-max-waypoint-tries "$AVOIDANCE_MAX_WAYPOINT_TRIES"
  --avoidance-max-candidates "$AVOIDANCE_MAX_CANDIDATES"
  --avoidance-up-values="$AVOIDANCE_UP_VALUES"
  --avoidance-left-values="$AVOIDANCE_LEFT_VALUES"
  --avoidance-forward-values="$AVOIDANCE_FORWARD_VALUES"
  --avoidance-approach-up-values="$AVOIDANCE_APPROACH_UP_VALUES"
  --snapshot-file "$SNAPSHOT_FILE"
  --log-file "$LOG_FILE"
)

if [[ -n "$YOLO_MODEL" ]]; then
  args+=(--yolo-model "$YOLO_MODEL")
fi

if [[ "$EXECUTE" == "1" ]]; then
  args+=(--execute)
fi

if [[ "$GRIP" != "none" ]]; then
  args+=(--grip "$GRIP")
fi

echo "ARMS_DIR=$(pwd)"
echo "LOG_FILE=$LOG_FILE"
echo "SNAPSHOT_FILE=$SNAPSHOT_FILE"
echo "EXECUTE=$EXECUTE"
echo "GRIP=$GRIP"
echo "YOLO_TARGET_CLASS=$YOLO_TARGET_CLASS"
echo "YOLO_MODEL=$YOLO_MODEL"

exec env -i \
  HOME="$HOME" \
  USER="${USER:-unitree}" \
  LOGNAME="${LOGNAME:-${USER:-unitree}}" \
  PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
  ARMS_DIR="$(pwd)" \
  "$DDS_WRAPPER" python3 "${args[@]}"
