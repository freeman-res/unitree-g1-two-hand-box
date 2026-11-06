#!/usr/bin/env bash
set -euo pipefail

cd /home/unitree/ARMS
source /home/unitree/cyclonedds_py_0_10_fix/env.sh

exec python3 -B -m arms.task5_ik_reach \
  --hand right \
  --network-interface "${IFACE:-eth0}" \
  --yolo-log "${YOLO_LOG:-runs/detect_and_reach_latest.json}" \
  --camera-pitch-down-deg "${CAMERA_PITCH_DOWN_DEG:-42}" \
  --grasp-forward-offset "${GRASP_FORWARD_OFFSET:--0.08}" \
  --grasp-left-offset "${GRASP_LEFT_OFFSET:--0.155}" \
  --grasp-up-offset "${GRASP_UP_OFFSET:-0.16}" \
  --obstacle-check "${OBSTACLE_CHECK:-realsense}" \
  --obstacle-save-json "${OBSTACLE_SAVE_JSON:-runs/obstacle_cloud_latest.json}" \
  --obstacle-clearance "${OBSTACLE_CLEARANCE:-0.12}" \
  --obstacle-extra-clearance "${OBSTACLE_EXTRA_CLEARANCE:-0.03}" \
  --obstacle-samples-per-leg "${OBSTACLE_SAMPLES_PER_LEG:-24}" \
  --obstacle-capture-s "${OBSTACLE_CAPTURE_S:-0.3}" \
  --obstacle-stride-px "${OBSTACLE_STRIDE_PX:-10}" \
  --obstacle-min-depth "${OBSTACLE_MIN_DEPTH:-0.15}" \
  --obstacle-max-depth "${OBSTACLE_MAX_DEPTH:-1.5}" \
  --obstacle-voxel-size "${OBSTACLE_VOXEL_SIZE:-0.025}" \
  --obstacle-max-points "${OBSTACLE_MAX_POINTS:-2500}" \
  --obstacle-exclude-target-radius "${OBSTACLE_EXCLUDE_TARGET_RADIUS:-0.04}" \
  --obstacle-early-accept-distance "${OBSTACLE_EARLY_ACCEPT_DISTANCE:-0.2}" \
  --avoidance-mode "${AVOIDANCE_MODE:-basic}" \
  --avoidance-up-values="${AVOIDANCE_UP_VALUES:-0.12,0.22,0.32}" \
  --avoidance-left-values="${AVOIDANCE_LEFT_VALUES:-0.0,-0.15,0.15}" \
  --avoidance-forward-values="${AVOIDANCE_FORWARD_VALUES:--0.10,-0.20,0.0}" \
  --avoidance-approach-up-values="${AVOIDANCE_APPROACH_UP_VALUES:-0.08,0.12,0.18}" \
  --avoidance-max-error "${AVOIDANCE_MAX_ERROR:-0.035}" \
  --avoidance-iterations "${AVOIDANCE_ITERATIONS:-35}" \
  --avoidance-max-waypoint-tries "${AVOIDANCE_MAX_WAYPOINT_TRIES:-24}" \
  --avoidance-max-candidates "${AVOIDANCE_MAX_CANDIDATES:-28}" \
  --avoidance-replans "${AVOIDANCE_REPLANS:-2}" \
  --avoidance-replan-scale "${AVOIDANCE_REPLAN_SCALE:-1.5}" \
  --snapshot-file "${SNAPSHOT_FILE:-before_detect_reach_final.json}" \
  --save-json "${SAVE_JSON:-runs/ik_reach_latest.json}" \
  --log-file "${LOG_FILE:-ik_reach_fixed_latest.log}" \
  --execute
