# Full Cup Test Runbook

This file is for the last robot session. Run commands from top to bottom and save the logs.

## 0. Copy project from laptop to robot

PowerShell on laptop:

```powershell
scp -r "$env:USERPROFILE\Desktop\ARMS" unitree@192.168.1.103:~/ARMS
```

Robot:

```bash
cd ~/ARMS
python3 -B -m arms.cup_pipeline --help
python3 -B -m arms.move_arm_preset --help
```

## 1. Detect cup with old YOLO/RealSense code

Put cup in front of the robot. This only detects and logs coordinates. It disables old Modbus hand/pregrasp logic.

```bash
cd ~/yolo_cup_project/V3

python3 itog_v3.py eth0 \
  --no-conversation \
  --no-head-emotions \
  --no-coffee-request \
  --no-pregrasp-before-approach \
  --no-arm-reach \
  --save-debug \
  --profile-every 1 \
  --max-time 8 \
  --approach-y-min -1.0 \
  --approach-y-max 1.0 \
  --min-detection-confidence 0.10 \
  --log-file yolo_probe_final.log
```

Check coordinates:

```bash
grep -E "frame=|Reject detection" yolo_probe_final.log | tail -n 30
```

Need `X=... Y=... Z=...`. If the PROFILE line prints only X/Z, use the last known Y manually with `--default-camera-y` in the next step.

## 2. Convert cup coordinates to arm target, no movement

Preferred, if the log has X/Y/Z:

```bash
cd ~/ARMS

python3 -B -m arms.cup_pipeline \
  --mode plan \
  --hand right \
  --yolo-log ~/yolo_cup_project/V3/yolo_probe_final.log \
  --allow-low-confidence \
  --grasp-forward-offset -0.10 \
  --save-json runs/cup_plan_final.json \
  --log-file cup_plan_final.log
```

If the log has only X/Z, add Y manually, example:

```bash
cd ~/ARMS

python3 -B -m arms.cup_pipeline \
  --mode plan \
  --hand right \
  --yolo-log ~/yolo_cup_project/V3/yolo_probe_final.log \
  --allow-low-confidence \
  --default-camera-y -0.064 \
  --grasp-forward-offset -0.10 \
  --save-json runs/cup_plan_final.json \
  --log-file cup_plan_final.log
```

Output must show:

```text
DETECTION {...}
TARGET {"x_m": ..., "y_m": ..., "z_m": ...}
```

## 3. Test old-move mirrored right-arm pregrasp, no fingers

This checks whether the old cup pose family moves the right arm in the useful direction.

```bash
cd ~/ARMS

python3 -B -m arms.cup_pipeline \
  --mode preset-pregrasp \
  --hand right \
  --camera-x 0.042 \
  --camera-y -0.064 \
  --camera-z 0.562 \
  --network-interface eth0 \
  --preset-pregrasp right_cup_side_out_table76 \
  --preset-duration 5.0 \
  --snapshot-file before_cup_preset.json \
  --log-file cup_preset_pregrasp.log
```

Restore after this test:

```bash
cd ~/ARMS

python3 -B -m arms.move_arm_preset \
  --hand right \
  --network-interface eth0 \
  --restore-only \
  --duration 5.0 \
  --snapshot-file before_cup_preset.json \
  --log-file cup_preset_restore.log
```

## 4. Full old-move-style arm sequence, optional grip

First run without gripping:

```bash
cd ~/ARMS

python3 -B -m arms.cup_pipeline \
  --mode preset-grasp \
  --hand right \
  --yolo-log ~/yolo_cup_project/V3/yolo_probe_final.log \
  --allow-low-confidence \
  --default-camera-y -0.064 \
  --network-interface eth0 \
  --grip none \
  --snapshot-file before_cup_preset_full.json \
  --log-file cup_preset_full_no_grip.log
```

If the arm is clear and the wrist is near the cup, run with gripping:

```bash
cd ~/ARMS

python3 -B -m arms.cup_pipeline \
  --mode preset-grasp \
  --hand right \
  --yolo-log ~/yolo_cup_project/V3/yolo_probe_final.log \
  --allow-low-confidence \
  --default-camera-y -0.064 \
  --network-interface eth0 \
  --grip close \
  --snapshot-file before_cup_preset_full_grip.json \
  --log-file cup_preset_full_grip.log
```

## 5. Record current pose as a calibrated IK point

Use this only when the arm is physically in the correct cup-reach pose.

```bash
cd ~/ARMS

python3 -B -m arms.cup_pipeline \
  --mode record-current \
  --hand right \
  --yolo-log ~/yolo_cup_project/V3/yolo_probe_final.log \
  --allow-low-confidence \
  --default-camera-y -0.064 \
  --network-interface eth0 \
  --arm-calibration config/arm_calibration.json \
  --calibration-name cup_final_right \
  --save-json runs/cup_record_final.json \
  --log-file cup_record_final.log
```

## 6. Test calibrated coordinate reach

This uses YOLO coordinates -> target XYZ -> nearest/interpolated calibration point -> SDK2 arm joints.

```bash
cd ~/ARMS

python3 -B -m arms.cup_pipeline \
  --mode reach-calibrated \
  --hand right \
  --yolo-log ~/yolo_cup_project/V3/yolo_probe_final.log \
  --allow-low-confidence \
  --default-camera-y -0.064 \
  --network-interface eth0 \
  --arm-calibration config/arm_calibration.json \
  --arm-duration 4.0 \
  --grip none \
  --log-file cup_reach_calibrated.log
```

Then, only if the hand is positioned safely:

```bash
cd ~/ARMS

python3 -B -m arms.cup_pipeline \
  --mode full-calibrated \
  --hand right \
  --yolo-log ~/yolo_cup_project/V3/yolo_probe_final.log \
  --allow-low-confidence \
  --default-camera-y -0.064 \
  --network-interface eth0 \
  --arm-calibration config/arm_calibration.json \
  --arm-duration 4.0 \
  --log-file cup_full_calibrated.log
```

## 7. Emergency safe commands

Open right hand:

```bash
cd ~/ARMS
python3 -B -m arms.task3_grasp_methods --hand right --method open --backend sdk2 --network-interface eth0 --log-file dex3_open_emergency.log
```

Restore last preset snapshot:

```bash
cd ~/ARMS
python3 -B -m arms.move_arm_preset --hand right --network-interface eth0 --restore-only --duration 5.0 --snapshot-file before_cup_preset_full.json --log-file restore_last_preset.log
```
