# Real Arm Test Commands

Default run is dry-run:

```bash
cd ~/ARMS
./run_task6_detect_and_reach_fixed_dds.sh
```

Real movement, no gripper close:

```bash
cd ~/ARMS
EXECUTE=1 ./run_task6_detect_and_reach_fixed_dds.sh
```

Return right arm to the saved snapshot:

```bash
cd ~/ARMS
./restore_right_arm_home_fixed_dds.sh
```

Useful safe overrides:

```bash
GRASP_UP_OFFSET=0.20 ./run_task6_detect_and_reach_fixed_dds.sh
OBSTACLE_EARLY_ACCEPT_DISTANCE=0.20 ./run_task6_detect_and_reach_fixed_dds.sh
```

For a real test with a custom log:

```bash
cd ~/ARMS
LOG_FILE=detect_reach_real_test.log EXECUTE=1 ./run_task6_detect_and_reach_fixed_dds.sh
./restore_right_arm_home_fixed_dds.sh
```

Do not add `GRIP=close` until the reach path is verified.
