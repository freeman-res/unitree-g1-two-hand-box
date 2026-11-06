from __future__ import annotations

import argparse
import logging
import sys

from .dex3_hand import Dex3HandController
from .dex3_sdk2_backend import UnitreeSdk2Dex3Backend
from .g1_arm_sdk2 import UnitreeSdk2ArmClient
from .safety import validate_hand_side, validate_pose
from .vision_kinematics import CameraPoint, CameraToArmConfig, GraspOffset, yolo_camera_point_to_arm_target


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Task 4: convert YOLO/RealSense cup XYZ to a calibrated arm target and optionally move the G1 arm."
    )
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    parser.add_argument("--camera-x", type=float, required=True, help="YOLO/RealSense X_m from camera frame, meters.")
    parser.add_argument("--camera-y", type=float, required=True, help="YOLO/RealSense Y_m from camera frame, meters.")
    parser.add_argument("--camera-z", type=float, required=True, help="YOLO/RealSense Z_m depth from camera, meters.")
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--arm-calibration", default="config/arm_calibration.json")
    parser.add_argument("--arm-duration", type=float, default=2.0)
    parser.add_argument("--camera-target-x", type=float, default=0.0, help="Camera origin X in the arm calibration target coordinates.")
    parser.add_argument("--camera-target-y", type=float, default=-0.15, help="Camera origin Y in target coordinates. Right of shoulder is negative if Y-left is positive.")
    parser.add_argument("--camera-target-z", type=float, default=0.50, help="Camera origin Z in target coordinates. Keep consistent with calibration labels.")
    parser.add_argument("--camera-x-to-target-y-sign", type=float, default=-1.0)
    parser.add_argument("--camera-y-to-target-z-sign", type=float, default=-1.0)
    parser.add_argument("--camera-pitch-down-deg", type=float, default=0.0, help="Camera downward pitch from horizontal, degrees.")
    parser.add_argument("--grasp-forward-offset", type=float, default=0.0)
    parser.add_argument("--grasp-left-offset", type=float, default=0.0)
    parser.add_argument("--grasp-up-offset", type=float, default=0.0)
    parser.add_argument("--grip", choices=("none", "open", "close"), default="none")
    parser.add_argument("--print-only", action="store_true", help="Only print the computed target; do not move the robot.")
    parser.add_argument("--log-file", default="")
    return parser.parse_args()


def configure_logging(log_file: str) -> None:
    handlers = [logging.StreamHandler()]
    if log_file:
        handlers.append(logging.FileHandler(log_file, encoding="utf-8"))
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s", handlers=handlers)


def run(args: argparse.Namespace) -> int:
    hand = validate_hand_side(args.hand)
    point = CameraPoint(args.camera_x, args.camera_y, args.camera_z)
    transform = CameraToArmConfig(
        camera_target_x_m=args.camera_target_x,
        camera_target_y_m=args.camera_target_y,
        camera_target_z_m=args.camera_target_z,
        camera_x_to_target_y_sign=args.camera_x_to_target_y_sign,
        camera_y_to_target_z_sign=args.camera_y_to_target_z_sign,
        camera_pitch_down_deg=args.camera_pitch_down_deg,
    )
    offset = GraspOffset(
        forward_m=args.grasp_forward_offset,
        left_m=args.grasp_left_offset,
        up_m=args.grasp_up_offset,
    )
    target = yolo_camera_point_to_arm_target(point, transform, offset)
    validate_pose(target)
    logging.info("TASK4 target hand=%s pose=%s print_only=%s grip=%s", hand, target, args.print_only, args.grip)

    if args.print_only:
        print(f"TARGET x={target.x_m:.3f} y={target.y_m:.3f} z={target.z_m:.3f}")
        return 0

    arm = UnitreeSdk2ArmClient(
        network_interface=args.network_interface,
        calibration_path=args.arm_calibration,
        duration_s=args.arm_duration,
    )
    arm_ack = arm.move_to(hand, target)
    logging.info("TASK4 arm result ok=%s message=%s", arm_ack.ok, arm_ack.message)
    if not arm_ack.ok:
        return 2

    if args.grip != "none":
        dex3 = Dex3HandController(UnitreeSdk2Dex3Backend(args.network_interface))
        hand_ack = dex3.open(hand) if args.grip == "open" else dex3.close(hand, "wrap")
        logging.info("TASK4 grip result ok=%s message=%s", hand_ack.ok, hand_ack.message)
        if not hand_ack.ok:
            return 3

    logging.info("TASK4 result=OK")
    return 0


def main() -> int:
    args = parse_args()
    configure_logging(args.log_file)
    try:
        return run(args)
    except Exception as exc:
        logging.exception("TASK4 failed: %s", exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())

