from __future__ import annotations

import argparse
import logging
import sys
import time

from .sdk2_common import initialize_sdk2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Check Unitree SDK2 DDS topics without moving the robot.")
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--topic", choices=("arm", "lowstate", "both"), default="both")
    parser.add_argument("--wait-lowstate-s", type=float, default=1.0)
    parser.add_argument("--log-file", default="sdk2_healthcheck.log")
    return parser.parse_args()


def configure_logging(log_file: str) -> None:
    handlers = [logging.StreamHandler()]
    if log_file:
        handlers.append(logging.FileHandler(log_file, encoding="utf-8"))
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s", handlers=handlers)


def run(args: argparse.Namespace) -> int:
    initialize_sdk2(args.network_interface)
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_

    low_state = {"received": False}

    def callback(_msg) -> None:
        low_state["received"] = True

    if args.topic in ("arm", "both"):
        logging.info("SDK2_HEALTH creating publisher topic=rt/arm_sdk type=%s", LowCmd_)
        publisher = ChannelPublisher("rt/arm_sdk", LowCmd_)
        publisher.Init()
        logging.info("SDK2_HEALTH publisher OK topic=rt/arm_sdk")

    if args.topic in ("lowstate", "both"):
        logging.info("SDK2_HEALTH creating subscriber topic=rt/lowstate type=%s", LowState_)
        subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        subscriber.Init(callback, 10)
        logging.info("SDK2_HEALTH subscriber OK topic=rt/lowstate")
        deadline = time.monotonic() + max(0.0, float(args.wait_lowstate_s))
        while time.monotonic() < deadline and not low_state["received"]:
            time.sleep(0.02)
        logging.info("SDK2_HEALTH lowstate_received=%s", low_state["received"])

    logging.info("SDK2_HEALTH result=OK no robot command was sent")
    return 0


def main() -> int:
    args = parse_args()
    configure_logging(args.log_file)
    try:
        return run(args)
    except Exception as exc:
        logging.exception("SDK2_HEALTH failed: %s", exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())
