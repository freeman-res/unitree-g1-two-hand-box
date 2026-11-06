from __future__ import annotations

import argparse
import logging
import sys
from dataclasses import dataclass

from .dex3_hand import Dex3HandController, DryRunDex3Backend, GRASP_PRESETS, JsonUdpDex3Backend
from .safety import validate_hand_side
from .types import GraspMethod


@dataclass(frozen=True)
class GraspScenario:
    method: GraspMethod
    object_type: str
    description: str


SCENARIOS: dict[GraspMethod, GraspScenario] = {
    "pinch": GraspScenario("pinch", "small/thin object", "Thumb and index finger close toward each other; middle finger stays relaxed."),
    "wrap": GraspScenario("wrap", "cup/cylinder/bottle", "Thumb opposes index and middle fingers; this is the main method for a cup."),
    "side": GraspScenario("side", "flat object or side pickup", "Thumb moves into stronger opposition; index and middle fingers partially close."),
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Task 3: test parameterized grasp methods for the Unitree G1 three-finger hand."
    )
    parser.add_argument("--hand", choices=("left", "right"), required=True)
    parser.add_argument("--method", choices=("open", "pinch", "wrap", "side", "all"), default="all")
    parser.add_argument("--backend", choices=("dry-run", "udp", "sdk2"), default="udp")
    parser.add_argument("--hand-host", default="127.0.0.1")
    parser.add_argument("--hand-port", type=int, default=15002)
    parser.add_argument("--network-interface", default="", help="Unitree SDK2 network interface, for example eth0.")
    parser.add_argument("--timeout", type=float, default=2.0)
    parser.add_argument("--dry-run", action="store_true", help="Alias for --backend dry-run.")
    parser.add_argument("--log-file", default="")
    return parser.parse_args()


def effective_backend(args: argparse.Namespace) -> str:
    return "dry-run" if args.dry_run else args.backend


def configure_logging(log_file: str) -> None:
    handlers: list[logging.Handler] = [logging.StreamHandler()]
    if log_file:
        handlers.append(logging.FileHandler(log_file, encoding="utf-8"))
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s", handlers=handlers)


def make_hand_controller(args: argparse.Namespace) -> Dex3HandController:
    backend = effective_backend(args)
    if backend == "sdk2":
        logging.info("TASK3 hand backend=sdk2 interface=%s", args.network_interface)
        from .dex3_sdk2_backend import UnitreeSdk2Dex3Backend

        return Dex3HandController(UnitreeSdk2Dex3Backend(args.network_interface))
    if backend == "dry-run":
        logging.info("TASK3 hand backend=dry-run")
        return Dex3HandController(DryRunDex3Backend())
    logging.info("TASK3 hand backend=json-udp host=%s port=%d", args.hand_host, args.hand_port)
    return Dex3HandController(JsonUdpDex3Backend(args.hand_host, args.hand_port, args.timeout))


def selected_methods(method: str) -> list[GraspMethod]:
    if method == "all":
        return ["pinch", "wrap", "side"]
    if method == "open":
        return []
    return [method]  # type: ignore[list-item]


def run_task3(args: argparse.Namespace) -> int:
    backend = effective_backend(args)
    logging.info("TASK3 start backend=%s args=%s", backend, vars(args))
    hand = validate_hand_side(args.hand)
    dex3 = make_hand_controller(args)

    methods = selected_methods(args.method)
    logging.info("TASK3 plan hand=%s methods=%s backend=%s", hand, methods if methods else ["open"], backend)

    if args.method == "open":
        open_ack = dex3.open(hand)
        logging.info("TASK3 open-only result ok=%s message=%s", open_ack.ok, open_ack.message)
        return 0 if open_ack.ok else 2

    for index, method in enumerate(methods, start=1):
        scenario = SCENARIOS[method]
        pose = GRASP_PRESETS[method]
        logging.info(
            "TASK3 METHOD %d/%d method=%s object_type=%s description=%s pose=%s",
            index,
            len(methods),
            method,
            scenario.object_type,
            scenario.description,
            pose.as_dict(),
        )

        logging.info("TASK3 METHOD %d/%d start: open before grasp", index, len(methods))
        open_ack = dex3.open(hand)
        logging.info("TASK3 open result ok=%s message=%s", open_ack.ok, open_ack.message)
        if not open_ack.ok:
            logging.error("TASK3 failed while opening hand for method=%s", method)
            return 2

        logging.info("TASK3 METHOD %d/%d start: close method=%s", index, len(methods), method)
        close_ack = dex3.close(hand, method)
        logging.info("TASK3 close result ok=%s message=%s", close_ack.ok, close_ack.message)
        if not close_ack.ok:
            logging.error("TASK3 failed while closing hand for method=%s", method)
            return 3

    logging.info("TASK3 result=OK methods=%s", methods)
    return 0


def main() -> int:
    args = parse_args()
    configure_logging(args.log_file)
    try:
        return run_task3(args)
    except ValueError as exc:
        logging.error("TASK3 invalid input: %s", exc)
        return 1
    except RuntimeError as exc:
        logging.error("TASK3 runtime error: %s", exc)
        return 4


if __name__ == "__main__":
    sys.exit(main())

