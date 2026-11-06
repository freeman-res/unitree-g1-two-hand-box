from __future__ import annotations

import argparse
import json
import logging
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from .vision_kinematics import CameraPoint


@dataclass(frozen=True)
class ServerBoxDetection:
    camera_point: CameraPoint
    confidence: float
    class_name: str
    frame: int
    bbox: Tuple[float, float, float, float]
    center_px: Tuple[float, float]
    source: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Send RGB frames to Agrohab server and read the final box detection.")
    parser.add_argument("--server-url", default="http://192.168.10.179:8000", help="Agrohab app URL with /stream and /context.")
    parser.add_argument("--target-class", default="box")
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--camera-mode", choices=("realsense", "opencv"), default="realsense")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--color-fps", type=int, default=15)
    parser.add_argument("--depth-width", type=int, default=640)
    parser.add_argument("--depth-height", type=int, default=480)
    parser.add_argument("--depth-fps", type=int, default=15)
    parser.add_argument("--max-time", type=float, default=8.0)
    parser.add_argument("--fps", type=float, default=5.0)
    parser.add_argument("--min-confidence", type=float, default=0.25)
    parser.add_argument("--box-distance", type=float, default=0.60, help="Fallback distance when RealSense depth is unavailable.")
    parser.add_argument("--depth-roi-ratio", type=float, default=0.45)
    parser.add_argument("--depth-stat", choices=("median", "nearest_percentile"), default="nearest_percentile")
    parser.add_argument("--depth-percentile", type=float, default=20.0)
    parser.add_argument("--min-depth", type=float, default=0.15)
    parser.add_argument("--max-depth", type=float, default=2.00)
    parser.add_argument("--bbox-edge-margin-px", type=float, default=0.0, help="Reject detections clipped by the image border. Use 0 to disable.")
    parser.add_argument("--max-object-last-seen-s", type=float, default=0.75, help="Reject tracked objects not seen in the current/fresh frame.")
    parser.add_argument(
        "--depth-fallback",
        choices=("none", "center_nearest"),
        default="center_nearest",
        help="Fallback to the nearest central RealSense depth cluster when server YOLO has no fresh usable box.",
    )
    parser.add_argument("--context-timeout", type=float, default=1.0)
    parser.add_argument(
        "--context-frame-match",
        choices=("exact", "at_least"),
        default="exact",
        help=(
            "Use exact to accept only the /context produced for the robot frame "
            "we just sent. at_least is the old shared-stream behavior."
        ),
    )
    parser.add_argument("--prefer-server-3d", action="store_true", help="Use server coords_3d when present. Default is robot depth inside bbox.")
    parser.add_argument("--fx", type=float, default=914.76)
    parser.add_argument("--fy", type=float, default=914.55)
    parser.add_argument("--cx", type=float, default=653.72)
    parser.add_argument("--cy", type=float, default=382.25)
    parser.add_argument("--save-debug", action="store_true")
    parser.add_argument("--sent-image", default="runs/server_box_sent_latest.jpg")
    parser.add_argument("--debug-image", default="runs/server_box_latest.jpg")
    parser.add_argument("--heatmap-image", default="runs/server_box_depth_heatmap_latest.jpg")
    parser.add_argument("--save-json", default="runs/server_box_latest.json")
    parser.add_argument("--log-file", default="server_box_detector.log")
    return parser.parse_args()


def configure_logging(log_file: str) -> None:
    handlers = [logging.StreamHandler()]
    if log_file:
        handlers.append(logging.FileHandler(log_file, encoding="utf-8"))
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s: %(message)s", handlers=handlers)


def run(args: argparse.Namespace) -> int:
    detection = detect_box_from_server(args)
    if detection is None:
        logging.error("SERVER_BOX no detection target_class=%s server=%s", args.target_class, args.server_url)
        return 2
    save_detection(args.save_json, detection)
    print_detection(detection)
    return 0


def detect_box_from_server(args: argparse.Namespace) -> Optional[ServerBoxDetection]:
    if args.camera_mode == "realsense":
        return detect_box_from_server_realsense(args)
    return detect_box_from_server_opencv(args)


def detect_box_from_server_realsense(args: argparse.Namespace) -> Optional[ServerBoxDetection]:
    import pyrealsense2 as rs

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, int(args.width), int(args.height), rs.format.bgr8, int(args.color_fps))
    config.enable_stream(rs.stream.depth, int(args.depth_width), int(args.depth_height), rs.format.z16, int(args.depth_fps))
    align = rs.align(rs.stream.color)

    logging.info(
        "SERVER_BOX RealSense start color=%dx%d@%d depth=%dx%d@%d",
        args.width,
        args.height,
        args.color_fps,
        args.depth_width,
        args.depth_height,
        args.depth_fps,
    )
    profile = pipeline.start(config)
    try:
        color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
        intr = color_stream.get_intrinsics()
        depth_scale = float(profile.get_device().first_depth_sensor().get_depth_scale())
        logging.info("SERVER_BOX intrinsics fx=%.2f fy=%.2f cx=%.2f cy=%.2f", intr.fx, intr.fy, intr.ppx, intr.ppy)
        logging.info("SERVER_BOX depth_scale=%.6f", depth_scale)

        start_s = time.monotonic()
        frame_index = 0
        best: Optional[ServerBoxDetection] = None
        delay_s = 1.0 / max(0.1, float(args.fps))

        while time.monotonic() - start_s <= float(args.max_time):
            frames = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()
            if not color_frame or not depth_frame:
                time.sleep(0.03)
                continue
            frame_index += 1

            import numpy as np

            color = np.asanyarray(color_frame.get_data())
            if args.save_debug:
                save_sent_image(args.sent_image, color)
            server_seq = post_frame(args.server_url, color)
            context = read_fresh_context(
                args.server_url,
                server_seq,
                args.context_timeout,
                frame_match=args.context_frame_match,
            )
            try:
                detection = detection_from_context(
                    context,
                    target_class=args.target_class,
                    min_confidence=args.min_confidence,
                    distance_m=args.box_distance,
                    fx=intr.fx,
                    fy=intr.fy,
                    cx=intr.ppx,
                    cy=intr.ppy,
                    frame_index=frame_index,
                    depth_frame=depth_frame,
                    depth_roi_ratio=args.depth_roi_ratio,
                    depth_stat=args.depth_stat,
                    depth_percentile=args.depth_percentile,
                    min_depth=args.min_depth,
                    max_depth=args.max_depth,
                    bbox_edge_margin_px=args.bbox_edge_margin_px,
                    max_object_last_seen_s=args.max_object_last_seen_s,
                    prefer_server_3d=args.prefer_server_3d,
                )
            except ValueError as exc:
                logging.warning("SERVER_BOX skip frame=%d reason=%s", frame_index, exc)
                detection = None
            if detection is None and args.depth_fallback == "center_nearest":
                detection = depth_fallback_detection(
                    depth_frame,
                    target_class=args.target_class,
                    frame_index=frame_index,
                    fx=intr.fx,
                    fy=intr.fy,
                    cx=intr.ppx,
                    cy=intr.ppy,
                    min_confidence=args.min_confidence,
                    min_depth=args.min_depth,
                    max_depth=args.max_depth,
                    depth_percentile=args.depth_percentile,
                )
            if detection is not None:
                best = detection if best is None or detection.confidence > best.confidence else best
                log_detection(detection, context)
                if args.save_debug:
                    save_debug_image(args.debug_image, color, detection)
                    save_depth_heatmap(args.heatmap_image, depth_frame, depth_scale, detection, args.min_depth, args.max_depth)
                return detection
            time.sleep(delay_s)
        return best
    finally:
        pipeline.stop()


def detect_box_from_server_opencv(args: argparse.Namespace) -> Optional[ServerBoxDetection]:
    import cv2

    capture = cv2.VideoCapture(int(args.camera_index))
    try:
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, int(args.width))
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, int(args.height))
        start_s = time.monotonic()
        frame_index = 0
        best: Optional[ServerBoxDetection] = None
        delay_s = 1.0 / max(0.1, float(args.fps))

        while time.monotonic() - start_s <= float(args.max_time):
            ok, frame = capture.read()
            if not ok or frame is None:
                time.sleep(0.03)
                continue
            frame_index += 1
            if args.save_debug:
                save_sent_image(args.sent_image, frame)
            server_seq = post_frame(args.server_url, frame)
            context = read_fresh_context(
                args.server_url,
                server_seq,
                args.context_timeout,
                frame_match=args.context_frame_match,
            )
            detection = detection_from_context(
                context,
                target_class=args.target_class,
                min_confidence=args.min_confidence,
                distance_m=args.box_distance,
                fx=args.fx,
                fy=args.fy,
                cx=args.cx,
                cy=args.cy,
                frame_index=frame_index,
                image_width=frame.shape[1],
                image_height=frame.shape[0],
                bbox_edge_margin_px=args.bbox_edge_margin_px,
                max_object_last_seen_s=args.max_object_last_seen_s,
                prefer_server_3d=args.prefer_server_3d,
            )
            if detection is not None:
                best = detection if best is None or detection.confidence > best.confidence else best
                log_detection(detection, context)
                if args.save_debug:
                    save_debug_image(args.debug_image, frame, detection)
                return detection
            time.sleep(delay_s)
        return best
    finally:
        capture.release()


def post_frame(server_url: str, frame) -> Optional[int]:
    import cv2

    ok, encoded = cv2.imencode(".jpg", frame)
    if not ok:
        raise RuntimeError("Could not encode camera frame as JPEG")
    request = urllib.request.Request(
        _join_url(server_url, "/stream"),
        data=encoded.tobytes(),
        headers={"Content-Type": "image/jpeg"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=2.0) as response:
        payload = json.loads(response.read().decode("utf-8"))
    if not payload.get("ok"):
        raise RuntimeError(f"Server /stream rejected frame: {payload}")
    seq = payload.get("seq")
    return int(seq) if seq is not None else None


def read_context(server_url: str) -> Dict[str, object]:
    with urllib.request.urlopen(_join_url(server_url, "/context"), timeout=2.0) as response:
        return json.loads(response.read().decode("utf-8"))


def read_fresh_context(
    server_url: str,
    min_frame_seq: Optional[int],
    timeout_s: float,
    *,
    frame_match: str = "exact",
) -> Dict[str, object]:
    deadline = time.monotonic() + max(0.05, float(timeout_s))
    last_context: Dict[str, object] = {}
    last_matching_context: Dict[str, object] = {}
    while time.monotonic() <= deadline:
        last_context = read_context(server_url)
        frame_seq = _optional_int(last_context.get("frame_seq"))
        if min_frame_seq is None:
            return last_context
        if frame_seq is not None:
            if frame_match == "exact":
                if frame_seq == min_frame_seq:
                    last_matching_context = last_context
                elif frame_seq > min_frame_seq and last_matching_context:
                    return last_matching_context
            if frame_match == "at_least" and frame_seq >= min_frame_seq:
                return last_context
        time.sleep(0.03)
    if last_matching_context:
        return last_matching_context
    logging.warning(
        "SERVER_BOX stale/foreign context expected_seq=%s last_seq=%s frame_match=%s",
        min_frame_seq,
        _optional_int(last_context.get("frame_seq")),
        frame_match,
    )
    if frame_match == "exact":
        return {}
    return last_context


def detection_from_context(
    context: Dict[str, object],
    *,
    target_class: str,
    min_confidence: float,
    distance_m: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    frame_index: int,
    depth_frame=None,
    depth_roi_ratio: float = 0.45,
    depth_stat: str = "nearest_percentile",
    depth_percentile: float = 20.0,
    min_depth: float = 0.15,
    max_depth: float = 2.00,
    image_width: Optional[int] = None,
    image_height: Optional[int] = None,
    bbox_edge_margin_px: float = 0.0,
    max_object_last_seen_s: float = 0.75,
    prefer_server_3d: bool = False,
) -> Optional[ServerBoxDetection]:
    objects = context.get("objects")
    if not isinstance(objects, list):
        return None

    wanted = target_class.strip().lower()
    best_obj: Optional[Dict[str, object]] = None
    best_conf = -1.0
    for obj in objects:
        if not isinstance(obj, dict):
            continue
        class_name = str(obj.get("class_name") or obj.get("class") or "").lower()
        yolo_class = str(obj.get("yolo_class") or "").lower()
        if wanted not in (class_name, yolo_class):
            continue
        last_seen_ago = obj.get("last_seen_ago")
        if last_seen_ago is not None and float(last_seen_ago) > float(max_object_last_seen_s):
            continue
        conf = float(obj.get("conf") or 0.0)
        if conf < min_confidence or conf <= best_conf:
            continue
        bbox = obj.get("bbox_2d")
        if not isinstance(bbox, list) or len(bbox) != 4:
            continue
        best_obj = obj
        best_conf = conf

    if best_obj is None:
        return None

    x1, y1, x2, y2 = [float(value) for value in best_obj["bbox_2d"]]
    if image_width is None and depth_frame is not None:
        image_width = int(depth_frame.get_width())
    if image_height is None and depth_frame is not None:
        image_height = int(depth_frame.get_height())
    if float(bbox_edge_margin_px) > 0.0 and image_width is not None and image_height is not None and is_bbox_clipped(
        (x1, y1, x2, y2),
        int(image_width),
        int(image_height),
        float(bbox_edge_margin_px),
    ):
        raise ValueError(
            f"bbox touches image border bbox={[round(v, 1) for v in (x1, y1, x2, y2)]} "
            f"image={image_width}x{image_height} margin={bbox_edge_margin_px:g}px"
        )
    u = (x1 + x2) * 0.5
    v = (y1 + y2) * 0.5
    coords_3d = best_obj.get("coords_3d")
    if prefer_server_3d and isinstance(coords_3d, list) and len(coords_3d) >= 3:
        point = CameraPoint(float(coords_3d[0]), float(coords_3d[1]), float(coords_3d[2]))
        source = "context_coords_3d"
    elif depth_frame is not None:
        z_m, samples = depth_in_bbox(
            depth_frame,
            (x1, y1, x2, y2),
            roi_ratio=depth_roi_ratio,
            stat=depth_stat,
            percentile=depth_percentile,
            min_depth=min_depth,
            max_depth=max_depth,
        )
        point = CameraPoint(
            x_m=(u - float(cx)) * z_m / float(fx),
            y_m=(v - float(cy)) * z_m / float(fy),
            z_m=z_m,
        )
        source = f"realsense_depth_bbox stat={depth_stat} percentile={depth_percentile:g} samples={samples}"
    else:
        z_m = float(distance_m)
        point = CameraPoint(
            x_m=(u - float(cx)) * z_m / float(fx),
            y_m=(v - float(cy)) * z_m / float(fy),
            z_m=z_m,
        )
        source = "bbox_center_plus_manual_distance"

    return ServerBoxDetection(
        camera_point=point,
        confidence=best_conf,
        class_name=str(best_obj.get("class_name") or best_obj.get("class") or target_class),
        frame=int(context.get("frame_seq") or frame_index),
        bbox=(x1, y1, x2, y2),
        center_px=(u, v),
        source=source,
    )


def median_depth_in_bbox(
    depth_frame,
    bbox: Tuple[float, float, float, float],
    *,
    roi_ratio: float,
    min_depth: float,
    max_depth: float,
) -> Tuple[float, int]:
    return depth_in_bbox(
        depth_frame,
        bbox,
        roi_ratio=roi_ratio,
        stat="median",
        percentile=50.0,
        min_depth=min_depth,
        max_depth=max_depth,
    )


def depth_in_bbox(
    depth_frame,
    bbox: Tuple[float, float, float, float],
    *,
    roi_ratio: float,
    stat: str,
    percentile: float,
    min_depth: float,
    max_depth: float,
) -> Tuple[float, int]:
    values: List[float] = []
    for u, v in bbox_roi_pixels(depth_frame.get_width(), depth_frame.get_height(), bbox, roi_ratio):
        depth_m = float(depth_frame.get_distance(u, v))
        if min_depth <= depth_m <= max_depth:
            values.append(depth_m)
    if not values:
        raise ValueError(f"No valid depth inside bbox={bbox}")
    values.sort()
    if stat == "nearest_percentile":
        pct = max(0.0, min(100.0, float(percentile)))
        index = int(round((len(values) - 1) * pct / 100.0))
        return values[index], len(values)
    mid = len(values) // 2
    if len(values) % 2:
        return values[mid], len(values)
    return (values[mid - 1] + values[mid]) * 0.5, len(values)


def depth_fallback_detection(
    depth_frame,
    *,
    target_class: str,
    frame_index: int,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    min_confidence: float,
    min_depth: float,
    max_depth: float,
    depth_percentile: float,
) -> Optional[ServerBoxDetection]:
    width = int(depth_frame.get_width())
    height = int(depth_frame.get_height())
    roi = (
        int(width * 0.20),
        int(height * 0.12),
        int(width * 0.80),
        int(height * 0.88),
    )
    values: List[float] = []
    stride = 4
    for v in range(roi[1], roi[3], stride):
        for u in range(roi[0], roi[2], stride):
            depth_m = float(depth_frame.get_distance(u, v))
            if min_depth <= depth_m <= max_depth:
                values.append(depth_m)
    if len(values) < 80:
        logging.info("SERVER_BOX depth fallback skipped: too few valid depths=%d", len(values))
        return None

    values.sort()
    pct = max(0.0, min(100.0, float(depth_percentile)))
    z_ref = values[int(round((len(values) - 1) * pct / 100.0))]
    band = max(0.045, z_ref * 0.10)
    us: List[int] = []
    vs: List[int] = []
    zs: List[float] = []
    for v in range(roi[1], roi[3], stride):
        for u in range(roi[0], roi[2], stride):
            depth_m = float(depth_frame.get_distance(u, v))
            if z_ref - 0.025 <= depth_m <= z_ref + band:
                us.append(u)
                vs.append(v)
                zs.append(depth_m)
    if len(zs) < 80:
        logging.info("SERVER_BOX depth fallback skipped: too few cluster depths=%d z_ref=%.3f", len(zs), z_ref)
        return None

    x1 = float(max(0, min(us) - stride))
    y1 = float(max(0, min(vs) - stride))
    x2 = float(min(width - 1, max(us) + stride))
    y2 = float(min(height - 1, max(vs) + stride))
    u = (x1 + x2) * 0.5
    v = (y1 + y2) * 0.5
    z_m = sorted(zs)[len(zs) // 2]
    point = CameraPoint(
        x_m=(u - float(cx)) * z_m / float(fx),
        y_m=(v - float(cy)) * z_m / float(fy),
        z_m=z_m,
    )
    logging.info(
        "SERVER_BOX depth fallback selected frame=%d X=%.3f Y=%.3f Z=%.3f bbox=%s samples=%d z_ref=%.3f",
        frame_index,
        point.x_m,
        point.y_m,
        point.z_m,
        [round(value, 1) for value in (x1, y1, x2, y2)],
        len(zs),
        z_ref,
    )
    return ServerBoxDetection(
        camera_point=point,
        confidence=max(float(min_confidence), 0.05),
        class_name=target_class,
        frame=frame_index,
        bbox=(x1, y1, x2, y2),
        center_px=(u, v),
        source=f"realsense_depth_center_fallback samples={len(zs)} z_ref={z_ref:.3f}",
    )


def bbox_roi_pixels(width: int, height: int, bbox: Tuple[float, float, float, float], roi_ratio: float) -> List[Tuple[int, int]]:
    x1, y1, x2, y2 = bbox
    cx = (x1 + x2) * 0.5
    cy = (y1 + y2) * 0.5
    roi_w = max(4.0, (x2 - x1) * float(roi_ratio))
    roi_h = max(4.0, (y2 - y1) * float(roi_ratio))
    rx1 = max(0, min(width - 1, int(cx - roi_w * 0.5)))
    rx2 = max(0, min(width, int(cx + roi_w * 0.5)))
    ry1 = max(0, min(height - 1, int(cy - roi_h * 0.5)))
    ry2 = max(0, min(height, int(cy + roi_h * 0.5)))
    return [(u, v) for v in range(ry1, ry2) for u in range(rx1, rx2)]


def is_bbox_clipped(bbox: Tuple[float, float, float, float], width: int, height: int, margin_px: float) -> bool:
    if margin_px < 0:
        return False
    x1, y1, x2, y2 = bbox
    margin = float(margin_px)
    # Top-clipped boxes are acceptable for the table/box setup: the model may see
    # either the table plus the box front or the full box. Keep rejecting side and
    # bottom clipping, which corrupts lateral center or hides the near grasp face.
    return x1 <= margin or x2 >= float(width) - margin or y2 >= float(height) - margin


def log_detection(detection: ServerBoxDetection, context: Dict[str, object]) -> None:
    logging.info(
        "SERVER_BOX frame=%d context_seq=%s class=%s conf=%.3f X=%.3f Y=%.3f Z=%.3f bbox=%s source=%s",
        detection.frame,
        context.get("frame_seq"),
        detection.class_name,
        detection.confidence,
        detection.camera_point.x_m,
        detection.camera_point.y_m,
        detection.camera_point.z_m,
        [round(value, 1) for value in detection.bbox],
        detection.source,
    )


def save_debug_image(path: str, frame, detection: ServerBoxDetection) -> None:
    import cv2

    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    x1, y1, x2, y2 = [int(value) for value in detection.bbox]
    u, v = [int(value) for value in detection.center_px]
    image = frame.copy()
    cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.circle(image, (u, v), 5, (0, 0, 255), -1)
    text = (
        f"{detection.class_name} conf={detection.confidence:.2f} "
        f"X={detection.camera_point.x_m:+.2f} Y={detection.camera_point.y_m:+.2f} Z={detection.camera_point.z_m:.2f}"
    )
    cv2.putText(image, text, (20, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.imwrite(str(output), image)


def save_sent_image(path: str, frame) -> None:
    import cv2

    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), frame)


def save_depth_heatmap(
    path: str,
    depth_frame,
    depth_scale: float,
    detection: ServerBoxDetection,
    min_depth: float,
    max_depth: float,
) -> None:
    import cv2
    import numpy as np

    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    depth_m = np.asanyarray(depth_frame.get_data()).astype(np.float32) * float(depth_scale)
    valid = (depth_m >= float(min_depth)) & (depth_m <= float(max_depth))
    clipped = np.clip((depth_m - float(min_depth)) / max(1e-6, float(max_depth) - float(min_depth)), 0.0, 1.0)
    heat = cv2.applyColorMap((255 - (clipped * 255).astype(np.uint8)), cv2.COLORMAP_JET)
    heat[~valid] = (0, 0, 0)
    x1, y1, x2, y2 = [int(value) for value in detection.bbox]
    cv2.rectangle(heat, (x1, y1), (x2, y2), (255, 255, 255), 2)
    cv2.imwrite(str(output), heat)


def save_detection(path: str, detection: ServerBoxDetection) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    payload = {
        "created_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        "class_name": detection.class_name,
        "confidence": detection.confidence,
        "frame": detection.frame,
        "source": detection.source,
        "camera_point": {
            "x_m": detection.camera_point.x_m,
            "y_m": detection.camera_point.y_m,
            "z_m": detection.camera_point.z_m,
        },
        "bbox": list(detection.bbox),
        "center_px": list(detection.center_px),
    }
    output.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    logging.info("SERVER_BOX saved json path=%s", output)


def print_detection(detection: ServerBoxDetection) -> None:
    point = detection.camera_point
    print(
        "BOX_DETECTION",
        json.dumps(
            {
                "class_name": detection.class_name,
                "confidence": detection.confidence,
                "frame": detection.frame,
                "source": detection.source,
                "x_m": point.x_m,
                "y_m": point.y_m,
                "z_m": point.z_m,
                "bbox": list(detection.bbox),
            },
            ensure_ascii=False,
        ),
    )
    print(
        f"frame={detection.frame} X={point.x_m:+.3f}m Y={point.y_m:+.3f}m "
        f"Z={point.z_m:.3f}m conf={detection.confidence:.2f} class={detection.class_name}"
    )


def _join_url(server_url: str, path: str) -> str:
    return server_url.rstrip("/") + path


def _optional_int(value: object) -> Optional[int]:
    try:
        return int(value) if value is not None else None
    except (TypeError, ValueError):
        return None


def main() -> int:
    args = parse_args()
    configure_logging(args.log_file)
    try:
        return run(args)
    except (urllib.error.URLError, TimeoutError) as exc:
        logging.exception("SERVER_BOX network failed: %s", exc)
        return 3
    except Exception as exc:
        logging.exception("SERVER_BOX failed: %s", exc)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
