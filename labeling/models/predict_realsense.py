"""
RealSense 카메라 라이브 피드에서 yolov8-seg best.pt로 실시간 추론.

실행:
    cd ~/cobot_ws/src/cobot2
    python3 predict_realsense.py
    python3 predict_realsense.py --conf 0.5
    python3 predict_realsense.py --weights runs/segment/product_seg/weights/best.pt
    python3 predict_realsense.py --device cpu       # GPU 없을 때

키:
    q : 종료
    s : 현재 프레임 스냅샷 저장 (./realsense_snaps/)

주의:
    - 다른 프로세스가 RealSense를 점유하고 있으면 시작이 실패합니다.
      예) ros2 launch realsense2_camera ... 가 떠있으면 먼저 끄세요.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

from seg_obb_utils import draw_seg_with_obb

ROOT = Path(__file__).parent
SNAP_DIR = ROOT / "realsense_snaps"


def latest_best() -> Path:
    candidates = sorted(
        (ROOT / "runs" / "segment").glob("**/weights/best.pt"),
        key=lambda p: p.stat().st_mtime,
    )
    if not candidates:
        raise SystemExit(
            "best.pt 없음. 먼저 train_yolo8seg.py 로 학습부터 하세요."
        )
    return candidates[-1]


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default=None,
                    help="best.pt 경로 (없으면 runs/segment/**/best.pt 중 가장 최신)")
    ap.add_argument("--conf", type=float, default=0.4)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--device", default="0", help="GPU id 또는 'cpu'")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--show-depth", action="store_true",
                    help="컬러+depth 컬러맵을 옆에 같이 표시")
    args = ap.parse_args()

    weights = Path(args.weights) if args.weights else latest_best()
    print(f"[info] using weights: {weights}")
    model = YOLO(str(weights))
    names = model.names

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    if args.show_depth:
        cfg.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    profile = pipeline.start(cfg)
    dev_name = profile.get_device().get_info(rs.camera_info.name)
    print(f"[info] RealSense started: {dev_name} @ {args.width}x{args.height}/{args.fps}fps")

    align = rs.align(rs.stream.color) if args.show_depth else None
    SNAP_DIR.mkdir(exist_ok=True, parents=True)

    fps_t0 = time.time()
    fps_n = 0
    fps_disp = 0.0
    win = "YOLOv8-seg | RealSense (q=quit, s=snap)"
    try:
        while True:
            frames = pipeline.wait_for_frames()
            if align is not None:
                frames = align.process(frames)
            color = frames.get_color_frame()
            if not color:
                continue
            img = np.asanyarray(color.get_data())  # BGR

            result = model(img, conf=args.conf, imgsz=args.imgsz,
                           device=args.device, verbose=False)[0]
            annotated = draw_seg_with_obb(img, result, names)

            fps_n += 1
            if fps_n >= 10:
                t = time.time()
                fps_disp = fps_n / (t - fps_t0)
                fps_t0 = t
                fps_n = 0
            cv2.putText(annotated, f"FPS: {fps_disp:.1f}", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)

            display = annotated
            if args.show_depth:
                depth = frames.get_depth_frame()
                if depth:
                    d = np.asanyarray(depth.get_data())
                    d_vis = cv2.applyColorMap(
                        cv2.convertScaleAbs(d, alpha=0.03), cv2.COLORMAP_JET
                    )
                    display = np.hstack([annotated, d_vis])

            cv2.imshow(win, display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if key == ord('s'):
                ts = time.strftime("%Y%m%d_%H%M%S")
                out = SNAP_DIR / f"snap_{ts}.jpg"
                cv2.imwrite(str(out), display)
                print(f"[snap] saved {out}")
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
