"""
학습한 모델로 test 폴더에서 N장 추론, bbox+polygon 시각화 저장.
실행:
    python3 predict_one.py            # 기본 10장
    python3 predict_one.py --n 5      # 5장
    python3 predict_one.py --n 0      # 전체
"""
from __future__ import annotations

import argparse
import random
from pathlib import Path

import cv2
from ultralytics import YOLO

from seg_obb_utils import draw_seg_with_obb

ROOT = Path(__file__).parent
IMG_DIR = ROOT / "product_classification-1" / "test" / "images"
OUT_DIR = ROOT / "predict_results"


def latest_best() -> Path:
    candidates = sorted(
        (ROOT / "runs" / "segment").glob("product_seg*/weights/best.pt"),
        key=lambda p: p.stat().st_mtime,
    )
    if not candidates:
        raise SystemExit(
            "best.pt 없음. 먼저 train_yolo8seg.py 로 학습부터 하세요."
        )
    return candidates[-1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=10,
                    help="추론할 이미지 수 (0이면 전체)")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    weights = latest_best()
    print(f"[info] using weights: {weights}")

    images = [p for p in IMG_DIR.iterdir()
              if p.suffix.lower() in {".jpg", ".jpeg", ".png"}]
    if not images:
        raise SystemExit(f"이미지 없음: {IMG_DIR}")

    random.seed(args.seed)
    random.shuffle(images)
    if args.n > 0:
        images = images[: args.n]
    print(f"[info] {len(images)}장 추론 시작")

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    model = YOLO(str(weights))
    names = model.names

    results = model(
        [str(p) for p in images],
        conf=args.conf,
        imgsz=args.imgsz,
    )

    for img_path, result in zip(images, results):
        base = result.orig_img if result.orig_img is not None else cv2.imread(str(img_path))
        annotated = draw_seg_with_obb(base, result, names)
        out_path = OUT_DIR / f"{img_path.stem}_pred.jpg"
        cv2.imwrite(str(out_path), annotated)
        print(f"[ok] {img_path.name} -> {out_path.name}")

    print(f"\n[done] 결과 위치: {OUT_DIR}")


if __name__ == "__main__":
    main()