"""
YOLO-World Zero-Shot 테스트.

텍스트로 클래스를 지정해서 open-vocabulary detection.

사용:
    cd ~/cobot_ws2/src/test/models
    python3 test_yolo_world.py
"""
from __future__ import annotations

from pathlib import Path

import cv2
from ultralytics import YOLO

ROOT = Path(__file__).parent
IMG_DIR = ROOT / "test_images"
OUT_DIR = ROOT / "test_images_predictions" / "yolo_world"

# 데이터셋 클래스들 (영어로 변환)
CLASSES = [
    "coke can",
    "pepsi can",
    "soda can",
    "cracker",
    "chewing gum",
    "milk",
    "snack",
    "beverage",
]

# 또는 한국어 제품명 그대로 (테스트용)
# CLASSES = [
#     "can_coke_decaf", "can_coke_zero", "can_pepsi", "can_pepsi_zero",
#     "cracker_ace", "cracker_vege", "gum_green", "gum_yellow",
#     "milk_blue", "milk_green",
# ]


def collect_images(img_dir: Path, n: int = 0) -> list[Path]:
    """이미지 파일 수집."""
    exts = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
    images = sorted(p for p in img_dir.iterdir() if p.suffix.lower() in exts)
    if n > 0:
        images = images[:n]
    return images


def main():
    print("[info] YOLO-World 모델 로딩...")
    model = YOLO("yolov8s-worldv2.pt")  # 자동 다운로드
    
    # 클래스 설정
    model.set_classes(CLASSES)
    print(f"[info] 클래스: {CLASSES}")
    
    if not IMG_DIR.exists():
        raise SystemExit(f"이미지 폴더 없음: {IMG_DIR}")
    
    images = collect_images(IMG_DIR, n=0)  # 0이면 전체
    print(f"[info] {len(images)}장 테스트")
    
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    labels_dir = OUT_DIR / "labels"
    labels_dir.mkdir(exist_ok=True)
    
    for i, img_path in enumerate(images, 1):
        print(f"[{i}/{len(images)}] {img_path.name}", end=" ... ")
        
        results = model.predict(
            source=str(img_path),
            conf=0.1,  # 낮은 confidence로 테스트
            verbose=False,
        )
        
        result = results[0]
        
        # 결과 이미지 저장
        annotated = result.plot()
        out_img = OUT_DIR / f"{img_path.stem}.jpg"
        cv2.imwrite(str(out_img), annotated)
        
        # 라벨 저장
        boxes = result.boxes
        out_txt = labels_dir / f"{img_path.stem}.txt"
        with open(out_txt, "w", encoding="utf-8") as f:
            f.write(f"Classes: {CLASSES}\n\n")
            if len(boxes) > 0:
                for box in boxes:
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    xyxy = box.xyxy[0].tolist()
                    cls_name = CLASSES[cls_id] if cls_id < len(CLASSES) else f"class_{cls_id}"
                    f.write(f"{cls_name}: {conf:.2f} [{int(xyxy[0])},{int(xyxy[1])},{int(xyxy[2])},{int(xyxy[3])}]\n")
        
        n_detections = len(boxes)
        print(f"OK ({n_detections} detections)")
    
    print(f"\n[done] 결과 위치: {OUT_DIR}")
    print(f"       이미지 뷰어로 확인: eog {OUT_DIR}")


if __name__ == "__main__":
    main()
