"""
YOLO-seg 데이터셋(product_classification-1)으로 yolov8s-seg 모델을 학습.

사용:
    cd ~/cobot_ws/src/cobot2
    python3 train_yolo_seg.py

결과:
    runs/segment/product_seg/weights/{best.pt,last.pt}
"""

from __future__ import annotations

from pathlib import Path

from ultralytics import YOLO

ROOT = Path(__file__).parent

MODEL = "yolo11s.pt"
DATA = ROOT / "datasets" / "product_classification-10" / "data.yaml"
EPOCHS = 100
IMGSZ = 640
BATCH = 16
PATIENCE = 20
DEVICE = "0"
PROJECT = ROOT / "runs" / "segment"
NAME = "product_seg"


def main() -> None:
    if not DATA.exists():
        raise SystemExit(f"data.yaml 없음: {DATA}")

    model = YOLO(MODEL)
    results = model.train(
        data=str(DATA),
        epochs=EPOCHS,
        imgsz=IMGSZ,
        batch=BATCH,    
        patience=PATIENCE,
        device=DEVICE,
        project=str(PROJECT),
        name=NAME,
        exist_ok=False,
    )

    save_dir = Path(results.save_dir) if hasattr(results, "save_dir") else None
    if save_dir is not None:
        print(f"\n[done] 학습 산출물: {save_dir}")
        print(f"       best weights: {save_dir / 'weights' / 'best.pt'}")
        print("\n다음 단계 (추론):")
        print(f"  yolo segment predict model={save_dir / 'weights' / 'best.pt'} \\")
        print(f"      source=product_classification-1/test/images \\")
        print(f"      save=True save_txt=True imgsz={IMGSZ} conf=0.25")


if __name__ == "__main__":
    main()
