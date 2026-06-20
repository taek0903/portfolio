"""
test_images/ 폴더의 이미지들을 여러 모델로 추론하고 결과를 별도 폴더에 저장.
--xai 플래그를 추가하면 각 모델별 XAI 히트맵(열화상 스타일)도 함께 생성.

실행:
    cd ~/cobot_ws2-main/src/test/models
    python3 predict_test_images.py
    python3 predict_test_images.py --conf 0.4 --imgsz 1024
    python3 predict_test_images.py --xai                  # 추론 + XAI 히트맵
    python3 predict_test_images.py --xai-only             # XAI 히트맵만 생성
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
import torch
from ultralytics import YOLO

ROOT = Path(__file__).parent
IMG_DIR = ROOT / "test_images"

MODELS = {
    "seg": ROOT / "runs" / "segment" / "product_seg-2" / "weights" / "best.pt",
    "det": ROOT / "runs" / "segment" / "product_seg3" / "weights" / "best.pt",
    "11s-seg": ROOT / "runs" / "segment" / "product_seg4" / "weights" / "best.pt",
    "11m-seg": ROOT / "runs" / "segment" / "product_seg5" / "weights" / "best.pt",
    "11m-seg(total)": ROOT / "runs" / "segment" / "product_seg2" / "weights" / "best.pt",
    "11m-seg(v10)": ROOT / "runs" / "segment" / "product_seg8" / "weights" / "best.pt",
    "11m-seg(v10)_2": ROOT / "runs" / "segment" / "product_seg15" / "weights" / "best.pt",
    # "8s(v10)": ROOT / "runs" / "segment" / "product_seg-4" / "weights" / "best.pt",
}

OUT_BASE = ROOT / "test_images_predictions"


def collect_images(img_dir: Path) -> list[Path]:
    exts = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
    return sorted(p for p in img_dir.iterdir() if p.suffix.lower() in exts)


# ---------------------------------------------------------------------------
# Prediction
# ---------------------------------------------------------------------------
def run_model(tag: str, weights: Path, src_dir: Path,
              conf: float, imgsz: int) -> Path:
    if not weights.exists():
        raise SystemExit(f"weights 없음: {weights}")

    out_dir = OUT_BASE / tag
    print(f"\n[{tag}] weights: {weights}")
    print(f"[{tag}] 결과 폴더: {out_dir}")

    model = YOLO(str(weights))
    model.predict(
        source=str(src_dir),
        conf=conf,
        imgsz=imgsz,
        save=True,
        save_txt=True,
        save_conf=True,
        project=str(OUT_BASE),
        name=tag,
        exist_ok=True,
        verbose=False,
    )
    return out_dir


# ---------------------------------------------------------------------------
# XAI heatmap (EigenCAM via pytorch-grad-cam)
# ---------------------------------------------------------------------------
def _find_target_layers(model: YOLO) -> list:
    """Neck(FPN)의 C2f/C3k2 레이어들을 수집 (멀티스케일 융합용)."""
    seq = model.model.model
    neck_types = ("C2f", "C3k2", "C2fCIB", "RepC3")
    candidates = []
    for layer in seq:
        if type(layer).__name__ in neck_types:
            candidates.append(layer)
    if len(candidates) >= 3:
        return candidates[-3:]
    if candidates:
        return candidates
    for layer in seq:
        if type(layer).__name__ in ("SPPF", "SPP", "C2PSA"):
            return [layer]
    return [seq[min(9, len(seq) - 1)]]


def _letterbox(img_bgr: np.ndarray, imgsz: int):
    """YOLO 스타일 letterbox 리사이즈 (정사각 패딩)."""
    h, w = img_bgr.shape[:2]
    scale = imgsz / max(h, w)
    nh, nw = int(h * scale), int(w * scale)
    resized = cv2.resize(img_bgr, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((imgsz, imgsz, 3), 114, dtype=np.uint8)
    dy, dx = (imgsz - nh) // 2, (imgsz - nw) // 2
    canvas[dy : dy + nh, dx : dx + nw] = resized
    return canvas


def _layer_heatmap(act: torch.Tensor) -> np.ndarray:
    """단일 레이어의 활성화 맵 -> [0,1] 히트맵."""
    act = act.squeeze(0).float()              # [C, H, W]
    weights = act.flatten(1).norm(dim=1)      # [C]
    weights = weights / (weights.sum() + 1e-8)
    hm = torch.relu((weights[:, None, None] * act).sum(dim=0))
    return hm.cpu().numpy().astype(np.float64)


def _extract_heatmap(
    torch_model: torch.nn.Module,
    target_layers: list[torch.nn.Module],
    input_tensor: torch.Tensor,
    out_hw: int,
) -> np.ndarray:
    """멀티 레이어 활성화를 융합하여 고품질 [0,1] 히트맵 반환."""
    captured: dict[int, torch.Tensor] = {}
    hooks = []
    for idx, layer in enumerate(target_layers):
        def _hook(idx_=idx):
            return lambda _m, _i, o: captured.update({idx_: o.detach()})
        hooks.append(layer.register_forward_hook(_hook()))

    with torch.no_grad():
        torch_model(input_tensor)
    for h in hooks:
        h.remove()

    layer_maps = []
    for idx in sorted(captured):
        hm = _layer_heatmap(captured[idx])
        hm = cv2.resize(hm, (out_hw, out_hw), interpolation=cv2.INTER_LINEAR)
        layer_maps.append(hm)

    combined = np.mean(layer_maps, axis=0)

    # 퍼센타일 정규화: 배경 노이즈 강하게 억제
    p_lo, p_hi = np.percentile(combined, [30, 99])
    combined = np.clip(combined, p_lo, p_hi)
    if p_hi > p_lo:
        combined = (combined - p_lo) / (p_hi - p_lo)

    # 감마 보정: 약한 활성화 억제, 강한 피크(객체)만 부각
    combined = np.power(combined, 2.0)

    # 가우시안 블러: 자연스러운 열화상 느낌
    ksize = max(3, (out_hw // 30) | 1)
    combined = cv2.GaussianBlur(combined, (ksize, ksize), sigmaX=0)

    return combined


def _bbox_mask(boxes: np.ndarray, h: int, w: int, margin: int = 10) -> np.ndarray:
    """바운딩 박스 영역을 1로, 나머지를 0으로 채운 마스크 반환."""
    mask = np.zeros((h, w), dtype=np.float64)
    for x1, y1, x2, y2 in boxes.astype(int):
        x1 = max(0, x1 - margin)
        y1 = max(0, y1 - margin)
        x2 = min(w, x2 + margin)
        y2 = min(h, y2 + margin)
        mask[y1:y2, x1:x2] = 1.0
    ksize = max(3, margin * 2 + 1) | 1
    mask = cv2.GaussianBlur(mask, (ksize, ksize), sigmaX=0)
    return mask


def _overlay_heatmap(
    img_bgr: np.ndarray,
    heatmap: np.ndarray,
    bbox_mask: np.ndarray | None = None,
    alpha: float = 0.5,
    colormap: int = cv2.COLORMAP_JET,
) -> tuple[np.ndarray, np.ndarray]:
    """바운딩 박스 내부만 히트맵 오버레이, 외부는 원본 유지."""
    h, w = img_bgr.shape[:2]
    hm_resized = cv2.resize(heatmap, (w, h), interpolation=cv2.INTER_LINEAR)

    if bbox_mask is not None:
        hm_resized = hm_resized * bbox_mask

    hm_uint8 = (hm_resized * 255).astype(np.uint8)
    hm_color = cv2.applyColorMap(hm_uint8, colormap)

    if bbox_mask is not None:
        blend = bbox_mask[:, :, None].astype(np.float32)
        overlay = (
            img_bgr.astype(np.float32) * (1 - blend * alpha)
            + hm_color.astype(np.float32) * (blend * alpha)
        ).astype(np.uint8)
    else:
        overlay = cv2.addWeighted(img_bgr, 1 - alpha, hm_color, alpha, 0)

    return overlay, hm_color


def run_xai(
    tag: str,
    weights: Path,
    images: list[Path],
    imgsz: int,
    conf: float = 0.25,
) -> Path:
    """한 모델에 대해 모든 이미지의 XAI 히트맵을 생성·저장 (bbox 내부만)."""
    if not weights.exists():
        raise SystemExit(f"weights 없음: {weights}")

    xai_dir = OUT_BASE / f"{tag}_xai"
    xai_dir.mkdir(parents=True, exist_ok=True)

    print(f"\n[{tag}] XAI 히트맵 생성 중...")
    print(f"[{tag}] XAI 결과 폴더: {xai_dir}")

    model = YOLO(str(weights))
    model.model.eval()
    target_layers = _find_target_layers(model)
    layer_names = [type(l).__name__ for l in target_layers]
    print(f"[{tag}] target layers ({len(target_layers)}개): {layer_names}")

    for img_path in images:
        img_bgr = cv2.imread(str(img_path))
        if img_bgr is None:
            print(f"  [skip] 읽기 실패: {img_path.name}")
            continue

        h_orig, w_orig = img_bgr.shape[:2]

        # 1) 추론 -> 바운딩 박스 추출 (predict가 모델을 GPU로 이동시킴)
        results = model.predict(
            source=str(img_path), conf=conf, imgsz=imgsz, verbose=False,
        )
        boxes = results[0].boxes.xyxy.cpu().numpy() if len(results[0].boxes) else np.empty((0, 4))

        if len(boxes) == 0:
            print(f"  [skip] 탐지 없음: {img_path.name}")
            continue

        # 2) 활성화 히트맵 추출 (predict 이후 device 확인)
        device = next(model.model.parameters()).device
        canvas = _letterbox(img_bgr, imgsz)
        tensor = (
            torch.from_numpy(
                cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
            )
            .permute(2, 0, 1)
            .unsqueeze(0)
            .to(device)
        )
        heatmap = _extract_heatmap(model.model, target_layers, tensor, imgsz)

        # 3) bbox 마스크 생성 & 오버레이
        mask = _bbox_mask(boxes, h_orig, w_orig, margin=15)
        overlay, hm_color = _overlay_heatmap(img_bgr, heatmap, bbox_mask=mask)

        stem = img_path.stem
        cv2.imwrite(str(xai_dir / f"{stem}_overlay.jpg"), overlay)
        cv2.imwrite(str(xai_dir / f"{stem}_heatmap.jpg"), hm_color)

    print(f"[{tag}] XAI done -> {xai_dir}")
    return xai_dir


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--xai", action="store_true",
                    help="추론과 함께 XAI 히트맵(열화상 스타일)도 생성")
    ap.add_argument("--xai-only", action="store_true",
                    help="추론 없이 XAI 히트맵만 생성")
    ap.add_argument("--model", type=str, nargs="+", default=None,
                    help="특정 모델만 실행 (예: --model 11m-seg(v10)_2)")
    args = ap.parse_args()

    if not IMG_DIR.exists():
        raise SystemExit(f"이미지 폴더 없음: {IMG_DIR}")

    images = collect_images(IMG_DIR)
    if not images:
        raise SystemExit(f"이미지 없음: {IMG_DIR}")

    # 모델 필터링
    if args.model:
        selected = {}
        for m in args.model:
            if m not in MODELS:
                raise SystemExit(
                    f"모델 '{m}' 없음. 사용 가능: {', '.join(MODELS.keys())}"
                )
            selected[m] = MODELS[m]
        targets = selected
    else:
        targets = MODELS

    print(f"[info] {len(images)}장 대상, 모델 {len(targets)}개: {', '.join(targets)}")

    OUT_BASE.mkdir(parents=True, exist_ok=True)

    do_predict = not args.xai_only
    do_xai = args.xai or args.xai_only

    # --- 추론 ---
    if do_predict:
        for tag, weights in targets.items():
            out_dir = run_model(tag, weights, IMG_DIR,
                                conf=args.conf, imgsz=args.imgsz)
            print(f"[{tag}] done -> {out_dir}")

    # --- XAI 히트맵 ---
    if do_xai:
        print("\n" + "=" * 60)
        print(" XAI 히트맵 생성 (EigenCAM 방식)")
        print("=" * 60)
        for tag, weights in targets.items():
            run_xai(tag, weights, images, imgsz=args.imgsz, conf=args.conf)

    # --- 결과 요약 ---
    print(f"\n[done] 전체 결과 루트: {OUT_BASE}")
    for tag in targets:
        if do_predict:
            print(f"  - {tag:<20s} 추론: {OUT_BASE / tag}")
        if do_xai:
            print(f"  - {tag:<20s} XAI:  {OUT_BASE / tag}_xai")


if __name__ == "__main__":
    main()
