"""
Florence 2 모델을 사용한 zero-shot 객체 탐지 테스트.

지원 Task:
    - OD: 일반 객체 탐지 (모든 객체)
    - OPEN_VOC: Open Vocabulary Detection (특정 객체 텍스트로 찾기)
    - CAPTION: 이미지 캡션 생성
    - GROUNDING: 캡션 기반 위치 찾기

실행 예시:
    cd ~/cobot_ws2/src/test/models
    python3 predict_florence2.py --task OD                    # 일반 객체 탐지
    python3 predict_florence2.py --task OPEN_VOC --text "gum" # 특정 객체 찾기
    python3 predict_florence2.py --task CAPTION               # 캡션 생성
    python3 predict_florence2.py --n 5                        # 5장만 테스트
    python3 predict_florence2.py --model base                 # 작은 모델 사용
"""
from __future__ import annotations

import argparse
import random
from pathlib import Path

import cv2
import numpy as np
import torch
from PIL import Image
from transformers import AutoProcessor, Florence2ForConditionalGeneration

ROOT = Path(__file__).parent
IMG_DIR = ROOT / "test_images"
OUT_BASE = ROOT / "test_images_predictions" / "florence2"

TASK_PROMPTS = {
    "OD": "<OD>",
    "OPEN_VOC": "<OPEN_VOCABULARY_DETECTION>",
    "CAPTION": "<CAPTION>",
    "DETAILED_CAPTION": "<MORE_DETAILED_CAPTION>",
    "GROUNDING": "<CAPTION_TO_PHRASE_GROUNDING>",
}

MODEL_IDS = {
    "large": "florence-community/Florence-2-large",
    "base": "florence-community/Florence-2-base",
}

COLORS = [
    (255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0),
    (255, 0, 255), (0, 255, 255), (128, 0, 0), (0, 128, 0),
    (0, 0, 128), (128, 128, 0), (128, 0, 128), (0, 128, 128),
]


def load_model(model_size: str = "large"):
    """Florence 2 모델과 프로세서 로드."""
    model_id = MODEL_IDS.get(model_size, MODEL_IDS["large"])
    print(f"[info] 모델 로딩: {model_id}")
    
    dtype = torch.float16 if torch.cuda.is_available() else torch.float32
    device_map = "auto" if torch.cuda.is_available() else "cpu"
    
    model = Florence2ForConditionalGeneration.from_pretrained(
        model_id,
        torch_dtype=dtype,
        device_map=device_map,
    )
    processor = AutoProcessor.from_pretrained(model_id)
    
    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"[info] 디바이스: {device}, dtype: {dtype}")
    return model, processor


def run_inference(model, processor, image: Image.Image, task: str, text_input: str = None):
    """Florence 2 추론 실행."""
    task_prompt = TASK_PROMPTS.get(task, task)
    
    if text_input and task in ("OPEN_VOC", "GROUNDING"):
        prompt = task_prompt + text_input
    else:
        prompt = task_prompt
    
    device = next(model.parameters()).device
    dtype = next(model.parameters()).dtype
    
    inputs = processor(text=prompt, images=image, return_tensors="pt")
    inputs = {k: v.to(device, dtype) if v.dtype == torch.float32 else v.to(device) 
              for k, v in inputs.items()}
    
    generated_ids = model.generate(
        **inputs,
        max_new_tokens=1024,
        num_beams=3,
        early_stopping=False,
        do_sample=False,
    )
    
    generated_text = processor.batch_decode(generated_ids, skip_special_tokens=False)[0]
    parsed = processor.post_process_generation(
        generated_text,
        task=task_prompt,
        image_size=(image.width, image.height)
    )
    
    return parsed


def draw_results(image: Image.Image, result: dict, task: str) -> tuple:
    """결과를 이미지에 그리기."""
    img_cv = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
    labels_info = []
    
    task_prompt = TASK_PROMPTS.get(task, task)
    
    if task_prompt == "<OD>" and task_prompt in result:
        data = result[task_prompt]
        bboxes = data.get("bboxes", [])
        labels = data.get("labels", [])
        
        for i, (bbox, label) in enumerate(zip(bboxes, labels)):
            color = COLORS[i % len(COLORS)]
            x1, y1, x2, y2 = map(int, bbox)
            cv2.rectangle(img_cv, (x1, y1), (x2, y2), color, 2)
            cv2.putText(img_cv, label, (x1, y1 - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            labels_info.append(f"{label}: [{x1},{y1},{x2},{y2}]")
    
    elif task_prompt == "<OPEN_VOCABULARY_DETECTION>" and task_prompt in result:
        data = result[task_prompt]
        bboxes = data.get("bboxes", [])
        labels = data.get("bboxes_labels", data.get("labels", []))
        
        for i, (bbox, label) in enumerate(zip(bboxes, labels)):
            color = COLORS[i % len(COLORS)]
            x1, y1, x2, y2 = map(int, bbox)
            cv2.rectangle(img_cv, (x1, y1), (x2, y2), color, 3)
            cv2.putText(img_cv, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            labels_info.append(f"{label}: [{x1},{y1},{x2},{y2}]")
    
    elif task_prompt == "<CAPTION_TO_PHRASE_GROUNDING>" and task_prompt in result:
        data = result[task_prompt]
        bboxes = data.get("bboxes", [])
        labels = data.get("labels", [])
        
        for i, (bbox, label) in enumerate(zip(bboxes, labels)):
            color = COLORS[i % len(COLORS)]
            x1, y1, x2, y2 = map(int, bbox)
            cv2.rectangle(img_cv, (x1, y1), (x2, y2), color, 2)
            cv2.putText(img_cv, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            labels_info.append(f"{label}: [{x1},{y1},{x2},{y2}]")
    
    elif task_prompt in ("<CAPTION>", "<MORE_DETAILED_CAPTION>"):
        caption = result.get(task_prompt, "")
        labels_info.append(f"Caption: {caption}")
        h, w = img_cv.shape[:2]
        cv2.putText(img_cv, caption[:80], (10, h - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    return img_cv, labels_info


def collect_images(img_dir: Path) -> list[Path]:
    """이미지 파일 수집."""
    exts = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
    return sorted(p for p in img_dir.iterdir() if p.suffix.lower() in exts)


def main():
    ap = argparse.ArgumentParser(description="Florence 2 Zero-Shot 객체 탐지")
    ap.add_argument("--task", type=str, default="OD",
                    choices=["OD", "OPEN_VOC", "CAPTION", "DETAILED_CAPTION", "GROUNDING"],
                    help="실행할 task (기본: OD)")
    ap.add_argument("--text", type=str, default=None,
                    help="OPEN_VOC/GROUNDING task에서 찾을 객체 텍스트")
    ap.add_argument("--model", type=str, default="large",
                    choices=["large", "base"],
                    help="모델 크기 (기본: large)")
    ap.add_argument("--n", type=int, default=0,
                    help="테스트할 이미지 수 (0이면 전체)")
    ap.add_argument("--seed", type=int, default=42)
    args = ap.parse_args()
    
    if args.task in ("OPEN_VOC", "GROUNDING") and not args.text:
        print("[warn] --text 옵션이 필요합니다. 기본값 'product' 사용")
        args.text = "product"
    
    if not IMG_DIR.exists():
        raise SystemExit(f"이미지 폴더 없음: {IMG_DIR}")
    
    images = collect_images(IMG_DIR)
    if not images:
        raise SystemExit(f"이미지 없음: {IMG_DIR}")
    
    random.seed(args.seed)
    random.shuffle(images)
    if args.n > 0:
        images = images[:args.n]
    
    print(f"[info] {len(images)}장 처리 예정")
    print(f"[info] Task: {args.task}")
    if args.text:
        print(f"[info] 검색 텍스트: {args.text}")
    
    model, processor = load_model(args.model)
    
    out_dir = OUT_BASE / args.task.lower()
    out_dir.mkdir(parents=True, exist_ok=True)
    labels_dir = out_dir / "labels"
    labels_dir.mkdir(exist_ok=True)
    
    print(f"[info] 결과 폴더: {out_dir}")
    
    for i, img_path in enumerate(images, 1):
        print(f"[{i}/{len(images)}] {img_path.name}", end=" ... ")
        
        try:
            image = Image.open(img_path).convert("RGB")
            result = run_inference(model, processor, image, args.task, args.text)
            
            img_cv, labels_info = draw_results(image, result, args.task)
            
            out_img = out_dir / f"{img_path.stem}.jpg"
            cv2.imwrite(str(out_img), img_cv)
            
            out_txt = labels_dir / f"{img_path.stem}.txt"
            with open(out_txt, "w", encoding="utf-8") as f:
                f.write(f"Task: {args.task}\n")
                if args.text:
                    f.write(f"Query: {args.text}\n")
                f.write(f"Raw: {result}\n\n")
                for info in labels_info:
                    f.write(info + "\n")
            
            print(f"OK ({len(labels_info)} items)")
            
        except Exception as e:
            print(f"ERROR: {e}")
    
    print(f"\n[done] 결과 위치: {out_dir}")


if __name__ == "__main__":
    main()
