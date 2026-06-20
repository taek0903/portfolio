"""Florence 2 빠른 테스트 - VQA + Bounding Box"""
import torch
import cv2
import numpy as np
from pathlib import Path
from PIL import Image
from transformers import Florence2ForConditionalGeneration, AutoProcessor

ROOT = Path(__file__).parent
OUT_BASE = ROOT / "test_images_predictions" / "florence2"

# ============== 모델 선택 ==============
# model_id = "florence-community/Florence-2-base-ft"
model_id = "florence-community/Florence-2-large-ft"
# =======================================

model = Florence2ForConditionalGeneration.from_pretrained(
    model_id,
    torch_dtype=torch.float16,
    device_map="auto"
)
processor = AutoProcessor.from_pretrained(model_id)

images = ["test_images_vla/img01.jpg", "test_images_vla/img02.jpg", "test_images_vla/img03.jpg"]
question = "Is there any object inside the red boundary area? Answer Yes or No and explain."

# 모델 이름으로 출력 폴더 생성
model_name = model_id.split("/")[-1]
out_dir = OUT_BASE / model_name
out_dir.mkdir(parents=True, exist_ok=True)


def run_inference(image, task_prompt, text_input=None):
    """Florence 2 추론 실행."""
    prompt = task_prompt + text_input if text_input else task_prompt
    inputs = processor(text=prompt, images=image, return_tensors="pt").to(model.device)
    generated_ids = model.generate(**inputs, max_new_tokens=1024, do_sample=False, num_beams=3)
    generated_text = processor.batch_decode(generated_ids, skip_special_tokens=False)[0]
    parsed = processor.post_process_generation(generated_text, task=task_prompt, image_size=image.size)
    return parsed


def draw_boxes(image, boxes, labels):
    """이미지에 bounding box 그리기."""
    img_cv = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
    colors = [(0, 255, 0), (255, 0, 0), (0, 0, 255), (255, 255, 0), (255, 0, 255)]
    
    for i, (box, label) in enumerate(zip(boxes, labels)):
        color = colors[i % len(colors)]
        x1, y1, x2, y2 = map(int, box)
        cv2.rectangle(img_cv, (x1, y1), (x2, y2), color, 3)
        cv2.putText(img_cv, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    
    return img_cv


for img_path in images:
    if not Path(img_path).exists():
        print(f"{img_path}: 파일 없음, 스킵")
        continue
        
    image = Image.open(img_path).convert("RGB")
    
    # 1. VQA로 yes/no 확인
    vqa_result = run_inference(image, "<VQA>", question)
    answer = vqa_result.get("<VQA>", "")
    has_object = "yes" in answer.lower()
    
    print(f"\n{img_path}:")
    print(f"  VQA: {answer}")
    print(f"  Has object: {has_object}")
    
    # 2. yes이면 Object Detection으로 위치 찾기
    if has_object:
        od_result = run_inference(image, "<OD>")
        od_data = od_result.get("<OD>", {})
        boxes = od_data.get("bboxes", [])
        labels = od_data.get("labels", [])
        
        print(f"  Detected: {len(boxes)} objects")
        for box, label in zip(boxes, labels):
            print(f"    - {label}: {box}")
        
        # 3. Bounding box 그려서 저장
        if boxes:
            img_cv = draw_boxes(image, boxes, labels)
            out_path = out_dir / f"{Path(img_path).stem}_detected.jpg"
            cv2.imwrite(str(out_path), img_cv)
            print(f"  Saved: {out_path}")
    else:
        # no인 경우 원본 저장
        out_path = out_dir / f"{Path(img_path).stem}_no_object.jpg"
        image.save(str(out_path))
        print(f"  Saved (no object): {out_path}")

print(f"\n[done] 결과 폴더: {out_dir}")


# ============== 이전 코드 (VQA만) ==============
# """Florence 2 빠른 테스트 - VQA"""
# import torch
# from PIL import Image
# from transformers import Florence2ForConditionalGeneration, AutoProcessor
# 
# model_id = "florence-community/Florence-2-large-ft"
# model = Florence2ForConditionalGeneration.from_pretrained(
#     model_id,
#     torch_dtype=torch.float16,
#     device_map="auto"
# )
# processor = AutoProcessor.from_pretrained(model_id)
# 
# images = ["florence2_img.jpg", "florence2_img_2.jpg", "florence2_img_3.jpg"]
# question = "Is there any object inside the red boundary area? Answer Yes or No and explain."
# 
# for img_path in images:
#     image = Image.open(img_path).convert("RGB")
#     prompt = f"<VQA>{question}"
#     
#     inputs = processor(text=prompt, images=image, return_tensors="pt").to(model.device)
#     generated_ids = model.generate(**inputs, max_new_tokens=100, do_sample=False, num_beams=3)
#     answer = processor.batch_decode(generated_ids, skip_special_tokens=True)[0]
#     
#     print(f"{img_path}: {answer}")
# ===============================================
