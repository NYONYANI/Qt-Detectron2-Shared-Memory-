import os
import sys
import cv2
import numpy as np
import json
from multiprocessing import shared_memory
import time
import struct
import logging
import posix_ipc
import traceback 
import threading
import queue

from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2 import model_zoo
from ultralytics import YOLO

# 로깅 설정
logging.basicConfig(
    level=logging.WARNING, 
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler('/tmp/cup_detector.log'),
        logging.StreamHandler(sys.stdout)
    ]
)

# 공유 메모리 이름 및 크기 정의
SHM_IMAGE_NAME = "realsense_image"
SHM_RESULT_NAME = "detection_result"
SHM_CONTROL_NAME = "detection_control"
SEM_IMAGE_NAME = "/sem_realsense_image"
SEM_RESULT_NAME = "/sem_detection_result"
SEM_CONTROL_NAME = "/sem_detection_control"

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
IMAGE_CHANNELS = 3
IMAGE_SIZE = IMAGE_WIDTH * IMAGE_HEIGHT * IMAGE_CHANNELS
RESULT_SIZE = 100 * 1024
CONTROL_SIZE = 16

OFFSET_NEW_FRAME = 0
OFFSET_RESULT_READY = 1
OFFSET_SHUTDOWN = 2

# 설정
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUTPUT_DIR = os.path.join(BASE_DIR, "output")
VIS_OUTPUT_DIR = os.path.join(BASE_DIR, "visualization")
CLASS_NAMES = ["body-handle", "body", "handle"]
CLASS_ID_BODY = CLASS_NAMES.index("body")
CLASS_ID_HANDLE = CLASS_NAMES.index("handle")

os.makedirs(VIS_OUTPUT_DIR, exist_ok=True)

# 시각화 색상 정의 (BGR)
COLOR_MAP = {
    0: (0, 0, 255),      # body-handle: Red
    1: (0, 255, 0),      # body: Green
    2: (255, 0, 0)       # handle: Blue
}

# 💡 시각화 작업 큐 (스레드 간 통신)
visualization_queue = queue.Queue(maxsize=10)
visualization_thread = None
visualization_running = False

def setup_cfg_for_inference():
    """추론용 config 설정"""
    cfg = get_cfg()
    cfg.merge_from_file(
        model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")
    )
    cfg.MODEL.ROI_HEADS.NUM_CLASSES = len(CLASS_NAMES)
    
    model_path = os.path.join(OUTPUT_DIR, "dt_v1.pth")
    if not os.path.exists(model_path):
        logging.error(f"오류: 모델 파일을 찾을 수 없습니다: {model_path}")
        raise FileNotFoundError(f"모델 파일 없음: {model_path}")
    
    cfg.MODEL.WEIGHTS = model_path
    cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5
    cfg.MODEL.DEVICE = "cuda"
    cfg.INPUT.MIN_SIZE_TEST = 640
    cfg.INPUT.MAX_SIZE_TEST = 1333
    
    return cfg

def non_max_suppression(boxes, scores, threshold=0.5):
    """간단한 NMS 구현"""
    if len(boxes) == 0:
        return []
    boxes = np.array(boxes)
    scores = np.array(scores)
    indices = np.argsort(scores)[::-1]
    keep = []
    while len(indices) > 0:
        i = indices[0]
        keep.append(i)
        x1 = np.maximum(boxes[i, 0], boxes[indices[1:], 0])
        y1 = np.maximum(boxes[i, 1], boxes[indices[1:], 1])
        x2 = np.minimum(boxes[i, 2], boxes[indices[1:], 2])
        y2 = np.minimum(boxes[i, 3], boxes[indices[1:], 3])
        w = np.maximum(0, x2 - x1)
        h = np.maximum(0, y2 - y1)
        inter = w * h
        union = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1]) + \
                (boxes[indices[1:], 2] - boxes[indices[1:], 0]) * \
                (boxes[indices[1:], 3] - boxes[indices[1:], 1]) - inter
        iou = inter / union
        indices = indices[1:][iou <= threshold]
    return keep

def rle_encode(mask):
    """
    마스크를 RLE로 인코딩
    C++ 디코더는 첫 런을 배경(0), 두 번째 런을 마스크(255)로 가정
    따라서 첫 픽셀이 255면 길이 0인 배경 런을 앞에 추가
    
    Returns:
        list of int: [배경길이1, 마스크길이1, 배경길이2, 마스크길이2, ...]
    """
    mask_flat = mask.flatten()
    rle = []
    
    if len(mask_flat) == 0:
        return rle
    
    # 첫 픽셀 값 저장
    first_pixel_value = mask_flat[0]
    current_value = first_pixel_value
    count = 1
    
    # 💡 디버깅: 첫 픽셀 정보
    logging.warning(f"[RLE Encode] First pixel value: {first_pixel_value}")
    
    # RLE 생성
    run_index = 0
    for idx, val in enumerate(mask_flat[1:], 1):
        if val == current_value:
            count += 1
        else:
            rle.append(count)
            # 💡 처음 3개 런만 상세 출력
            if run_index < 3:
                logging.debug(f"  └─ Run {run_index}: {count} pixels of value {current_value}")
            current_value = val
            count = 1
            run_index += 1
    rle.append(count)  # 마지막 런 추가
    if run_index < 3:
        logging.debug(f"  └─ Run {run_index} (last): {count} pixels of value {current_value}")
    
    # 💡 보정: 첫 픽셀이 마스크(255)면 배경 런(0) 추가
    if first_pixel_value == 255:
        rle.insert(0, 0)
        logging.debug(f"[RLE Encode] First pixel is 255, inserted 0 at beginning")
    
    logging.debug(f"[RLE Encode] Total runs: {len(rle)}")
    
    return rle

def rle_decode(rle, shape):
    """
    RLE를 다시 마스크로 디코딩 (시각화 및 검증용)
    C++ 디코더와 정확히 동일한 로직
    
    Args:
        rle: RLE 인코딩된 리스트 [배경길이1, 마스크길이1, ...]
        shape: [H, W] 마스크 크기
    
    Returns:
        numpy array (H, W) with 0 and 255
    """
    if not rle or len(shape) < 2:
        return None
    
    H, W = shape
    mask = np.zeros(H * W, dtype=np.uint8)
    
    current_pixel = 0
    current_value = 0  # 배경(0)부터 시작
    
    logging.debug(f"[RLE Decode] Shape: {H}x{W}, Total pixels: {H*W}, RLE runs: {len(rle)}")
    
    for run_index, run_length in enumerate(rle):
        # 💡 처음 3개 런만 상세 출력
        if run_index < 3:
            logging.debug(f"  └─ RLE[{run_index}]: Fill {run_length} pixels with value {current_value} (starting at pixel {current_pixel})")
        
        # 현재 값으로 run_length만큼 채우기
        for _ in range(run_length):
            if current_pixel < H * W:
                mask[current_pixel] = current_value
                current_pixel += 1
        
        # 값 토글 (0 <-> 255)
        current_value = 255 if current_value == 0 else 0
    
    logging.debug(f"[RLE Decode] Filled {current_pixel} pixels total")
    
    return mask.reshape(H, W)

def visualize_results_sync(image, results, frame_count):
    """
    💡 시각화 수행 (동기 버전 - 시각화 스레드에서 호출)
    
    Args:
        image: 원본 이미지 (numpy array)
        results: 검출 결과 리스트
        frame_count: 프레임 번호
    """
    try:
        vis_image = image.copy()
        
        for cup_idx, cup_result in enumerate(results):
            # 컵 박스 그리기
            box = cup_result["box"]
            cv2.rectangle(vis_image, (box[0], box[1]), (box[2], box[3]), 
                         (255, 255, 0), 2)
            
            # 각 파트별로 마스크 시각화
            parts = ["body_handle", "body", "handle"]
            for part_name in parts:
                part_data = cup_result.get(part_name)
                if part_data is None:
                    continue
                
                cls_id = part_data["cls_id"]
                offset = part_data["offset"]
                mask_shape = part_data["mask_shape"]
                mask_rle = part_data["mask_rle"]
                center = part_data["center"]
                
                # RLE 디코딩
                mask = rle_decode(mask_rle, mask_shape)
                
                if mask is None:
                    continue
                
                # 색상 오버레이 생성
                color = COLOR_MAP.get(cls_id, (255, 255, 255))
                colored_mask = np.zeros((mask_shape[0], mask_shape[1], 3), dtype=np.uint8)
                colored_mask[mask > 0] = color
                
                # 원본 이미지 위치에 오버레이
                x1, y1 = offset
                x2, y2 = x1 + mask_shape[1], y1 + mask_shape[0]
                
                # 경계 체크
                x1_clip = max(0, x1)
                y1_clip = max(0, y1)
                x2_clip = min(vis_image.shape[1], x2)
                y2_clip = min(vis_image.shape[0], y2)
                
                # 마스크도 같은 영역으로 클립
                mask_x1 = x1_clip - x1
                mask_y1 = y1_clip - y1
                mask_x2 = mask_x1 + (x2_clip - x1_clip)
                mask_y2 = mask_y1 + (y2_clip - y1_clip)
                
                if mask_x2 > mask_x1 and mask_y2 > mask_y1:
                    roi = vis_image[y1_clip:y2_clip, x1_clip:x2_clip]
                    mask_roi = colored_mask[mask_y1:mask_y2, mask_x1:mask_x2]
                    
                    # 알파 블렌딩
                    blended = cv2.addWeighted(roi, 0.5, mask_roi, 0.5, 0)
                    vis_image[y1_clip:y2_clip, x1_clip:x2_clip] = blended
                
                # 중심점 그리기
                cv2.circle(vis_image, (center[0], center[1]), 5, (255, 255, 255), -1)
                cv2.circle(vis_image, (center[0], center[1]), 3, color, -1)
                
                # 라벨 표시
                label = CLASS_NAMES[cls_id]
                cv2.putText(vis_image, label, (center[0] + 8, center[1] - 8),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(vis_image, label, (center[0] + 8, center[1] - 8),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # 프레임 정보 표시
        info_text = f"Frame: {frame_count} | Cups: {len(results)}"
        cv2.putText(vis_image, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        # 이미지 저장
        output_path = os.path.join(VIS_OUTPUT_DIR, f"frame_{frame_count:05d}.jpg")
        cv2.imwrite(output_path, vis_image)
        logging.warning(f"💾 Saved visualization: {output_path}")
        
        # OpenCV 윈도우로 표시
        cv2.imshow("Detection Results", vis_image)
        cv2.waitKey(1)  # 1ms 대기 (이벤트 처리)
        
    except Exception as e:
        logging.error(f"❌ Visualization error: {e}\n{traceback.format_exc()}")

def visualization_worker():
    """
    💡 시각화 전용 스레드 워커
    큐에서 시각화 작업을 가져와 처리
    """
    global visualization_running
    logging.warning("🎨 Visualization thread started")
    
    # OpenCV 윈도우 생성 (이 스레드에서만)
    cv2.namedWindow("Detection Results", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Detection Results", 800, 600)
    
    while visualization_running:
        try:
            # 큐에서 작업 가져오기 (타임아웃 1초)
            task = visualization_queue.get(timeout=1.0)
            
            if task is None:  # 종료 신호
                break
            
            image, results, frame_count = task
            visualize_results_sync(image, results, frame_count)
            
        except queue.Empty:
            # 큐가 비어있으면 계속 대기
            continue
        except Exception as e:
            logging.error(f"❌ Visualization worker error: {e}\n{traceback.format_exc()}")
    
    cv2.destroyAllWindows()
    logging.warning("🎨 Visualization thread stopped")

def start_visualization_thread():
    """시각화 스레드 시작"""
    global visualization_thread, visualization_running
    
    if visualization_thread is not None and visualization_thread.is_alive():
        logging.warning("Visualization thread already running")
        return
    
    visualization_running = True
    visualization_thread = threading.Thread(target=visualization_worker, daemon=True)
    visualization_thread.start()
    logging.warning("✓ Visualization thread started")

def stop_visualization_thread():
    """시각화 스레드 종료"""
    global visualization_thread, visualization_running
    
    if visualization_thread is None:
        return
    
    visualization_running = False
    visualization_queue.put(None)  # 종료 신호
    visualization_thread.join(timeout=3.0)
    logging.warning("✓ Visualization thread stopped")

def queue_visualization(image, results, frame_count):
    """
    💡 시각화 작업을 큐에 추가 (비동기)
    
    Args:
        image: 원본 이미지
        results: 검출 결과
        frame_count: 프레임 번호
    """
    try:
        # 큐가 가득 차있으면 오래된 작업 제거
        if visualization_queue.full():
            try:
                visualization_queue.get_nowait()
                logging.warning("Visualization queue full, dropped old frame")
            except queue.Empty:
                pass
        
        # 새 작업 추가 (이미지 복사본 사용)
        visualization_queue.put((image.copy(), results, frame_count), block=False)
        
    except queue.Full:
        logging.warning("Failed to queue visualization - queue full")
    except Exception as e:
        logging.error(f"Error queuing visualization: {e}")

def detect_cups_and_parts(image, yolo_model, predictor, frame_count, visualize=True):
    """
    이미지에서 컵과 손잡이를 검출하여 결과를 JSON 형식으로 반환
    """
    results = []

    # 1. YOLO로 컵 검출
    yolo_results = yolo_model(image, verbose=False)
    cups = []
    scores = []
    
    cup_count_yolo = 0
    for r in yolo_results:
        for box in r.boxes:
            cls_id = int(box.cls[0])
            if cls_id == 41:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                score = box.conf[0].item()
                cups.append([x1, y1, x2, y2])
                scores.append(score)
                cup_count_yolo += 1
    
    logging.warning(f"========================================") 
    logging.warning(f"[Python Server] Starting detection on new frame.") 
    logging.warning(f"[YOLO Debug] Found {cup_count_yolo} initial cup bounding boxes.") 

    # NMS 적용
    keep = non_max_suppression(cups, scores, threshold=0.5)
    filtered_cups = [cups[i] for i in keep]

    # 마진 추가
    margin = 5
    expanded_cups = []
    for (x1, y1, x2, y2) in filtered_cups:
        x1 = max(0, x1 - margin)
        y1 = max(0, y1 - margin)
        x2 = min(image.shape[1], x2 + margin)
        y2 = min(image.shape[0], y2 + margin)
        expanded_cups.append((x1, y1, x2, y2))
        
    logging.warning(f"[NMS Debug] Filtered to {len(expanded_cups)} cups for segmentation.") 

    # 2. 각 컵 영역에 대해 Detectron2 세그멘테이션
    for cup_index, (x1, y1, x2, y2) in enumerate(expanded_cups):
        cup_crop = image[y1:y2, x1:x2].copy()
        if cup_crop.size == 0:
            continue
        
        logging.warning(f"--- Processing Cup Instance {cup_index + 1} (Crop: {x1},{y1} to {x2},{y2}) ---")

        H, W, _ = cup_crop.shape
        crop_center = np.array([W // 2, H // 2])

        # Detectron2 추론
        outputs = predictor(cup_crop)
        instances = outputs["instances"].to("cpu")

        # 인스턴스별 데이터 추출
        instances_data = []
        for i in range(len(instances)):
            mask_data = instances.pred_masks[i].numpy().astype(np.uint8) * 255
            cls_id = instances.pred_classes[i].item()

            # 중심 좌표 계산
            contours, _ = cv2.findContours(mask_data, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            M = cv2.moments(contours[0])
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            center = [int(cx), int(cy)]

            # 마스크를 RLE로 압축
            rle = rle_encode(mask_data)
            
            masked_pixels = np.sum(mask_data == 255)
            
            # 💡 마스크 값 통계
            unique_values = np.unique(mask_data)
            mask_min = np.min(mask_data)
            mask_max = np.max(mask_data)
            
            # 💡 첫 10개와 마지막 10개 픽셀 값 출력
            mask_flat = mask_data.flatten()
            first_10 = mask_flat[:10].tolist()
            last_10 = mask_flat[-10:].tolist()
            
            # 💡 RLE 상세 정보
            rle_preview = str(rle[:20]) + "..." if len(rle) > 20 else str(rle)
            
            logging.warning(f"[Python Mask Values] Part: {CLASS_NAMES[cls_id]}")
            logging.warning(f"  └─ Shape: {H}x{W} | Total: {H*W} pixels")
            logging.warning(f"  └─ Unique values: {unique_values.tolist()} | Min: {mask_min} | Max: {mask_max}")
            logging.warning(f"  └─ Masked pixels (255): {masked_pixels} ({masked_pixels*100/(H*W):.1f}%)")
            logging.warning(f"  └─ First 10 pixels: {first_10}")
            logging.warning(f"  └─ Last 10 pixels: {last_10}")
            logging.warning(f"  └─ RLE length: {len(rle)} | RLE: {rle_preview}")
            
            # 💡 RLE 검증: 다시 디코딩해서 원본과 비교
            decoded_mask = rle_decode(rle, [H, W])
            if decoded_mask is not None:
                decoded_pixels = np.sum(decoded_mask == 255)
                decoded_flat = decoded_mask.flatten()
                decoded_first_10 = decoded_flat[:10].tolist()
                decoded_last_10 = decoded_flat[-10:].tolist()
                
                match = np.array_equal(mask_data, decoded_mask)
                logging.warning(f"  └─ RLE Decode Check: {'✅ MATCH' if match else '❌ MISMATCH'}")
                if not match:
                    logging.error(f"     └─ Decoded pixels: {decoded_pixels}")
                    logging.error(f"     └─ Decoded first 10: {decoded_first_10}")
                    logging.error(f"     └─ Decoded last 10: {decoded_last_10}")
                    diff_count = np.sum(mask_data != decoded_mask)
                    logging.error(f"     └─ Different pixels: {diff_count}") 

            instances_data.append({
                "cls_id": int(cls_id),
                "center": center, 
                "mask_rle": rle,
                "mask_shape": [int(H), int(W)]
            })

        # 필터링 로직
        selected_body = None
        selected_handle = None
        selected_body_handle = None

        # body-handle 필터링
        body_handle_instances = [inst for inst in instances_data if inst["cls_id"] == 0]
        if body_handle_instances:
            distances = [np.linalg.norm(np.array(inst["center"]) - crop_center) 
                         for inst in body_handle_instances]
            min_idx = np.argmin(distances)
            selected_body_handle = body_handle_instances[min_idx]

        # body 필터링
        body_instances = [inst for inst in instances_data if inst["cls_id"] == CLASS_ID_BODY]
        if body_instances:
            distances = [np.linalg.norm(np.array(inst["center"]) - crop_center) 
                         for inst in body_instances]
            min_idx = np.argmin(distances)
            selected_body = body_instances[min_idx]

        # handle 필터링
        handle_instances = [inst for inst in instances_data if inst["cls_id"] == CLASS_ID_HANDLE]
        if selected_body is not None and handle_instances:
            body_center = np.array(selected_body["center"]) 
            distances = [np.linalg.norm(np.array(inst["center"]) - body_center) 
                         for inst in handle_instances]
            min_idx = np.argmin(distances)
            selected_handle = handle_instances[min_idx]
            
        logging.warning(f"[Selection Debug] Final selections (Body-Handle: {selected_body_handle is not None}, Body: {selected_body is not None}, Handle: {selected_handle is not None})")

        # 결과 구성
        cup_result = {
            "box": [int(x1), int(y1), int(x2), int(y2)],
            "body_handle": None,
            "body": None,
            "handle": None
        }

        if selected_body_handle:
            cup_result["body_handle"] = {
                "cls_id": selected_body_handle["cls_id"],
                "center": [selected_body_handle["center"][0] + x1, 
                           selected_body_handle["center"][1] + y1],
                "mask_rle": selected_body_handle["mask_rle"],
                "mask_shape": selected_body_handle["mask_shape"],
                "offset": [int(x1), int(y1)]
            }

        if selected_body:
            cup_result["body"] = {
                "cls_id": selected_body["cls_id"],
                "center": [selected_body["center"][0] + x1, 
                           selected_body["center"][1] + y1],
                "mask_rle": selected_body["mask_rle"],
                "mask_shape": selected_body["mask_shape"],
                "offset": [int(x1), int(y1)]
            }

        if selected_handle:
            cup_result["handle"] = {
                "cls_id": selected_handle["cls_id"],
                "center": [selected_handle["center"][0] + x1, 
                           selected_handle["center"][1] + y1],
                "mask_rle": selected_handle["mask_rle"],
                "mask_shape": selected_handle["mask_shape"],
                "offset": [int(x1), int(y1)]
            }

        results.append(cup_result)

    # 💡 시각화 작업을 큐에 추가 (비동기)
    if visualize and len(results) > 0:
        queue_visualization(image, results, frame_count)

    logging.warning(f"========================================")
    return results

def main():
    logging.warning("Starting cup detector server...") 

    # 💡 시각화 스레드 시작
    start_visualization_thread()

    # 모델 로드
    logging.warning("Loading models...") 
    try:
        cfg = setup_cfg_for_inference()
        predictor = DefaultPredictor(cfg)
        yolo_model = YOLO('./yolov8n.pt')
        logging.warning("✓ Models loaded successfully") 
    except Exception as e:
        logging.error(f"❌ Failed to load models: {e}")
        stop_visualization_thread()
        return

    # 공유 메모리 연결
    shm_image = try_attach_shared_memory(SHM_IMAGE_NAME, IMAGE_SIZE)
    if shm_image is None:
        logging.error("❌ Failed to initialize image shared memory. Exiting...")
        stop_visualization_thread()
        return

    shm_result = try_attach_shared_memory(SHM_RESULT_NAME, RESULT_SIZE)
    if shm_result is None:
        logging.error("❌ Failed to initialize result shared memory. Exiting...")
        shm_image.close()
        stop_visualization_thread()
        return

    shm_control = try_attach_shared_memory(SHM_CONTROL_NAME, CONTROL_SIZE)
    if shm_control is None:
        logging.error("❌ Failed to initialize control shared memory. Exiting...")
        shm_image.close()
        shm_result.close()
        stop_visualization_thread()
        return

    # 제어 플래그 초기화
    try:
        sem_control = posix_ipc.Semaphore(SEM_CONTROL_NAME, 0, 0o666)
        sem_control.acquire()
        shm_control.buf[OFFSET_NEW_FRAME] = 0
        shm_control.buf[OFFSET_RESULT_READY] = 0
        shm_control.buf[OFFSET_SHUTDOWN] = 0
        sem_control.release()
        logging.warning("✓ Initialized control flags") 
    except Exception as e:
        logging.error(f"❌ Failed to initialize control flags: {e}")
        stop_visualization_thread()
        return

    # 세마포어 연결
    try:
        sem_image = posix_ipc.Semaphore(SEM_IMAGE_NAME, 0, 0o666)
        sem_result = posix_ipc.Semaphore(SEM_RESULT_NAME, 0, 0o666)
    except Exception as e:
        logging.error(f"❌ Failed to connect to semaphores: {e}")
        stop_visualization_thread()
        return

    frame_count = 0
    try:
        logging.warning("💡 Server ready - Waiting for frames from Qt application...")
        logging.warning(f"💡 Visualization output directory: {VIS_OUTPUT_DIR}")
        
        while True:
            # 프레임 요청 플래그 확인
            sem_control.acquire()
            if shm_control.buf[OFFSET_NEW_FRAME] == 1:
                sem_control.release()

                start_time = time.time()
                frame_count += 1

                # 이미지 데이터 읽기
                sem_image.acquire()
                image_data = np.ndarray(
                    (IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS),
                    dtype=np.uint8,
                    buffer=shm_image.buf
                ).copy() 
                sem_image.release()

                # 💡 검출 수행 (시각화는 별도 스레드에서)
                results = detect_cups_and_parts(image_data, yolo_model, predictor, 
                                               frame_count, visualize=True)

                # 결과를 JSON으로 직렬화
                result_json = json.dumps(results)
                result_bytes = result_json.encode('utf-8')

                # 결과 크기 확인 및 쓰기
                if len(result_bytes) > RESULT_SIZE - 4:
                    logging.warning(f"Result too large ({len(result_bytes)} bytes), truncating")
                    result_bytes = result_bytes[:RESULT_SIZE-4]

                # 결과 쓰기
                sem_result.acquire()
                struct.pack_into('I', shm_result.buf, 0, len(result_bytes))
                shm_result.buf[4:4+len(result_bytes)] = result_bytes
                sem_result.release()

                # 플래그 업데이트
                sem_control.acquire()
                shm_control.buf[OFFSET_NEW_FRAME] = 0
                shm_control.buf[OFFSET_RESULT_READY] = 1
                sem_control.release()

                elapsed = time.time() - start_time
                logging.warning(f"Frame {frame_count}: Detected {len(results)} cups in {elapsed*1000:.1f}ms") 
            else:
                sem_control.release()

            # 종료 신호 확인
            sem_control.acquire()
            if shm_control.buf[OFFSET_SHUTDOWN] == 1:
                sem_control.release()
                logging.warning("\nShutdown signal received") 
                break
            sem_control.release()

            time.sleep(0.005) 

    except KeyboardInterrupt:
        logging.warning("\nReceived KeyboardInterrupt, shutting down...") 
    except Exception as e:
        logging.error(f"❌ Error in main loop: {e}\n{traceback.format_exc()}") 
    finally:
        # 정리
        logging.warning("Cleaning up shared memory and semaphores...") 
        
        # 💡 시각화 스레드 종료
        stop_visualization_thread()
        
        try: shm_image.close()
        except: logging.warning(f"Failed to close {SHM_IMAGE_NAME}")
        try: shm_result.close()
        except: logging.warning(f"Failed to close {SHM_RESULT_NAME}")
        try: shm_control.close()
        except: logging.warning(f"Failed to close {SHM_CONTROL_NAME}")
        
        try: sem_image.close()
        except: pass
        try: sem_result.close()
        except: pass
        try: sem_control.close()
        except: pass

        logging.warning("Server shutdown complete.")

def try_attach_shared_memory(name, size, retry_delay=0.5):
    """공유 메모리 연결"""
    shm = None
    while True:
        try:
            shm = shared_memory.SharedMemory(name=name, create=False, size=size)
            logging.warning(f"✓ Attached to existing shared memory: {name}")
            return shm
        except FileNotFoundError:
            logging.warning(f"Shared memory {name} not found. Retrying...")
            time.sleep(retry_delay)
        except Exception as e:
            logging.error(f"Failed to attach {name}: {e}. Retrying...")
            time.sleep(retry_delay)

    return None

if __name__ == "__main__":
    main()