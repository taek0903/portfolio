"""RealSense + ArUco 로 'Area 1' 인식 테스트 (올인원)

- ROS 토픽에 의존하지 않고 이 파일 하나가 RealSense 카메라도 직접 연다.
- 이미 realsense2_camera 노드가 카메라 USB 를 잡고 있으면
  자동으로 감지해서 그 프로세스를 종료시키고 다시 연다.
- 작업 공간에 이미 ArUco 마커 ID 1, 2, 3, 4 가 네 꼭짓점으로 배치되어 있다는 가정.

하는 일:
    1) pyrealsense2 로 color stream 직접 open
    2) 흔히 쓰는 ArUco 딕셔너리들을 자동 순회 검출 (ID 1,2,3,4)
    3) 네 중심점을 잇는 사각형(Area 1) 을 한 영역으로 그림
       - centroid 기준 각도순 정렬 → 자기교차 없는 폴리곤
    4) 마우스 위치가 영역 안/밖인지 point-in-polygon 으로 실시간 표시

조작:
    q : 종료
    s : 현재 화면 PNG 저장 (./aruco_snapshots/)
    d : 현재 사용 중인 딕셔너리 고정/해제 토글
    --keep-ros : realsense2_camera 가 돌고 있어도 죽이지 않음 (그냥 실패)
"""

import os
os.environ['QT_QPA_PLATFORM'] = 'xcb'
os.environ['QT_LOGGING_RULES'] = '*.debug=false;qt.qpa.*=false'

import sys
import time
import subprocess
import signal
import numpy as np
import cv2
import cv2.aruco as aruco
import pyrealsense2 as rs


EXPECTED_IDS = [1, 2, 3, 4]
AREA_NAME = 'Area 1'
WIDTH, HEIGHT, FPS = 640, 480, 30

CANDIDATE_DICTS = [
    ('DICT_4X4_50',  aruco.DICT_4X4_50),
    ('DICT_5X5_50',  aruco.DICT_5X5_50),
    ('DICT_6X6_50',  aruco.DICT_6X6_50),
    ('DICT_7X7_50',  aruco.DICT_7X7_50),
    ('DICT_4X4_100', aruco.DICT_4X4_100),
    ('DICT_5X5_100', aruco.DICT_5X5_100),
    ('DICT_6X6_100', aruco.DICT_6X6_100),
    ('DICT_ARUCO_ORIGINAL', aruco.DICT_ARUCO_ORIGINAL),
]

SNAPSHOT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'aruco_snapshots')

CONFLICT_PATTERNS = [
    'realsense2_camera_node',
    'rs_launch',
    'rs_align_depth_launch',
]


def kill_camera_holders(verbose=True) -> int:
    """pyrealsense2 가 USB 를 열 수 있도록, RealSense 를 물고 있는 알려진
    ROS 프로세스들을 찾아서 종료시킨다. 종료한 프로세스 수를 반환.
    """
    try:
        out = subprocess.check_output(['ps', '-eo', 'pid,args'], text=True)
    except Exception as e:
        if verbose:
            print(f'[warn] ps failed: {e}')
        return 0

    killed = 0
    for line in out.splitlines():
        if not any(p in line for p in CONFLICT_PATTERNS):
            continue
        parts = line.strip().split(None, 1)
        if len(parts) < 2:
            continue
        try:
            pid = int(parts[0])
        except ValueError:
            continue
        if pid == os.getpid():
            continue
        try:
            os.kill(pid, signal.SIGTERM)
            killed += 1
            if verbose:
                print(f'[kill] SIGTERM pid={pid}  ({parts[1][:100]})')
        except ProcessLookupError:
            pass
        except PermissionError:
            if verbose:
                print(f'[warn] no permission to kill pid={pid}; '
                      f'try: sudo kill {pid}')

    if killed:
        time.sleep(2.0)  
        for line in out.splitlines():
            if not any(p in line for p in CONFLICT_PATTERNS):
                continue
            parts = line.strip().split(None, 1)
            if len(parts) < 2:
                continue
            try:
                pid = int(parts[0])
            except ValueError:
                continue
            try:
                os.kill(pid, 0)            
                os.kill(pid, signal.SIGKILL) 
                if verbose:
                    print(f'[kill] SIGKILL pid={pid}')
            except (ProcessLookupError, PermissionError):
                pass
        time.sleep(1.0)
    return killed


def hardware_reset_all(verbose=True):
    try:
        ctx = rs.context()
        for dev in ctx.query_devices():
            name = dev.get_info(rs.camera_info.name)
            try:
                dev.hardware_reset()
                if verbose:
                    print(f'[reset] {name}')
            except Exception as e:
                if verbose:
                    print(f'[warn] reset failed for {name}: {e}')
    except Exception as e:
        if verbose:
            print(f'[warn] cannot enumerate devices: {e}')
    time.sleep(3.0) 


def start_pipeline(keep_ros: bool):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)

    def _try_start():
        pipeline.start(config)

    try:
        _try_start()
        return pipeline
    except RuntimeError as e:
        msg = str(e).lower()
        if 'busy' not in msg and 'no device connected' not in msg and 'cannot open' not in msg:
            raise
        print(f'[warn] pipeline.start() failed: {e}')

    if not keep_ros:
        n = kill_camera_holders(verbose=True)
        if n == 0:
            print('[info] no known ROS camera holder found; attempting '
                  'hardware reset only')
        hardware_reset_all(verbose=True)
    else:
        hardware_reset_all(verbose=True)

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    pipeline.start(config)
    return pipeline


def build_detectors():
    params = aruco.DetectorParameters()
    return [(name, aruco.ArucoDetector(aruco.getPredefinedDictionary(did), params))
            for name, did in CANDIDATE_DICTS]


def detect_any(gray, detectors, preferred_idx=None):
    if preferred_idx is not None:
        name, detector = detectors[preferred_idx]
        corners, ids, _ = detector.detectMarkers(gray)
        return name, corners, ids, preferred_idx

    best_name, best_corners, best_ids, best_hit, best_idx = None, (), None, -1, -1
    for i, (name, detector) in enumerate(detectors):
        corners, ids, _ = detector.detectMarkers(gray)
        hit = 0 if ids is None else sum(int(x) in EXPECTED_IDS for x in ids.flatten())
        if hit > best_hit:
            best_name, best_corners, best_ids, best_hit, best_idx = name, corners, ids, hit, i
            if hit == len(EXPECTED_IDS):
                break
    return best_name, best_corners, best_ids, best_idx


def marker_centers_by_id(corners, ids):
    centers = {}
    if ids is None:
        return centers
    for c, mid in zip(corners, ids.flatten()):
        pts = c.reshape(-1, 2)
        centers[int(mid)] = (float(pts[:, 0].mean()), float(pts[:, 1].mean()))
    return centers


def order_polygon(points):
    """centroid 기준 각도순 정렬 → 자기교차 없는 폴리곤."""
    pts = np.asarray(points, dtype=np.float32)
    c = pts.mean(axis=0)
    angles = np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0])
    return pts[np.argsort(angles)]


def draw_area(img, centers, hover_xy=None):
    raw = [centers[i] for i in EXPECTED_IDS]
    poly = order_polygon(raw).astype(np.int32)

    overlay = img.copy()
    cv2.fillPoly(overlay, [poly], (0, 200, 0))
    cv2.addWeighted(overlay, 0.25, img, 0.75, 0, dst=img)
    cv2.polylines(img, [poly], isClosed=True, color=(0, 255, 0), thickness=2)

    for mid in EXPECTED_IDS:
        x, y = centers[mid]
        cv2.putText(img, f'#{mid}', (int(x) + 6, int(y) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    cx = int(poly[:, 0].mean())
    cy = int(poly[:, 1].mean())
    cv2.circle(img, (cx, cy), 4, (0, 255, 0), -1)
    cv2.putText(img, AREA_NAME, (cx - 40, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    inside = None
    if hover_xy is not None:
        r = cv2.pointPolygonTest(poly.astype(np.float32),
                                 (float(hover_xy[0]), float(hover_xy[1])), False)
        inside = r >= 0
        col = (0, 255, 0) if inside else (0, 0, 255)
        cv2.circle(img, hover_xy, 6, col, 2)
        tag = 'INSIDE Area 1' if inside else 'OUTSIDE'
        cv2.putText(img, tag, (hover_xy[0] + 10, hover_xy[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2)

    return poly, inside


def draw_status(img, dict_name, found_ids, fps, locked):
    missing = [i for i in EXPECTED_IDS if i not in found_ids]
    ready = (len(missing) == 0)
    color = (0, 255, 0) if ready else (0, 165, 255)
    title = 'AREA 1 LOCKED' if ready else f'WAITING: missing {missing}'
    lock_tag = '[locked]' if locked else '[auto]'
    cv2.rectangle(img, (0, 0), (img.shape[1], 30), (0, 0, 0), -1)
    cv2.putText(img, f'{title}  dict={dict_name}{lock_tag}  fps={fps:5.1f}',
                (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)


def main():
    keep_ros = '--keep-ros' in sys.argv
    os.makedirs(SNAPSHOT_DIR, exist_ok=True)

    print('[RealSense] opening color stream (direct, no ROS)...')
    pipeline = start_pipeline(keep_ros=keep_ros)
    print(f'[RealSense] running {WIDTH}x{HEIGHT}@{FPS} bgr8')

    detectors = build_detectors()
    print('[ArUco] candidate dicts:', ', '.join(n for n, _ in CANDIDATE_DICTS))
    print('Press "q" to quit, "s" to save snapshot, "d" to lock current dict.')

    mouse_xy = [None]
    win_name = 'ArUco Area 1 (hover=in/out test, q=quit, s=save, d=lock dict)'
    cv2.namedWindow(win_name)

    def _on_mouse(event, x, y, flags, userdata):
        if event == cv2.EVENT_MOUSEMOVE:
            mouse_xy[0] = (x, y)
    cv2.setMouseCallback(win_name, _on_mouse)

    last = time.time()
    fps = 0.0
    locked_idx = None
    area_reported = False

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            img = np.asanyarray(color_frame.get_data())

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            dict_name, corners, ids, used_idx = detect_any(gray, detectors, locked_idx)
            aruco.drawDetectedMarkers(img, corners, ids)

            centers = marker_centers_by_id(corners, ids)
            all_found = all(i in centers for i in EXPECTED_IDS)

            if all_found:
                poly, _ = draw_area(img, centers, hover_xy=mouse_xy[0])
                if not area_reported:
                    area_px = float(cv2.contourArea(poly.astype(np.float32)))
                    print(f'[Area 1] locked. polygon (CCW) px='
                          f'{[tuple(p.tolist()) for p in poly]}  area={area_px:.0f}px^2')
                    area_reported = True
            else:
                area_reported = False

            now = time.time()
            fps = 0.9 * fps + 0.1 * (1.0 / max(now - last, 1e-6))
            last = now
            draw_status(img, dict_name or '-', list(centers.keys()), fps,
                        locked=(locked_idx is not None))

            cv2.imshow(win_name, img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            if key == ord('s'):
                fname = os.path.join(SNAPSHOT_DIR, f'area1_{int(time.time())}.png')
                cv2.imwrite(fname, img)
                print(f'[saved] {fname}')
            if key == ord('d'):
                if locked_idx is None and used_idx is not None and used_idx >= 0:
                    locked_idx = used_idx
                    print(f'[lock] dictionary -> {CANDIDATE_DICTS[locked_idx][0]}')
                else:
                    locked_idx = None
                    print('[unlock] back to auto-scan')
    finally:
        try:
            pipeline.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()
        print('Done.')


if __name__ == '__main__':
    main()
