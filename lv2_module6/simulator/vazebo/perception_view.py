"""인지 결과 읽기·그리기 공용 — 지도 · RGB-SEG · 감지 물체 보기 창 · 로봇 3D · 조작 창 3D가 같이 쓴다 (tkinter 없음)

  /tracked_objects              두 카메라 융합 트랙 (보라)
  /perception/<cam>/detections  융합 전 카메라별 검출 (pole_front_cam 주황 · gripper_cam 분홍), id = 붙은 트랙
좌표: 인지 결과는 map 기준, 시뮬레이터 미로 좌표 = map + spawn (map→odom 보정은 무시 — 지도와 같은 규약).
3D 뷰(로봇 기준)는 **로봇이 아는 자기 위치**(TF map→odom∘odom→base_footprint — perception이 쓴 것과 같은 것)로 로봇 기준에
옮긴다 — 위치추정(EKF) 오차가 빠지고 인지 오차만 보인다. 지도(2D)는 미로 위라 위치추정 오차까지 같이 보인다.
"""
import cv2
import numpy as np

from bridge import read_msg

FRESH_S = 2.0        # 파일이 이 안에 바뀐 것만 '지금 받은 것' (이전 실행 파일은 안 그림)
FUSED_COLOR = (150, 0, 150)  # BGR
CAM_COLORS = {'pole_front_cam': (0, 170, 255), 'gripper_cam': (147, 20, 255)}  # 카메라 시야선 · 카메라별 검출 색 (BGR)
# 상자 12모서리 (꼭짓점 번호: x·y·z 비트)
_EDGES = [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7)]


def tracked_objects(fresh_s=FRESH_S):
    """지금 받은 /tracked_objects 목록 (없거나 오래되면 [])"""
    data, age = read_msg('tracked_objects')
    if age is None or age > fresh_s:
        return []
    return (data or {}).get('objects', [])


def camera_detections(fresh_s=FRESH_S):
    """{cam: [융합 전 검출]} — /perception/<cam>/detections 중 지금 받은 것만"""
    out = {}
    for cam in CAM_COLORS:
        data, age = read_msg(f'perception/{cam}/detections')
        if age is not None and age <= fresh_s:
            out[cam] = (data or {}).get('objects', [])
    return out


def fused_ids(cams_det):
    """두 카메라 검출이 모두 붙은 트랙 id = 지금 융합 중인 물체"""
    if len(cams_det) < 2:
        return set()
    return set.intersection(*({o.get('id') for o in objs} for objs in cams_det.values()))


def detected_key():
    """3D 뷰를 다시 그릴지 판단용 — 인지 결과가 바뀌면 달라짐"""
    objs = tracked_objects()
    dets = camera_detections()
    pos = lambda o: tuple(round(o['pose']['pose']['position'][k], 2) for k in 'xyz')
    return (tuple((o['id'], pos(o)) for o in objs),
            tuple((c, tuple((o['id'], pos(o)) for o in v)) for c, v in sorted(dets.items())))


def draw_detected_3d(img, view, size, base_map):
    """로봇 3D 뷰(OrbitView, 로봇 루트 프레임) 위에 감지 물체 — 융합 트랙 = 보라 와이어 상자 + 라벨(F = 두 카메라),
    카메라별 검출 = 카메라 색 점 + 트랙까지 선. 가림 없이 겹쳐 그림 (실제 물체 모형을 가리지 않게 면 대신 선).
    base_map: 로봇이 아는 자기 위치 (map x, y, yaw) — perception이 map으로 옮길 때 쓴 TF와 같은 것"""
    bx, by, byaw = base_map
    c_, s_ = np.cos(byaw), np.sin(byaw)
    R = np.array([[c_, -s_, 0.0], [s_, c_, 0.0], [0.0, 0.0, 1.0]])
    t = np.array([bx, by, 0.0])

    def to_robot(p):  # map → 로봇 루트 프레임
        return (np.asarray(p, float) - t) @ R

    def pos(o):
        p = o['pose']['pose']['position']
        return np.array([p['x'], p['y'], p['z']], float)

    dets = camera_detections()
    both = fused_ids(dets)
    centers = {}
    for o in tracked_objects():
        try:
            p = pos(o)
            s = o.get('size', {})
            sx, sy, sz = (max(float(s.get(k, 0.06)), 0.02) for k in 'xyz')
        except (KeyError, TypeError, ValueError):
            continue
        # 위치는 카메라 쪽 앞면 점 — 상자는 그 점을 중심으로, 바닥 아래로는 안 내려가게
        c = np.array([p[0], p[1], max(p[2], sz / 2)])
        half = np.array([sx, sy, sz]) / 2
        corners = np.array([c + half * [1 if i & 1 else -1, 1 if i & 2 else -1, 1 if i & 4 else -1] for i in range(8)])
        uv, z = view.project(np.vstack([to_robot(corners), to_robot([c])]), size)
        if (z < 0.02).any():
            continue
        q = uv.astype(int)
        for a, b in _EDGES:
            cv2.line(img, tuple(q[a]), tuple(q[b]), FUSED_COLOR, 1, cv2.LINE_AA)
        centers[o['id']] = tuple(q[8])
        top = q[:8][np.argmin(q[:8, 1])]
        cv2.putText(img, f"{o.get('class_name', '?')}#{o['id']}" + (' F' if o['id'] in both else ''),
                    (int(top[0]) + 4, int(top[1]) - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (220, 120, 220), 1, cv2.LINE_AA)
    for cam, objs in dets.items():
        col = CAM_COLORS[cam]
        for o in objs:
            try:
                uv, z = view.project([to_robot(pos(o))], size)
            except (KeyError, TypeError, ValueError):
                continue
            if z[0] < 0.02:
                continue
            c = tuple(uv[0].astype(int))
            cv2.circle(img, c, 3, col, -1, cv2.LINE_AA)
            if o.get('id') in centers:
                cv2.line(img, c, centers[o['id']], col, 1, cv2.LINE_AA)
