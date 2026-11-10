"""물체 기억 · 두 카메라 합치기 — 테스트용 (확인되면 perception_node.py의 fuse · 트랙 삭제 부분에 옮길 예정)

  perception_node.py와 다른 점:
    ① 기억: 퍽·목표 구역·QR 판(REMEMBER_PREFIXES)은 안 보여도 트랙(기록 카드)을 지우지 않음 → /tracked_objects에 계속 나감
       지우는 때 = 그 자리가 카메라에 '가려지지 않고 잘 보였어야' 하는데(expected_visible) miss_count번 연속 못 찾음 (옮겨짐·집어 감)
       화면 밖 · 너무 멂 · 화면 가장자리 · 깊이 없음 · 앞에 다른 물체 → 못 봤다고 세지 않음
       사람·박스·신호등은 지금처럼 TRACK_TIMEOUT_S(2초) 안 보이면 지움 (움직이거나 색이 바뀌어 옛 정보가 위험)
    ② 합치기: 퍽 계열(puck · puck_lying)은 같은 종류로 붙임 — 같은 퍽을 pole(옆에서 '서있음')과 gripper(위에서 '자세 모름')가
       다르게 봐도 id가 하나. 자세는 카드에 따로 두고, 자세를 아는 관측이 오면 갱신 → 발행할 때 puck / puck_lying
       같은 퍽 계열이어도 색이 다르면 다른 물체
  위치는 추적 좌표(odom — 바닥에 고정)로 저장 · 로봇이 움직인 건 TF(카메라 → odom)가 알려 줌 → 로봇이 돌아도 저장 위치는 그대로

  검출 하나 = dict(class_name, color, posture('standing'|'lying'|''), z[3] (odom 위치), R[3×3] (관측 공분산), conf)
  view = (rot, trans, fx, fy, cx, cy, depth_m) — 그 카메라의 자세(카메라 → odom)·내부값·깊이 영상 [m] (지우기 판정용)
"""
import numpy as np

TRACK_TIMEOUT_S = 2.0  # 기억하지 않는 물체가 이만큼 어느 카메라에도 안 보이면 삭제 [s] (perception_node.py와 같음)
# 안 보여도 기억하는 물체 — 스스로 움직이지 않고 나중에 다시 찾아가야 하는 것. 클래스 이름이 이 접두어로 시작하면 기억
REMEMBER_PREFIXES = ("puck", "goal_zone", "station_zone", "qr_board")
PUCK_FAMILY = ("puck", "puck_lying")  # 자세만 다른 같은 종류 — 합칠 때 한 종류로 봄


def remembered(cls):
    """안 보여도 기억하는 클래스인가"""
    return cls.startswith(REMEMBER_PREFIXES)


def kind(cls):
    """합칠 때 쓰는 종류 — 퍽 계열은 모두 'puck' (자세는 따로)"""
    return "puck" if cls in PUCK_FAMILY else cls


def expected_visible(p, rot, trans, fx, fy, cx, cy, depth_m, max_range, edge_px, occlusion_m):
    """추적 좌표의 점 p가 이 카메라 영상에 '가려지지 않고 보였어야' 하는가 — 기억한 물체가 사라졌는지 판정할 때만 씀.
    아니라고 보는 경우 (= 못 봤다고 셀 수 없음):
      카메라 뒤 · 0.1m보다 가까움 · max_range보다 멂 / 화면 밖 · 가장자리 edge_px 안 /
      그 픽셀 둘레(5×5)에 깊이가 없음 / 그 픽셀 깊이가 점까지 거리보다 occlusion_m 넘게 가까움 (앞에 다른 물체·팔)"""
    pc = rot.T @ (np.asarray(p, float) - trans)  # 카메라 광학 좌표 (x 오른쪽 · y 아래 · z 앞)
    z = float(pc[2])
    if z < 0.1 or z > max_range:
        return False
    u, v = fx * pc[0] / z + cx, fy * pc[1] / z + cy
    h, w = depth_m.shape[:2]
    if not (edge_px <= u < w - edge_px and edge_px <= v < h - edge_px):
        return False
    ui, vi = int(u), int(v)
    win = depth_m[max(0, vi - 2):vi + 3, max(0, ui - 2):ui + 3]
    vals = win[win > 0.05]
    if vals.size == 0:
        return False
    return float(np.median(vals)) >= z - occlusion_m


class TrackMemory:
    """기록 카드 보관함 — fuse(합치기) · prune(지우기) · published(발행용 목록)"""

    def __init__(self, match_dist=0.4, process_noise=0.01, miss_count=3, miss_max_range=3.0,
                 miss_edge_px=40, miss_occlusion_m=0.10):
        self.match_dist = match_dist          # 같은 물체로 붙이는 거리 한도 [m] (perception.yaml match_dist)
        self.q = process_noise                # 예측 공정 잡음 [m²/s] — 안 움직이는 물체
        self.miss_count = miss_count          # 잘 보이는 자리에서 연속 못 찾은 횟수 — 이만큼이면 지움
        self.miss_max_range = miss_max_range  # 이보다 먼 자리는 못 찾아도 세지 않음 [m]
        self.miss_edge_px = miss_edge_px      # 화면 가장자리 이만큼 안은 세지 않음 [px]
        self.miss_occlusion_m = miss_occlusion_m  # 그 자리 깊이가 이만큼 넘게 가까우면 가려짐 [m]
        self.tracks = {}                      # id → 카드 dict(kind, color, posture, p, P, t, miss, seen{cam: t}, conf)
        self.next_id = 1

    def fuse(self, cam, t, dets):
        """한 카메라 한 프레임의 검출들을 카드에 1:1로 붙여 칼만 갱신 → 검출마다 붙은 id
        후보 = 같은 종류(퍽 계열은 하나) · 같은 색 · match_dist 안 → 마할라노비스 거리 작은 쌍부터. 남은 검출은 새 카드"""
        pred = {tid: tr["P"] + np.eye(3) * self.q * max(0.0, t - tr["t"]) for tid, tr in self.tracks.items()}
        pairs = []
        for i, d in enumerate(dets):
            for tid, tr in self.tracks.items():
                if tr["kind"] != kind(d["class_name"]) or tr["color"] != d.get("color", ""):
                    continue
                e = np.asarray(d["z"], float) - tr["p"]
                if float(np.linalg.norm(e)) >= self.match_dist:
                    continue
                pairs.append((float(e @ np.linalg.solve(pred[tid] + d["R"], e)), i, tid))
        used_d, used_t = {}, set()
        for _, i, tid in sorted(pairs):
            if i in used_d or tid in used_t:
                continue
            used_d[i], d, tr = tid, dets[i], self.tracks[tid]
            used_t.add(tid)
            K = pred[tid] @ np.linalg.inv(pred[tid] + d["R"])
            tr["p"] = tr["p"] + K @ (np.asarray(d["z"], float) - tr["p"])
            P = (np.eye(3) - K) @ pred[tid]
            tr["P"] = 0.5 * (P + P.T)
            tr["t"] = max(tr["t"], t)
            tr["miss"] = 0                       # 다시 잡혔으니 '연속 못 찾음'을 처음부터
            tr["seen"][cam] = t
            tr["conf"] = d.get("conf", 0.5)
            if d.get("posture"):                 # 자세를 아는 관측만 자세를 바꿈 (위에서 본 '모름'은 덮어쓰지 않음)
                tr["posture"] = d["posture"]
        for i, d in enumerate(dets):
            if i in used_d:
                continue
            self.tracks[self.next_id] = {
                "kind": kind(d["class_name"]), "color": d.get("color", ""),
                "posture": d.get("posture") or ("lying" if d["class_name"] == "puck_lying" else ""),
                "p": np.asarray(d["z"], float), "P": np.asarray(d["R"], float), "t": t, "miss": 0,
                "seen": {cam: t}, "conf": d.get("conf", 0.5)}
            used_d[i] = self.next_id
            self.next_id += 1
        return [used_d[i] for i in range(len(dets))]

    def prune(self, t, matched, view):
        """한 카메라 한 추론 뒤 카드 정리 — matched = 이번에 검출이 붙은 id들 · view = 그 카메라 (rot, trans, fx, fy, cx, cy, depth_m)
          기억 안 하는 물체: TRACK_TIMEOUT_S 넘게 어느 카메라에도 안 보이면 삭제 (지금 perception_node와 같음)
          기억하는 물체: 시간으로는 안 지움. 이번에 안 잡혔는데 그 자리가 잘 보였어야 하면 miss +1 → miss_count면 삭제"""
        for tid, tr in list(self.tracks.items()):
            if not remembered(tr["kind"]):
                if t - tr["t"] > TRACK_TIMEOUT_S:
                    del self.tracks[tid]
                continue
            if tid in matched:
                continue
            if expected_visible(tr["p"], *view, self.miss_max_range, self.miss_edge_px, self.miss_occlusion_m):
                tr["miss"] += 1
                if tr["miss"] >= self.miss_count:
                    del self.tracks[tid]

    def published(self):
        """발행용 (id, class_name, color, p) — 퍽은 자세로 이름을 나눔 (설계: puck / puck_lying)"""
        out = []
        for tid, tr in sorted(self.tracks.items()):
            name = tr["kind"]
            if name == "puck" and tr["posture"] == "lying":
                name = "puck_lying"
            out.append((tid, name, tr["color"], tr["p"]))
        return out
