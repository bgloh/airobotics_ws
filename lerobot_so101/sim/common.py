"""SO-101 실습 공통 설정: 관절 이름, 단위 변환, 경로점(waypoint) 궤적.

단위 약속
- 실제 로봇(LeRobot, use_degrees=True): 팔 관절은 degree(도), gripper는 0(닫힘)~100(열림)
- MuJoCo: 모든 관절이 radian(라디안)
"""
import numpy as np

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

# sim-to-real 맞추기: LAB 1-4에서 실제 팔과 시뮬레이션을 나란히 움직여 보고
# 방향이 반대인 관절은 -1, 자세가 일정하게 어긋나는 관절은 OFFSET_DEG에 그 차이를 적는다.
SIGN = {j: 1.0 for j in JOINTS}
OFFSET_DEG = {j: 0.0 for j in JOINTS}

# MuJoCo 모델(so101_new_calib.xml)의 gripper 관절 범위(도). LeRobot 0~100을 이 범위로 펼친다.
GRIP_LO_DEG, GRIP_HI_DEG = -10.0, 100.0

FPS = 30  # LeRobot 기본 제어 주기와 같게 (초당 30번 명령)


def real_to_sim(values: dict) -> np.ndarray:
    """LeRobot 값 {"shoulder_pan.pos": 도, ...} → MuJoCo ctrl 배열(rad)."""
    q = np.zeros(len(JOINTS))
    for i, j in enumerate(JOINTS):
        v = float(values[f"{j}.pos"])
        if j == "gripper":
            deg = GRIP_LO_DEG + v / 100.0 * (GRIP_HI_DEG - GRIP_LO_DEG)
        else:
            deg = SIGN[j] * v + OFFSET_DEG[j]
        q[i] = np.deg2rad(deg)
    return q


def sim_to_real(q_rad) -> dict:
    """MuJoCo 관절각(rad) → LeRobot 값 dict (real_to_sim의 역변환)."""
    out = {}
    for i, j in enumerate(JOINTS):
        deg = float(np.rad2deg(q_rad[i]))
        if j == "gripper":
            v = (deg - GRIP_LO_DEG) / (GRIP_HI_DEG - GRIP_LO_DEG) * 100.0
        else:
            v = (deg - OFFSET_DEG[j]) * SIGN[j]
        out[f"{j}.pos"] = v
    return out


# ── 경로점: "A에서 집어 B에 놓기" 모양의 동작 (LeRobot 단위: 도, gripper 0~100) ──
#            pan  lift  elbow wrist roll grip
WAYPOINTS = [
    [  0, -60,  60,  60, 0,  0],   # W0 준비 자세
    [ 30, -10,  30,  70, 0, 80],   # W1 A 위, gripper 열기
    [ 30,   5,  20,  70, 0, 80],   # W2 A로 내려가기
    [ 30,   5,  20,  70, 0,  0],   # W3 gripper 닫기 (집기)
    [ 30, -10,  30,  70, 0,  0],   # W4 들어 올리기
    [-30, -10,  30,  70, 0,  0],   # W5 B 위로 이동
    [-30,   5,  20,  70, 0,  0],   # W6 B로 내려가기
    [-30,   5,  20,  70, 0, 80],   # W7 gripper 열기 (놓기)
    [  0, -60,  60,  60, 0,  0],   # W8 준비 자세로 복귀
]
SEGMENT_TIME_S = 1.5  # 경로점 사이 이동 시간(초)


def cubic(q0, q1, T, t):
    """3차 다항식 보간(시작·끝 속도 0) — 5장 trajectory planning과 같은 식.
    q(t) = q0 + (q1 - q0) * (3s^2 - 2s^3),  s = t / T
    """
    s = np.clip(t / T, 0.0, 1.0)
    return q0 + (q1 - q0) * (3 * s**2 - 2 * s**3)


def make_trajectory(waypoints=WAYPOINTS, seg_t=SEGMENT_TIME_S, fps=FPS):
    """경로점을 1/fps 초 간격의 목표값 배열(N×6, LeRobot 단위)로 만든다."""
    wp = np.asarray(waypoints, dtype=float)
    n = int(round(seg_t * fps))
    rows = []
    for a, b in zip(wp[:-1], wp[1:]):
        for k in range(n):
            rows.append(cubic(a, b, seg_t, k / fps))
    rows.append(wp[-1])
    return np.array(rows)


def row_to_action(row) -> dict:
    return {f"{j}.pos": float(v) for j, v in zip(JOINTS, row)}
