"""LAB 1-3: sim_run.py와 '똑같은' 궤적을 실제 SO-101 follower에 보내고 측정값을 기록한다.

python real_run.py --dry-run                                   # 연결 없이 궤적만 확인
python real_run.py --step --port /dev/ttyACM0 --id follower_g1 # 경로점을 하나씩 천천히 (처음엔 꼭 이것부터)
python real_run.py --port /dev/ttyACM0 --id follower_g1 --seg-time 3.0   # 전체 궤적, 느리게
python real_run.py --port /dev/ttyACM0 --id follower_g1                  # 전체 궤적, 기본 속도(1.5초/구간)
안전: 팔 주변 40 cm 비우기, 한 사람은 비상 시 follower 전원을 뽑을 준비. 끝나면 torque가 꺼지므로 팔을 받친다.
"""
import argparse, csv, time
import numpy as np
from common import JOINTS, FPS, WAYPOINTS, SEGMENT_TIME_S, make_trajectory, row_to_action

p = argparse.ArgumentParser()
p.add_argument("--port", default="/dev/ttyACM0")
p.add_argument("--id", default="follower_g1", help="calibration 때 쓴 id와 같아야 함")
p.add_argument("--out", default="real_log.csv")
p.add_argument("--seg-time", type=float, default=SEGMENT_TIME_S, help="경로점 사이 이동 시간(초). sim_run.py와 같은 값으로")
p.add_argument("--step", action="store_true", help="경로점마다 3초에 걸쳐 이동 후 멈추고, 목표와 실제 값을 비교")
p.add_argument("--dry-run", action="store_true")
args = p.parse_args()

traj = make_trajectory(seg_t=args.seg_time)
print(f"궤적: {len(traj)}개 명령, {len(traj) / FPS:.1f}초 (구간 {args.seg_time}초), 첫 자세 {traj[0].round(1).tolist()}")
if args.dry_run:
    raise SystemExit

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

# max_relative_target: 한 번의 명령으로 움직일 수 있는 최대 각도(도). 큰 점프를 막는 안전장치
robot = SO101Follower(SO101FollowerConfig(port=args.port, id=args.id, max_relative_target=10.0))
robot.connect()


def read():
    obs = robot.get_observation()
    return np.array([obs[f"{j}.pos"] for j in JOINTS])


def play(rows, log=None):
    t_start = time.perf_counter()
    for k, target in enumerate(rows):
        robot.send_action(row_to_action(target))
        if log is not None:
            log.append([k / FPS, *target, *read()])
        # 다음 명령 시각까지 기다리기 (30 Hz 유지)
        time.sleep(max(0.0, t_start + (k + 1) / FPS - time.perf_counter()))


def ramp_to(goal, seconds=3.0):
    now = read()
    play([now + (np.asarray(goal, float) - now) * s for s in np.linspace(0, 1, int(seconds * FPS))])


log = []
try:
    if args.step:
        # 경로점 점검 모드: 하나씩 가서 멈추고 숫자를 비교한다 (기록 파일 없음)
        for i, wp in enumerate(WAYPOINTS):
            ans = input(f"\nW{i} {wp} 로 이동하려면 Enter, 그만하려면 q + Enter: ")
            if ans.strip().lower() == "q":
                break
            ramp_to(wp)
            time.sleep(1.0)
            got = read()
            print(f"{'joint':14s} {'목표':>7s} {'실제':>7s} {'차이':>7s}")
            for j, a, b in zip(JOINTS, wp, got):
                print(f"{j:14s} {a:7.1f} {b:7.1f} {b - a:7.1f}")
    else:
        # 1) 지금 자세 → 첫 경로점: 3초 동안 천천히 (직선 보간)
        ramp_to(traj[0])
        time.sleep(1.0)
        # 2) 본 궤적 실행 + 기록
        play(traj, log)
finally:
    input("\n팔을 손으로 받친 뒤 Enter를 누르면 torque를 끕니다 ...")
    robot.disconnect()

if log:
    head = ["t"] + [f"target_{j}" for j in JOINTS] + [f"actual_{j}" for j in JOINTS]
    with open(args.out, "w", newline="") as f:
        csv.writer(f).writerows([head] + log)
    a = np.array(log)
    err = a[:, 7:13] - a[:, 1:7]
    print(f"저장: {args.out}")
    print("관절별 RMS 오차(도):", {j: round(float(np.sqrt((err[:, i] ** 2).mean())), 2) for i, j in enumerate(JOINTS[:5])})
