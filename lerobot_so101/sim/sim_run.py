"""LAB 1-2: 경로점 궤적을 MuJoCo에서 실행하고 목표 vs 실제 관절각을 기록한다.

python sim_run.py --scene ~/so101_lab/SO-ARM100/Simulation/SO101/scene.xml --viewer
python sim_run.py --scene ... --out sim_log.csv   # 화면 없이 기록만
"""
import argparse, csv, time
import numpy as np
import mujoco
import mujoco.viewer
from common import JOINTS, FPS, SEGMENT_TIME_S, make_trajectory, real_to_sim, sim_to_real, row_to_action

p = argparse.ArgumentParser()
p.add_argument("--scene", required=True, help="SO-ARM100/Simulation/SO101/scene.xml 경로")
p.add_argument("--viewer", action="store_true", help="MuJoCo 창으로 보기 (실시간 속도)")
p.add_argument("--out", default="sim_log.csv")
p.add_argument("--seg-time", type=float, default=SEGMENT_TIME_S, help="경로점 사이 이동 시간(초). real_run.py와 같은 값으로")
# '다른 로봇' 만들기: 모델 값을 바꿔 같은 명령이 어떻게 달라지는지 본다 (sim-to-real gap 체험)
p.add_argument("--kp-scale", type=float, default=1.0, help="위치 제어 gain(kp) 배율, 예: 0.05")
p.add_argument("--friction-scale", type=float, default=1.0, help="관절 마찰(frictionloss) 배율, 예: 5")
p.add_argument("--delay", type=int, default=0, help="명령 전달 지연(프레임, 1프레임=1/30초)")
args = p.parse_args()

m = mujoco.MjModel.from_xml_path(args.scene)
# kp 바꾸기: position actuator는 힘 = kp*(목표-현재) - kv*속도 → gainprm[0]=kp, biasprm[1]=-kp
m.actuator_gainprm[:, 0] *= args.kp_scale
m.actuator_biasprm[:, 1] *= args.kp_scale
m.dof_frictionloss[:] *= args.friction_scale
d = mujoco.MjData(m)
substeps = int(round(1.0 / (FPS * m.opt.timestep)))  # 명령 1번 사이 물리 계산 횟수
traj = make_trajectory(seg_t=args.seg_time)

# 첫 경로점 자세에서 시작 (실제 로봇은 real_run.py가 천천히 이 자세로 옮겨 준다)
d.qpos[:] = real_to_sim(row_to_action(traj[0]))
d.ctrl[:] = d.qpos[:]
mujoco.mj_forward(m, d)

viewer = mujoco.viewer.launch_passive(m, d) if args.viewer else None
rows, max_con = [], 0
for k, target in enumerate(traj):
    t0 = time.perf_counter()
    sent = traj[max(0, k - args.delay)]                  # 지연이 있으면 몇 프레임 전 명령이 도착
    d.ctrl[:] = real_to_sim(row_to_action(sent))         # ① 명령 보내기
    for _ in range(substeps):                            # ② 물리 1/30초 진행
        mujoco.mj_step(m, d)
    actual = sim_to_real(d.qpos)                         # ③ 실제 자세 읽기
    tip = d.site("gripperframe").xpos                    # gripper 끝 위치(FK 결과)
    max_con = max(max_con, d.ncon)
    rows.append([k / FPS, *target, *[actual[f"{j}.pos"] for j in JOINTS], *tip, d.ncon])
    if viewer:
        viewer.sync()
        time.sleep(max(0.0, 1.0 / FPS - (time.perf_counter() - t0)))
if viewer:
    viewer.close()

head = ["t"] + [f"target_{j}" for j in JOINTS] + [f"actual_{j}" for j in JOINTS] + ["tip_x", "tip_y", "tip_z", "contacts"]
with open(args.out, "w", newline="") as f:
    csv.writer(f).writerows([head] + rows)

a = np.array(rows)
err = a[:, 7:13] - a[:, 1:7]
print(f"저장: {args.out}  ({len(rows)}개 시점, {len(rows) / FPS:.1f}초)")
print("관절별 RMS 오차(도):", {j: round(float(np.sqrt((err[:, i] ** 2).mean())), 2) for i, j in enumerate(JOINTS[:5])})
print(f"gripper 끝 최저 높이: {a[:, 15].min() * 100:.1f} cm,  최대 접촉 수: {max_con}")
