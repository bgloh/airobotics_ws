"""LAB 1-4: 실제 팔의 각도를 읽어 MuJoCo viewer 창 속 SO-101을 똑같이 움직인다 (디지털 트윈, 화면 필요).
--follower 를 주면 leader로 follower를 조종하고, 창에는 follower가 실제로 측정한 자세를 그린다.

python twin.py --scene .../scene.xml --leader-port /dev/ttyACM1 --leader-id leader_g1
python twin.py --scene .../scene.xml --leader-port /dev/ttyACM1 --leader-id leader_g1 \
               --follower-port /dev/ttyACM0 --follower-id follower_g1
"""
import argparse, time
import numpy as np
import mujoco
import mujoco.viewer
from common import JOINTS, FPS, real_to_sim

p = argparse.ArgumentParser()
p.add_argument("--scene", required=True)
p.add_argument("--leader-port", default="/dev/ttyACM1")
p.add_argument("--leader-id", default="leader_g1")
p.add_argument("--follower-port", default=None)
p.add_argument("--follower-id", default="follower_g1")
args = p.parse_args()

from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

leader = SO101Leader(SO101LeaderConfig(port=args.leader_port, id=args.leader_id))
leader.connect()
follower = None
if args.follower_port:
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
    follower = SO101Follower(SO101FollowerConfig(port=args.follower_port, id=args.follower_id, max_relative_target=10.0))
    follower.connect()

m = mujoco.MjModel.from_xml_path(args.scene)
d = mujoco.MjData(m)
d.qpos[:] = real_to_sim(leader.get_action())
mujoco.mj_forward(m, d)

k = 0
try:
    with mujoco.viewer.launch_passive(m, d) as viewer:
        while viewer.is_running():
            t0 = time.perf_counter()
            action = leader.get_action()          # {"shoulder_pan.pos": 도, ...}
            real = action
            if follower:
                follower.send_action(action)      # 실제 팔에 보내고
                obs = follower.get_observation()  # 실제 팔이 지금 있는 각도를 읽는다
                real = {f"{j}.pos": obs[f"{j}.pos"] for j in JOINTS}
            with viewer.lock():
                d.qpos[:] = real_to_sim(real)     # 실제 각도를 MuJoCo 관절에 그대로 (물리 계산 없이)
                mujoco.mj_forward(m, d)
            viewer.sync()
            if k % FPS == 0:                      # 1초마다 값 출력 (방향·오프셋 확인용)
                print(" ".join(f"{j[:8]}={real[f'{j}.pos']:7.1f}" for j in JOINTS))
            k += 1
            time.sleep(max(0.0, 1.0 / FPS - (time.perf_counter() - t0)))
finally:
    leader.disconnect()
    if follower:
        input("follower 팔을 받친 뒤 Enter (torque 꺼짐) ...")
        follower.disconnect()
