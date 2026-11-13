"""LAB 1-1: 관절각을 넣으면 MuJoCo가 계산한 gripper 끝 위치(= Forward Kinematics)를 출력한다.
수업에서 배운 DH 변환을 곱한 결과와 같은 질문에 대한 답이다: '관절이 이 각도면 손끝은 어디?'

python fk_check.py --scene .../scene.xml 0 -60 60 60 0 0
"""
import argparse
import numpy as np
import mujoco
from common import JOINTS, real_to_sim, row_to_action

p = argparse.ArgumentParser()
p.add_argument("--scene", required=True)
p.add_argument("deg", nargs=6, type=float, help="pan lift elbow wrist_flex wrist_roll gripper(0~100)")
args = p.parse_args()

m = mujoco.MjModel.from_xml_path(args.scene)
d = mujoco.MjData(m)
d.qpos[:] = real_to_sim(row_to_action(args.deg))
mujoco.mj_forward(m, d)                      # 관절각 → 모든 링크의 위치·자세 계산
pos = d.site("gripperframe").xpos
R = d.site("gripperframe").xmat.reshape(3, 3)
T = np.eye(4); T[:3, :3] = R; T[:3, 3] = pos
np.set_printoptions(precision=3, suppress=True)
print("gripper 끝 위치 (m):", pos)
print("base → gripper 4x4 변환 행렬 T:\n", T)
print("충돌(접촉) 수:", d.ncon, "(0이 아니면 바닥이나 자기 몸에 닿은 자세)")
