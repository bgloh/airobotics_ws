"""LAB 1-3: 시뮬레이션과 실제 로봇의 기록을 한 그래프에 겹쳐 그리고 차이를 숫자로 본다.

python compare.py --sim sim_log.csv --real real_log.csv --png compare.png
"""
import argparse
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from common import JOINTS, FPS

p = argparse.ArgumentParser()
p.add_argument("--sim", default="sim_log.csv")
p.add_argument("--real", default=None)
p.add_argument("--png", default="compare.png")
args = p.parse_args()


def load(path):
    a = np.genfromtxt(path, delimiter=",", names=True)
    return a


sim = load(args.sim)
real = load(args.real) if args.real else None

fig, axes = plt.subplots(2, 3, figsize=(15, 7), sharex=True)
for ax, j in zip(axes.flat, JOINTS):
    ax.plot(sim["t"], sim[f"target_{j}"], "k--", lw=1.5, label="target (command)")
    ax.plot(sim["t"], sim[f"actual_{j}"], color="#1D63ED", lw=2, label="MuJoCo")
    if real is not None:
        ax.plot(real["t"], real[f"actual_{j}"], color="#C2601A", lw=2, label="real SO-101")
    ax.set_title(j)
    ax.grid(alpha=0.3)
axes[0, 0].legend(fontsize=9)
for ax in axes[1]:
    ax.set_xlabel("time (s)")
axes[0, 0].set_ylabel("deg (gripper: 0-100)")
axes[1, 0].set_ylabel("deg (gripper: 0-100)")
fig.tight_layout()
fig.savefig(args.png, dpi=120)
print("그래프 저장:", args.png)


def lag_frames(cmd, act, max_lag=15):
    """명령을 몇 프레임 늦게 따라가는지: 명령을 k프레임 밀었을 때 오차가 가장 작은 k."""
    errs = [np.mean((act[k:] - cmd[: len(cmd) - k]) ** 2) for k in range(max_lag)]
    return int(np.argmin(errs))


print(f"{'joint':14s} {'sim RMS':>8s} {'real RMS':>9s} {'sim-real':>9s} {'real lag':>9s}")
for j in JOINTS[:5]:
    s_rms = np.sqrt(np.mean((sim[f"actual_{j}"] - sim[f"target_{j}"]) ** 2))
    line = f"{j:14s} {s_rms:8.2f}"
    if real is not None:
        n = min(len(sim), len(real))
        r_rms = np.sqrt(np.mean((real[f"actual_{j}"][:n] - real[f"target_{j}"][:n]) ** 2))
        gap = np.sqrt(np.mean((real[f"actual_{j}"][:n] - sim[f"actual_{j}"][:n]) ** 2))
        lag = lag_frames(real[f"target_{j}"][:n], real[f"actual_{j}"][:n])
        line += f" {r_rms:9.2f} {gap:9.2f} {lag / FPS * 1000:7.0f}ms"
    print(line)
