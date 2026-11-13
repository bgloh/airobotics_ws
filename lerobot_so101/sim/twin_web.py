"""SO-101 MuJoCo 웹 보기 — 모니터 없이 ssh로 접속한 경우(B 방식)에 Part 1 전체를 브라우저로 한다.
Jetson이 MuJoCo 화면을 그려(EGL, 화면 불필요) 웹으로 보내고, PC 브라우저에서 http://<Jetson IP>:8000 을 연다.

모드 (container 안, sim 폴더에서 실행. 모든 명령 앞에 MUJOCO_GL=egl)
  LAB 1-1  모델 보기:      python twin_web.py --scene $SCENE --pose 0 -60 60 60 0 0
                           (브라우저의 slider로 관절을 움직이며 손끝 위치·충돌 확인 = viewer의 Control 패널)
  LAB 1-2  시뮬레이션 궤적: python twin_web.py --scene $SCENE --traj --seg-time 3.0
  LAB 1-3  sim → real:     python twin_web.py --scene $SCENE --traj --seg-time 3.0 \
                               --follower-port /dev/ttyACM0 --follower-id follower_g1
                           (같은 명령을 MuJoCo와 실제 팔에 동시에 보내고, 노란 팔 = 시뮬레이션,
                            파란 반투명 팔 = 실제 팔 측정값. 끝나면 비교 그래프가 브라우저에 나옴)
  LAB 1-4  real → sim:     python twin_web.py --scene $SCENE \
                               --leader-port /dev/ttyACM1 --leader-id leader_g1 \
                               --follower-port /dev/ttyACM0 --follower-id follower_g1
  팔 없이 시험: 위 명령에 --fake (실제 팔 대신 늦게 따라오는 가짜 팔)

종료: Ctrl+C. 실제 follower를 쓴 경우 팔을 손으로 받친 뒤 Enter를 누르면 torque가 꺼진다.
"""
import argparse, csv, io, json, os, subprocess, sys, threading, time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs
import numpy as np
import mujoco
from PIL import Image, ImageDraw
from common import JOINTS, FPS, SEGMENT_TIME_S, make_trajectory, real_to_sim, sim_to_real, row_to_action

HERE = os.path.dirname(os.path.abspath(__file__))

p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
p.add_argument("--scene", required=True, help="SO-ARM100/Simulation/SO101/scene.xml 경로")
p.add_argument("--pose", nargs=6, type=float, metavar="DEG", help="모델 보기: 시작 자세 (pan lift elbow wrist_flex roll gripper)")
p.add_argument("--traj", action="store_true", help="common.py의 경로점 궤적 실행 (follower를 주면 실제 팔도 함께)")
p.add_argument("--seg-time", type=float, default=SEGMENT_TIME_S, help="경로점 사이 이동 시간(초)")
p.add_argument("--out-dir", default=".", help="궤적 모드에서 sim_log.csv, real_log.csv, compare.png 저장 위치")
p.add_argument("--kp-scale", type=float, default=1.0, help="(gap 실험) MuJoCo 위치 gain 배율")
p.add_argument("--friction-scale", type=float, default=1.0, help="(gap 실험) MuJoCo 관절 마찰 배율")
p.add_argument("--delay", type=int, default=0, help="(gap 실험) MuJoCo에 명령이 늦게 도착하는 프레임 수")
p.add_argument("--leader-port", default=None)
p.add_argument("--leader-id", default="leader_g1")
p.add_argument("--follower-port", default=None)
p.add_argument("--follower-id", default="follower_g1")
p.add_argument("--fake", action="store_true", help="실제 팔 없이 시험")
p.add_argument("--http-port", type=int, default=8000)
p.add_argument("--width", type=int, default=960)
p.add_argument("--height", type=int, default=640)
p.add_argument("--render-fps", type=float, default=None, help="브라우저 화면 갱신 (기본: 궤적 15, 그 외 20)")
args = p.parse_args()

if args.pose:
    MODE = "pose"
elif args.traj:
    MODE = "s2r" if (args.follower_port or args.fake) else "sim"
else:
    MODE = "twin"
    if not (args.leader_port or args.fake):
        p.error("모드를 고르세요: --pose, --traj, 또는 --leader-port (디지털 트윈). 팔 없이 시험은 --fake")
RENDER_FPS = args.render_fps or (15.0 if MODE in ("sim", "s2r") else 20.0)
TITLE = {"pose": "LAB 1-1 모델 보기", "sim": "LAB 1-2 시뮬레이션 궤적",
         "s2r": "LAB 1-3 sim → real (노랑 = MuJoCo, 파랑 = 실제 팔)", "twin": "LAB 1-4 디지털 트윈 (real → sim)"}[MODE]

# ── 공유 상태 ───────────────────────────────────────────────
lock = threading.Lock()
zero = {f"{j}.pos": 0.0 for j in JOINTS}
state = {
    "main": dict(row_to_action(args.pose) if args.pose else zero),  # 노란 팔(도)
    "main_rad": None,      # 궤적 모드: MuJoCo 물리 계산 결과(rad)를 그대로
    "ghost": None,         # 파란 반투명 팔(도) = 실제 팔 측정값
    "cmd": None,           # 명령(도)
    "hz": 0.0, "msg": "", "progress": "",
    "result": "", "has_compare": False, "compare_ver": 0,
}
view = {"az": 135.0, "el": -20.0, "dist": 1.0}
stop = threading.Event()
go_evt = threading.Event()      # 터미널 Enter 또는 브라우저 '시작' 버튼
stdin_ok = {"v": sys.stdin is not None and sys.stdin.isatty()}
frame = {"jpg": None}


def stdin_reader():
    while not stop.is_set():
        try:
            line = sys.stdin.readline()
        except Exception:
            line = ""
        if line == "":              # 입력이 닫힘 (터미널 없이 실행)
            stdin_ok["v"] = False
            return
        go_evt.set()


def set_msg(s):
    with lock:
        state["msg"] = s
    print(s, flush=True)


# ── 가짜 팔 (--fake): 명령을 늦고 무디게 따라간다 ─────────────────
class FakeFollower:
    def __init__(self):
        self.q = np.array(make_trajectory(seg_t=args.seg_time)[0]) if MODE == "s2r" else np.zeros(6)
        self.buf = []

    def send_action(self, a):
        self.buf.append(np.array([a[f"{j}.pos"] for j in JOINTS]))
        if len(self.buf) <= 3:                       # 3프레임(0.1초) 늦게 도착
            return
        target = self.buf.pop(0)
        self.q = self.q + 0.25 * (target - self.q)   # 1차 지연(물렁한 servo)
        self.q[1] += 0.15                            # 중력 때문에 조금 덜 올라감

    def get_observation(self):
        return {f"{j}.pos": float(v) for j, v in zip(JOINTS, self.q)}

    def disconnect(self):
        pass


class FakeLeader:
    def __init__(self):
        self.t0 = time.time()

    def get_action(self):
        t = time.time() - self.t0
        a = {f"{j}.pos": 30 * np.sin(0.8 * t + i) for i, j in enumerate(JOINTS)}
        a["gripper.pos"] = 50 + 40 * np.sin(1.5 * t)
        return a

    def disconnect(self):
        pass


def connect_follower():
    if args.fake:
        return FakeFollower()
    from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
    f = SO101Follower(SO101FollowerConfig(port=args.follower_port, id=args.follower_id, max_relative_target=10.0))
    f.connect()
    return f


def connect_leader():
    if args.fake:
        return FakeLeader()
    from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
    l = SO101Leader(SO101LeaderConfig(port=args.leader_port, id=args.leader_id))
    l.connect()
    return l


def read_deg(robot):
    obs = robot.get_observation()
    return {f"{j}.pos": float(obs[f"{j}.pos"]) for j in JOINTS}


def pace(t0, k):
    time.sleep(max(0.0, t0 + (k + 1) / FPS - time.perf_counter()))


def wait_start(prompt):
    """터미널 Enter 또는 브라우저 '시작' 버튼을 기다린다."""
    go_evt.clear()
    set_msg(prompt)
    while not go_evt.wait(0.2):
        if stop.is_set():
            raise KeyboardInterrupt


def release(follower):
    """torque를 끄기 전에 팔을 받칠 시간을 준다."""
    if follower is None:
        return
    if stdin_ok["v"]:
        go_evt.clear()
        print("\nfollower 팔을 손으로 받친 뒤 Enter를 누르면 torque를 끕니다 ...", flush=True)
        try:
            go_evt.wait()
        except KeyboardInterrupt:
            pass
    follower.disconnect()


# ── 모드별 제어 루프 (메인 스레드) ──────────────────────────────
def run_pose():
    set_msg("브라우저의 slider로 관절을 움직여 보세요. 종료: Ctrl+C")
    while not stop.wait(0.5):
        pass


def run_twin():
    leader = connect_leader()
    follower = connect_follower() if args.follower_port else None
    set_msg("leader를 손으로 움직이세요" + (" (follower가 따라가고, 화면은 follower 측정값)" if follower else "") + ". 종료: Ctrl+C")
    t_prev = time.perf_counter(); k = 0; t0 = t_prev
    try:
        while not stop.is_set():
            cmd = leader.get_action()
            real = cmd
            if follower:
                follower.send_action(cmd)
                real = read_deg(follower)
            now = time.perf_counter()
            with lock:
                state["main"] = {j: float(v) for j, v in real.items()}
                state["cmd"] = {j: float(v) for j, v in cmd.items()} if follower else None
                state["hz"] = 0.9 * state["hz"] + 0.1 / max(1e-6, now - t_prev)
            t_prev = now
            pace(t0, k); k += 1
    finally:
        leader.disconnect()
        release(follower)


def run_traj():
    """궤적을 MuJoCo(물리)에서 실행하고, follower가 있으면 같은 명령을 실제 팔에도 보낸다."""
    traj = make_trajectory(seg_t=args.seg_time)
    m = mujoco.MjModel.from_xml_path(args.scene)
    m.actuator_gainprm[:, 0] *= args.kp_scale
    m.actuator_biasprm[:, 1] *= args.kp_scale
    m.dof_frictionloss[:] *= args.friction_scale
    d = mujoco.MjData(m)
    substeps = int(round(1.0 / (FPS * m.opt.timestep)))
    follower = connect_follower() if MODE == "s2r" else None
    os.makedirs(args.out_dir, exist_ok=True)
    try:
        while not stop.is_set():
            # 시작 자세 맞추기
            d.qpos[:] = real_to_sim(row_to_action(traj[0])); d.qvel[:] = 0; d.ctrl[:] = d.qpos[:]
            mujoco.mj_forward(m, d)
            with lock:
                state["main_rad"] = d.qpos.copy(); state["main"] = row_to_action(traj[0]); state["progress"] = ""
            if follower:
                set_msg("실제 팔이 3초 동안 시작 자세(W0)로 이동합니다 ...")
                now = np.array([read_deg(follower)[f"{j}.pos"] for j in JOINTS])
                t0 = time.perf_counter()
                for k, s in enumerate(np.linspace(0, 1, 3 * FPS)):
                    follower.send_action(row_to_action(now + (traj[0] - now) * s))
                    with lock:
                        state["ghost"] = read_deg(follower)
                    pace(t0, k)
            wait_start(f"준비 완료 ({len(traj)}개 명령, {len(traj) / FPS:.0f}초). "
                       + ("작업대를 비우고 정지 담당을 정한 뒤 " if follower else "")
                       + "터미널에서 Enter 또는 브라우저 '시작' 버튼")
            set_msg("실행 중 ...")
            sim_rows, real_rows, max_con = [], [], 0
            t0 = time.perf_counter(); t_prev = t0
            for k, target in enumerate(traj):
                if stop.is_set():
                    break
                cmd = row_to_action(target)
                if follower:
                    follower.send_action(cmd)                                   # ① 실제 팔에 명령
                d.ctrl[:] = real_to_sim(row_to_action(traj[max(0, k - args.delay)]))  # ① MuJoCo에 같은 명령
                for _ in range(substeps):                                       # ② 물리 1/30초
                    mujoco.mj_step(m, d)
                sim_deg = sim_to_real(d.qpos)                                   # ③ 시뮬레이션 자세
                tip = d.site("gripperframe").xpos.copy()
                max_con = max(max_con, d.ncon)
                sim_rows.append([k / FPS, *target, *[sim_deg[f"{j}.pos"] for j in JOINTS], *tip, d.ncon])
                real = None
                if follower:
                    real = read_deg(follower)                                   # ③ 실제 팔 자세
                    real_rows.append([k / FPS, *target, *[real[f"{j}.pos"] for j in JOINTS]])
                now = time.perf_counter()
                with lock:
                    state["main_rad"] = d.qpos.copy(); state["main"] = sim_deg
                    state["ghost"] = real; state["cmd"] = cmd
                    state["progress"] = f"{k + 1}/{len(traj)}"
                    state["hz"] = 0.9 * state["hz"] + 0.1 / max(1e-6, now - t_prev)
                t_prev = now
                pace(t0, k)
            if stop.is_set():
                break
            save_and_compare(sim_rows, real_rows, max_con)
            wait_start("다시 하려면 터미널 Enter 또는 '시작' 버튼, 끝내려면 Ctrl+C")
    finally:
        release(follower)


def save_and_compare(sim_rows, real_rows, max_con):
    head_s = ["t"] + [f"target_{j}" for j in JOINTS] + [f"actual_{j}" for j in JOINTS] + ["tip_x", "tip_y", "tip_z", "contacts"]
    head_r = ["t"] + [f"target_{j}" for j in JOINTS] + [f"actual_{j}" for j in JOINTS]
    sim_csv = os.path.join(args.out_dir, "sim_log.csv")
    with open(sim_csv, "w", newline="") as f:
        csv.writer(f).writerows([head_s] + sim_rows)
    cmd = [sys.executable, os.path.join(HERE, "compare.py"), "--sim", sim_csv, "--png", os.path.join(args.out_dir, "compare.png")]
    if real_rows:
        real_csv = os.path.join(args.out_dir, "real_log.csv")
        with open(real_csv, "w", newline="") as f:
            csv.writer(f).writerows([head_r] + real_rows)
        cmd += ["--real", real_csv]
    env = dict(os.environ, MPLBACKEND="Agg")
    r = subprocess.run(cmd, capture_output=True, text=True, env=env)
    a = np.array(sim_rows)
    text = (f"저장: {sim_csv}" + (f", {os.path.join(args.out_dir, 'real_log.csv')}" if real_rows else "")
            + f"\ngripper 끝 최저 높이(시뮬레이션): {a[:, 15].min() * 100:.1f} cm, 최대 접촉 수: {max_con}\n\n"
            + "관절별 RMS 오차(도), lag = 실제 팔이 명령보다 늦은 시간\n" + r.stdout + r.stderr)
    with lock:
        state["result"] = text; state["has_compare"] = r.returncode == 0; state["compare_ver"] += 1
    print(text, flush=True)


# ── 렌더 스레드: 노란 팔(+ 파란 반투명 팔)을 그려 JPEG로 ─────────────────
def render_loop():
    m = mujoco.MjModel.from_xml_path(args.scene)
    m.vis.global_.offwidth = max(m.vis.global_.offwidth, args.width)
    m.vis.global_.offheight = max(m.vis.global_.offheight, args.height)
    d = mujoco.MjData(m)
    dg = mujoco.MjData(m)
    r = mujoco.Renderer(m, args.height, args.width)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [0.12, 0.0, 0.18]
    blue = np.array([29, 99, 237], float)
    while not stop.is_set():
        t0 = time.perf_counter()
        with lock:
            main = dict(state["main"]); main_rad = None if state["main_rad"] is None else state["main_rad"].copy()
            ghost = None if state["ghost"] is None else dict(state["ghost"])
            cmd = None if state["cmd"] is None else dict(state["cmd"])
            hz, msg, prog = state["hz"], state["msg"], state["progress"]
        d.qpos[:] = main_rad if main_rad is not None else real_to_sim(main)
        mujoco.mj_forward(m, d)
        cam.azimuth, cam.elevation, cam.distance = view["az"], view["el"], view["dist"]
        r.update_scene(d, cam)
        img = r.render().astype(float)
        if ghost is not None:                                   # 실제 팔 자세를 파란 반투명으로 겹치기
            dg.qpos[:] = real_to_sim(ghost)
            mujoco.mj_forward(m, dg)
            r.update_scene(dg, cam)
            g = r.render().astype(float)
            r.enable_segmentation_rendering()
            r.update_scene(dg, cam)
            seg = r.render()
            r.disable_segmentation_rendering()
            gid = seg[:, :, 0]
            ok = (gid >= 0) & (seg[:, :, 1] == int(mujoco.mjtObj.mjOBJ_GEOM))
            body = np.full(gid.shape, 0)
            body[ok] = m.geom_bodyid[gid[ok]]
            mask = body > 0
            img[mask] = 0.4 * img[mask] + 0.6 * (0.3 * g[mask] + 0.7 * blue)
        im = Image.fromarray(img.clip(0, 255).astype(np.uint8))
        dr = ImageDraw.Draw(im)
        lines = ["pose (slider)" if MODE == "pose" else f"{MODE}  {hz:4.1f} Hz  {prog}",
                 f"gripper tip z = {d.site('gripperframe').xpos[2] * 100:5.1f} cm   contacts = {d.ncon}"]
        if ghost is not None:
            lines.append(f"{'joint':13s} {'sim':>6s} {'real':>6s} {'real-sim':>8s}")
            for j in JOINTS:
                a, b = main[f"{j}.pos"], ghost[f"{j}.pos"]
                lines.append(f"{j:13s} {a:6.1f} {b:6.1f} {b - a:8.1f}")
        else:
            for j in JOINTS:
                s = f"{j:13s} {main[f'{j}.pos']:7.1f}"
                if cmd and MODE == "twin":
                    s += f"   cmd {cmd[f'{j}.pos']:7.1f}   diff {main[f'{j}.pos'] - cmd[f'{j}.pos']:6.1f}"
                lines.append(s)
        w = 300 if (ghost is None and not (cmd and MODE == "twin")) else 470
        dr.rectangle([8, 8, w, 16 + 15 * len(lines)], fill=(15, 27, 45))
        for i, s in enumerate(lines):
            dr.text((14, 12 + 15 * i), s, fill=(230, 237, 246))
        buf = io.BytesIO()
        im.save(buf, format="JPEG", quality=80)
        frame["jpg"] = buf.getvalue()
        time.sleep(max(0.0, 1.0 / RENDER_FPS - (time.perf_counter() - t0)))
    r.close()


# ── 웹 페이지 ───────────────────────────────────────────────
# 관절 slider 범위 = MJCF 관절 range를 LeRobot 단위(도, gripper 0–100)로 바꾼 값.
# 범위 밖 값(예: gripper 음수)을 넣으면 두 집게가 서로 통과하는 불가능한 자세가 되므로 막는다.
def _slider_ranges():
    mm = mujoco.MjModel.from_xml_path(args.scene)
    lo, hi = np.zeros(len(JOINTS)), np.zeros(len(JOINTS))
    for i, j in enumerate(JOINTS):
        lo[i], hi[i] = mm.jnt_range[mm.joint(j).id]
    a, b = sim_to_real(lo), sim_to_real(hi)
    return {j: (int(round(min(a[f"{j}.pos"], b[f"{j}.pos"]))), int(round(max(a[f"{j}.pos"], b[f"{j}.pos"]))))
            for j in JOINTS}
SL_RANGE = _slider_ranges()
SLIDERS = "".join(
    f'<label>{j}<input type="range" min="{SL_RANGE[j][0]}" max="{SL_RANGE[j][1]}" step="1" '
    f'value="{int(round(state["main"][f"{j}.pos"]))}" '
    f'oninput="fetch(\'/set?j={j}&v=\'+this.value);this.nextElementSibling.textContent=this.value">'
    f'<span>{int(round(state["main"][f"{j}.pos"]))}</span></label>' for j in JOINTS)
PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>SO-101 MuJoCo</title>
<style>body{margin:0;background:#13233A;color:#F6F7F4;font-family:sans-serif;text-align:center}
img{max-width:100%;border-radius:8px}button{font-size:17px;margin:6px 3px;padding:6px 12px}
#msg{font-size:18px;margin:8px;color:#F2A65A}#sl label{display:inline-block;margin:6px 10px;font-size:15px}
#sl input{width:160px;vertical-align:middle}pre{text-align:left;display:inline-block;background:#0F1B2D;padding:12px;border-radius:8px}</style></head>
<body><h2>__TITLE__</h2><div id="msg"></div><img src="/stream"><div>
<button onclick="fetch('/view?az=135&el=-20')">비스듬히</button><button onclick="fetch('/view?az=90&el=0')">옆</button>
<button onclick="fetch('/view?az=180&el=-40')">정면 위</button><button onclick="fetch('/view?az=90&el=-89')">위</button>
<button onclick="fetch('/view?zoom=0.9')">확대</button><button onclick="fetch('/view?zoom=1.1')">축소</button>
__START__</div><div id="sl">__SLIDERS__</div><div id="res"></div>
<script>
let ver=-1;
async function poll(){try{const s=await (await fetch('/status')).json();
document.getElementById('msg').textContent=s.msg;
if(s.compare_ver!=ver){ver=s.compare_ver;if(s.result){document.getElementById('res').innerHTML=
(s.has_compare?'<h3>비교 결과</h3><img src="/compare.png?v='+ver+'"><br>':'')+'<pre></pre>';
document.querySelector('#res pre').textContent=s.result;}}
if(s.pose){document.querySelectorAll('#sl input').forEach((e,i)=>{if(document.activeElement!==e){e.value=s.pose[i];e.nextElementSibling.textContent=Math.round(s.pose[i]);}});}
}catch(e){} setTimeout(poll,1000);} poll();
</script></body></html>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, body, ctype, code=200):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path); q = parse_qs(u.query)
        if u.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while not stop.is_set():
                    jpg = frame["jpg"]
                    if jpg:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                         + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                    time.sleep(1.0 / RENDER_FPS)
            except (BrokenPipeError, ConnectionResetError):
                pass
        elif u.path == "/frame.jpg":
            self._send(frame["jpg"] or b"", "image/jpeg")
        elif u.path == "/view":
            if "az" in q: view["az"] = float(q["az"][0])
            if "el" in q: view["el"] = float(q["el"][0])
            if "zoom" in q: view["dist"] = min(2.5, max(0.3, view["dist"] * float(q["zoom"][0])))
            self._send(b"", "text/plain", 204)
        elif u.path == "/set" and MODE == "pose":
            j, v = q.get("j", [""])[0], q.get("v", ["0"])[0]
            if j in JOINTS:
                with lock:
                    lo, hi = SL_RANGE[j]
                    state["main"][f"{j}.pos"] = min(hi, max(lo, float(v)))
            self._send(b"", "text/plain", 204)
        elif u.path == "/start":
            go_evt.set()
            self._send(b"", "text/plain", 204)
        elif u.path == "/status":
            with lock:
                s = {"msg": state["msg"], "result": state["result"], "has_compare": state["has_compare"],
                     "compare_ver": state["compare_ver"],
                     "pose": [state["main"][f"{j}.pos"] for j in JOINTS] if MODE == "pose" else None}
            self._send(json.dumps(s).encode(), "application/json")
        elif u.path == "/compare.png":
            path = os.path.join(args.out_dir, "compare.png")
            if os.path.exists(path):
                with open(path, "rb") as f:
                    self._send(f.read(), "image/png")
            else:
                self._send(b"", "text/plain", 404)
        else:
            page = (PAGE.replace("__TITLE__", TITLE)
                    .replace("__START__", '<button onclick="fetch(\'/start\')" style="background:#F2A65A">시작</button>' if MODE in ("sim", "s2r") else "")
                    .replace("__SLIDERS__", SLIDERS if MODE == "pose" else ""))
            self._send(page.encode(), "text/html; charset=utf-8")


threading.Thread(target=render_loop, daemon=True).start()
if stdin_ok["v"]:
    threading.Thread(target=stdin_reader, daemon=True).start()
server = ThreadingHTTPServer(("0.0.0.0", args.http_port), Handler)
threading.Thread(target=server.serve_forever, daemon=True).start()
print(f"[{TITLE}] 브라우저에서 http://<Jetson IP>:{args.http_port} 를 여세요. 종료는 Ctrl+C", flush=True)
try:
    {"pose": run_pose, "twin": run_twin, "sim": run_traj, "s2r": run_traj}[MODE]()
except KeyboardInterrupt:
    pass
finally:
    stop.set()
    server.shutdown()
    print("종료", flush=True)
