# SO-101 실습 코드 (Part 1: MuJoCo sim-to-real)

Jetson의 `~/so101_lab/sim` 에 풀고, LeRobot container 안(`/root/so101_lab/sim`)에서 실행합니다.

## A 방식: Jetson에 모니터 직접 연결 (바탕화면 터미널)

```bash
export SCENE=/root/so101_lab/SO-ARM100/Simulation/SO101/scene.xml
python -m mujoco.viewer --mjcf=$SCENE                          # LAB 1-1 모델 보기
python fk_check.py --scene $SCENE 0 -60 60 60 0 0              # LAB 1-1 FK
python sim_run.py  --scene $SCENE --seg-time 3.0 --out sim_log.csv --viewer   # LAB 1-2
python compare.py  --sim sim_log.csv --png sim_only.png
python real_run.py --step --port /dev/ttyACM0 --id follower_g1              # LAB 1-3 (처음엔 이것부터)
python real_run.py --port /dev/ttyACM0 --id follower_g1 --seg-time 3.0
python compare.py  --sim sim_log.csv --real real_log.csv
python twin.py --scene $SCENE --leader-port /dev/ttyACM1 --leader-id leader_g1 \
  --follower-port /dev/ttyACM0 --follower-id follower_g1                    # LAB 1-4
```

## B 방식: PC에서 ssh 접속 (모니터 없음) → twin_web.py + 브라우저 http://<Jetson IP>:8000

```bash
export SCENE=/root/so101_lab/SO-ARM100/Simulation/SO101/scene.xml
export MUJOCO_GL=egl
python twin_web.py --scene $SCENE --pose 0 0 0 0 0 0           # LAB 1-1 자세 보기 (slider)
python fk_check.py --scene $SCENE 0 -60 60 60 0 0              # LAB 1-1 FK (화면 불필요)
python twin_web.py --scene $SCENE --traj --seg-time 3.0        # LAB 1-2 시뮬레이션 (페이지의 시작 버튼)
python real_run.py --step --port /dev/ttyACM0 --id follower_g1 # LAB 1-3 경로점 하나씩
python twin_web.py --scene $SCENE --traj --seg-time 3.0 \
  --follower-port /dev/ttyACM0 --follower-id follower_g1      # LAB 1-3 sim+real 동시 실행·겹쳐 보기·비교
python twin_web.py --scene $SCENE \
  --leader-port /dev/ttyACM1 --leader-id leader_g1 \
  --follower-port /dev/ttyACM0 --follower-id follower_g1      # LAB 1-4 디지털 트윈
python twin_web.py --scene $SCENE --traj --fake                # 팔 없이 시험 (가상 follower)
```

twin_web.py 모드: `--pose` → pose, `--traj` → sim, `--traj` + `--follower-port`(또는 `--fake`) → s2r, 그 외 → twin.
s2r/sim 모드는 끝나면 `sim_log.csv`(·`real_log.csv`)와 `compare.png`를 `--out-dir`(기본 현재 폴더)에 저장하고 페이지 아래에 그래프·표를 보여 줍니다.
접속이 안 되면 Jetson에서 `sudo ufw allow 8000/tcp`, 또는 PC에서 `ssh -L 8000:localhost:8000 lerobot@<Jetson IP>` 후 http://localhost:8000.

gap 실험: `python sim_run.py --scene $SCENE --kp-scale 0.05 --friction-scale 5 --delay 3 --out other_log.csv`

| 파일 | 내용 |
|---|---|
| common.py | 관절 이름, LeRobot(도)↔MuJoCo(rad) 변환, SIGN·OFFSET_DEG, 경로점, 3차 다항식 보간 |
| fk_check.py | 관절각 → gripper 끝 위치·4×4 변환 행렬, 충돌 수 |
| sim_run.py | 궤적을 MuJoCo에서 실행해 sim_log.csv 기록 |
| real_run.py | 같은 궤적을 SO101Follower로 실행해 real_log.csv 기록 (max_relative_target=10) |
| compare.py | 관절별 그래프, RMS 오차, sim-real 차이, 지연 추정 |
| waypoints_ref.png | 경로점 9개를 MuJoCo로 그린 참고 그림 (--step 때 실제 팔과 비교) |
| twin_web.py | B 방식(ssh)의 Part 1 전체: MuJoCo 렌더(EGL) → 브라우저 :8000. 모드 pose / sim / s2r(실제 팔+MuJoCo 겹쳐 보기, 비교 그래프) / twin. --fake로 팔 없이 시험 |
| twin.py | leader → MuJoCo 디지털 트윈 (옵션: follower 동시 구동) |

확인 상태 (2026-11-11):
- fk_check / sim_run / compare: MuJoCo 4.0에서 실행 확인 (슬라이드의 숫자)
- twin_web.py twin 모드: 조교 Jetson(Orin NX, LeRobot 0.7.0 image, MuJoCo 3.8.1)에서 leader·follower 연결, PC 브라우저로 확인
- MuJoCo viewer 창: Jetson 모니터에 뜨는 것 확인 (container를 DISPLAY=:0 + xhost 허용으로 만든 경우)
- twin_web.py pose / sim / s2r 모드: --fake(가상 팔)로 확인, 실제 팔 미실행
- real_run.py(--step 포함): 가짜 로봇으로 흐름만 확인 / twin.py: 실제 팔 미실행
- common.py의 SIGN·OFFSET_DEG, 경로점 높이는 LAB 1-3(--step)·1-4에서 실제 팔로 확인

주의: 모든 코드는 container 안(root@…)에서 실행. Jetson 본체에는 mujoco가 없음.
브라우저가 시간 초과면 Jetson에서 sudo ufw allow 8000/tcp (또는 PC에서 ssh -L 8000:localhost:8000 …).

MuJoCo 모델: https://github.com/TheRobotStudio/SO-ARM100 (Simulation/SO101/scene.xml, so101_new_calib.xml)
