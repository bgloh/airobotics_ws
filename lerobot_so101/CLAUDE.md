# lerobot_so101 — 11주차 LeRobot SO-ARM101 실습

이 폴더는 Jetson Orin NX + SO-ARM101 실습 자료입니다. 수정할 때 지킬 것:

- 실행 환경은 Jetson의 Docker container(`ghcr.io/ravediamond/lerobot-jetson:0.7.0-jp6.2`, Python 3.12, LeRobot 0.7.0, mujoco 3.8.1) 안입니다. Jetson 본체에는 mujoco가 없습니다.
- 실습 방식은 두 가지이며 문서·코드 모두 둘 다 지원해야 합니다.
  - A: Jetson 모니터 직접 연결 → MuJoCo viewer 창, Rerun 창
  - B: PC에서 ssh → `sim/twin_web.py`(MUJOCO_GL=egl, HTTP :8000)로 MuJoCo를 브라우저에, `--display_data=false`, 녹화 키 n/r/q
- `sim/common.py`가 단일 기준입니다(관절 이름, 도↔라디안 변환, SIGN·OFFSET_DEG, 경로점, 3차 보간). 다른 파일에 값을 복사하지 말고 import 합니다.
- `twin_web.py`는 모드 4개(pose / sim / s2r / twin)와 `--fake`(가상 leader·follower)를 가집니다. 고친 뒤에는 실제 팔 없이 `--fake`로 네 모드를 모두 돌려 보고, `/status`·`/stream`·`/compare.png`가 응답하는지 확인합니다.
- 실제 팔로 보내는 코드는 안전장치를 유지합니다: `max_relative_target=10`, 시작 전 W0으로 3초 천천히 이동, torque를 끄기 전 Enter 대기.
- 슬라이드나 README에 숫자를 쓸 때는 실제로 실행한 값만 쓰고, 확인하지 않은 것은 "미확인"으로 표시합니다(README 맨 아래 "확인 상태").
- 문서는 한국어, 명령·옵션·파일 이름은 원문 그대로 씁니다.
