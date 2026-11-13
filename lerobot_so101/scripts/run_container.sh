#!/usr/bin/env bash
# LeRobot container 실행 (Jetson 본체 터미널 H 에서 실행)
#   bash run_container.sh a   # A 방식: Jetson 모니터 직접 연결 (MuJoCo viewer·Rerun 창을 Jetson 화면에)
#   bash run_container.sh b   # B 방식: PC에서 ssh 접속 (화면 없음, MuJoCo는 twin_web.py로 브라우저)
# container가 이미 있으면 다시 만들지 않고 그대로 들어갑니다(docker start -ai).
set -e
MODE=${1:-b}
NAME=${NAME:-lerobot}
IMAGE=${IMAGE:-ghcr.io/ravediamond/lerobot-jetson:0.7.0-jp6.2}
mkdir -p ~/so101_lab ~/.cache/huggingface

if [ "$MODE" = "a" ]; then
  # Jetson 화면(X 서버)이 쓰는 인증 파일을 찾아 container의 창 표시를 허용 (재부팅 후 다시 필요)
  AUTH=$(ps -eo args | grep -m1 -oP '(?<=-auth )\S+' || true)
  if [ -n "$AUTH" ]; then
    sudo DISPLAY=:0 XAUTHORITY="$AUTH" xhost +local: || echo "[경고] xhost 실패: Jetson 화면이 켜져 있는지 확인 (ls /tmp/.X11-unix 에 X0)"
  else
    echo "[경고] X 서버를 찾지 못함. Jetson에 모니터를 연결하고 로그인했는지 확인"
  fi
fi

if docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
  echo "기존 container '$NAME' 에 들어갑니다. (다른 터미널: docker exec -it $NAME bash)"
  exec docker start -ai "$NAME"
fi

# A·B 같은 옵션으로 만듭니다. DISPLAY 옵션은 B 방식에서 쓰이지 않을 뿐 해가 없고,
# 나중에 A 방식으로 바꿔도 container를 다시 만들 필요가 없습니다.
exec docker run -it --name "$NAME" --runtime nvidia \
  --network host --ipc=host --privileged -v /dev:/dev \
  -e DISPLAY=:0 -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/so101_lab:/root/so101_lab \
  -v ~/.cache/huggingface:/root/.cache/huggingface \
  "$IMAGE"
