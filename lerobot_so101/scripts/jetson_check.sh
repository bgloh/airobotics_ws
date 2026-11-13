#!/usr/bin/env bash
# Jetson + SO-101 실습 환경 점검 (sudo 없이 실행 가능, 결과는 jetson_check.txt)
# 사용: bash jetson_check.sh
out=jetson_check.txt
{
sec(){ echo; echo "===== $1 ====="; }
sec "보드 / OS"
tr -d '\0' < /proc/device-tree/model 2>/dev/null; echo
cat /etc/nv_tegra_release 2>/dev/null | head -1
dpkg-query -W -f='nvidia-jetpack ${Version}\n' nvidia-jetpack 2>/dev/null || echo "nvidia-jetpack 패키지 정보 없음"
grep PRETTY_NAME /etc/os-release; uname -r; uptime
sec "CPU / 메모리 / swap"
nproc; lscpu | grep -E "Model name|CPU max MHz"
free -h
sec "저장 공간 (LeRobot image 약 28 GB + 데이터 필요, 40 GB 이상 여유 권장)"
df -h / /home 2>/dev/null | uniq
lsblk -d -o NAME,SIZE,MODEL 2>/dev/null
sec "전원 모드 / 클럭"
nvpmodel -q 2>/dev/null || echo "nvpmodel 확인하려면: sudo nvpmodel -q"
sec "온도"
for z in /sys/class/thermal/thermal_zone*; do echo "$(cat $z/type 2>/dev/null): $(( $(cat $z/temp 2>/dev/null || echo 0) / 1000 )) C"; done
sec "tegrastats 1회 (RAM, GPU GR3D_FREQ, 온도)"
timeout 3 tegrastats --interval 1000 2>/dev/null | head -1 || echo "tegrastats 없음"
sec "CUDA"
ls -d /usr/local/cuda* 2>/dev/null; /usr/local/cuda/bin/nvcc --version 2>/dev/null | tail -1
sec "Docker"
docker --version 2>/dev/null || echo "docker 없음"
docker info 2>/dev/null | grep -iE "Runtimes|Default Runtime|Docker Root Dir" || echo "docker info 실패 (권한: sudo usermod -aG docker $USER 후 재로그인)"
docker images 2>/dev/null | grep -iE "lerobot|REPOSITORY" || true
docker ps -a 2>/dev/null | grep -iE "lerobot|NAMES" || true
sec "Python"
python3 --version
python3 -c "import torch;print('torch',torch.__version__,'cuda',torch.cuda.is_available())" 2>/dev/null || echo "호스트 python에 torch 없음 (container 사용 시 정상)"
sec "USB 장치 (팔 /dev/ttyACM*, 카메라 /dev/video*)"
ls -l /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "팔(serial) 장치 없음"
ls -l /dev/video* 2>/dev/null || echo "카메라 장치 없음"
lsusb 2>/dev/null
groups
sec "네트워크"
hostname; hostname -I
ping -c 1 -W 2 github.com >/dev/null 2>&1 && echo "인터넷 연결 OK" || echo "인터넷 연결 안 됨 (docker pull, pip 불가)"
sec "화면 (MuJoCo·Rerun 창용)"
echo "DISPLAY=${DISPLAY:-없음 (ssh 접속이면 정상, 창은 Jetson 모니터에서)}"
} > "$out" 2>&1
cat "$out"
echo; echo ">> 결과 저장: $(pwd)/$out"
