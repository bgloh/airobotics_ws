#!/usr/bin/env bash
# 실습 폴더 준비 (Jetson 본체 터미널 H, 한 번만)
#   이 repo를 Jetson에 clone한 뒤:  bash lerobot_so101/scripts/setup_lab.sh
set -e
HERE=$(cd "$(dirname "$0")/.." && pwd)
mkdir -p ~/so101_lab
[ -d ~/so101_lab/SO-ARM100 ] || git clone --depth 1 https://github.com/TheRobotStudio/SO-ARM100.git ~/so101_lab/SO-ARM100
mkdir -p ~/so101_lab/sim
cp -n "$HERE"/sim/*.py "$HERE"/sim/waypoints_ref.png ~/so101_lab/sim/ 2>/dev/null || true
echo "완료: ~/so101_lab/SO-ARM100 (MuJoCo 모델), ~/so101_lab/sim (실습 코드)"
echo "기존 파일은 덮어쓰지 않았습니다. 새 버전으로 바꾸려면: cp $HERE/sim/*.py ~/so101_lab/sim/"
