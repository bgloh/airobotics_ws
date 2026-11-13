# 로보틱스 워크스페이스

덕키봇(Duckiebot) 제어를 위한 ROS 패키지들을 포함한 로보틱스 워크스페이스입니다.

## 프로젝트 구조

```
src/
├── my-proj1/
│   └── packages/
│       ├── led_control/          # LED 제어 패키지
│       ├── motor_control/        # 저수준 모터 제어 패키지  
│       ├── car_control/          # 고수준 차량 제어 패키지
│       ├── camera_control/       # 카메라 제어 패키지
│       └── my_first_package/     # 기본 pub/sub 예제
└── CLAUDE.md                     # 개발 가이드
lerobot_so101/                    # 11주차 LeRobot SO-ARM101 실습
```

## 11주차 · LeRobot SO-ARM101 실습 (MuJoCo sim-to-real, teleoperation, ACT, Diffusion Policy)

Jetson Orin NX + SO-ARM101 팔로 MuJoCo sim-to-real → teleoperation·시연 녹화 → ACT → Diffusion Policy 순서로 실습합니다.

> 📖 **실습 안내**: [`lerobot_so101/README.md`](lerobot_so101/README.md)
>
> 강의 슬라이드: `SO-ARM101_LeRobot_실습.pptx` (LMS 배포, 이 repo에는 없음)

**실습 방식:**
- A: Jetson에 모니터·키보드를 직접 연결 → MuJoCo viewer 창, Rerun 창을 Jetson 화면에서 확인
- B: PC에서 `ssh`로 접속 → `twin_web.py`로 MuJoCo 화면을 PC 브라우저(`http://<Jetson IP>:8000`)에서 확인

**실습 폴더 준비 (Jetson 본체 터미널):**
```bash
if [ -d ~/airobotics_ws ]; then cd ~/airobotics_ws && git pull
else git clone https://github.com/bgloh/airobotics_ws.git ~/airobotics_ws; fi
bash ~/airobotics_ws/lerobot_so101/scripts/setup_lab.sh
ls ~/so101_lab/sim
```

## 패키지 설명

### 1. led_control 패키지
덕키봇의 LED 색상을 제어하는 패키지입니다.

**기능:**
- 단일 문자 명령을 통한 LED 색상 변경
- 지원 색상: GREEN(G), BLUE(B), RED(R), WHITE(W), OFF(O)
- rosservice를 통한 LED 패턴 설정

**실행 방법:**
```bash
rosrun led_control led_controller_node.py
```

### 2. motor_control 패키지  
덕키봇의 바퀴 모터를 저수준에서 제어하는 패키지입니다.

**기능:**
- WASD 키를 통한 방향 제어
- W: 전진, S: 후진, A: 좌회전, D: 우회전, X: 정지
- 직접적인 바퀴 속도 제어 (WheelsCmdStamped)
- 최적화된 ROS 메시지 발행

**토픽:**
- 출력: `/duckiealexa/wheels_driver_node/wheels_cmd`

**실행 방법:**
```bash
rosrun motor_control motor_controller_node.py
```

### 3. car_control 패키지
덕키봇의 고수준 차량 제어를 위한 패키지입니다.

**기능:**
- 향상된 인터랙티브 제어
- 복합 움직임 지원 (전진+회전)
- Twist2DStamped 메시지를 통한 정밀한 속도 제어
- 선형 및 각속도 독립 제어
- 두 가지 제어 모드 제공

**제어 노드:**

**3.1 car_controller_node.py (Enter 방식)**
- 제어 명령: W/S/A/D/Q/E/X/Z (대문자)
- Enter 키 입력 후 실행
- 표준 터미널 인터페이스

**3.2 car_controller2_node.py (실시간 방식)**
- 제어 명령: w/s/a/d/q/e/x/z (소문자)
- Enter 불필요 - 즉시 반응
- curses 기반 실시간 인터페이스
- 실시간 상태 표시

**공통 제어 명령:**
- w/s: 전진/후진 (선형 속도)
- a/d: 좌/우회전 (각속도)
- q/e: 전진+좌회전/전진+우회전
- x: 정지, z: 종료

**토픽:**
- 출력: `/{ROBOT_NAME}/car_cmd_switch_node/cmd`

**실행 방법:**
```bash
# Enter 방식 (기본)
rosrun car_control car_controller_node.py

# 실시간 방식 (추천)
rosrun car_control car_controller2_node.py
```

### 4. camera_control 패키지
덕키봇의 카메라 영상을 처리하는 패키지입니다.

**기능:**
- 그레이스케일 변환 및 엣지 검출
- 처리된 영상에 텍스트 오버레이 추가
- Docker 환경에서 동작하도록 최적화

**토픽:**
- 입력: `/{vehicle_name}/camera_node/image/compressed`
- 출력: 
  - `/{vehicle_name}/camera_node/image/processed/grayscaled/compressed`
  - `/{vehicle_name}/camera_node/image/processed/edgedetected/compressed`

**실행 방법:**
```bash
rosrun camera_control camera_controller_node.py
```

### 5. my_first_package 패키지
ROS의 기본 publisher/subscriber 패턴을 학습하기 위한 예제 패키지입니다.

**구성:**
- `my_publisher_node.py`: 메시지 발행 노드
- `my_subscriber_node.py`: 메시지 구독 노드

## ROS 패키지 생성 방법

Duckietown 가이드라인에 따른 catkin 패키지 생성 방법입니다.

> 📖 **상세 가이드**: [Duckietown 공식 문서](https://docs.duckietown.com/daffy/devmanual-software/beginner/ros/catkin-packages.html)

### 1단계: 패키지 디렉토리 생성
```bash
mkdir -p ./packages/my_package
```

### 2단계: package.xml 파일 생성
```xml
<package>
  <name>my_package</name>
  <version>0.0.1</version>
  <description>
  Duckietown에서의 첫 번째 Catkin 패키지입니다.
  </description>
  <maintainer email="[your email]">YOUR_FULL_NAME</maintainer>
  <license>None</license>

  <buildtool_depend>catkin</buildtool_depend>
</package>
```

### 3단계: CMakeLists.txt 파일 생성
```cmake
cmake_minimum_required(VERSION 2.8.3)
project(my_package)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()
```

### 4단계: Python 스크립트 추가 (선택사항)
Python 노드를 추가하려면:

1. `scripts/` 디렉토리 생성:
```bash
mkdir -p ./packages/my_package/scripts
```

2. Python 스크립트 생성 후 실행 권한 부여:
```bash
chmod +x ./packages/my_package/scripts/my_node.py
```

3. CMakeLists.txt에 스크립트 추가:
```cmake
catkin_install_python(PROGRAMS
  scripts/my_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

## 빌드 및 실행

### 워크스페이스 빌드
```bash
cd /path/to/robotics_hansung/src
catkin_make
source devel/setup.bash
```

### 개별 노드 실행
```bash
# LED 컨트롤러
rosrun led_control led_controller_node.py

# 저수준 모터 컨트롤러 (바퀴 직접 제어)
rosrun motor_control motor_controller_node.py

# 고수준 차량 컨트롤러 (Enter 방식)
rosrun car_control car_controller_node.py

# 고수준 차량 컨트롤러 (실시간 방식)
rosrun car_control car_controller2_node.py

# 카메라 컨트롤러
rosrun camera_control camera_controller_node.py

# 퍼블리셰/서브스크라이버 예제
rosrun my_first_package my_publisher_node.py
rosrun my_first_package my_subscriber_node.py
```

## 개발 환경 설정

### 필요 사항
- ROS (Robot Operating System)
- Python 3
- OpenCV
- Duckietown 소프트웨어 스택

### 의존성 설치
각 패키지의 의존성은 해당 package.xml 파일에 정의되어 있습니다. 기본적으로 다음이 필요합니다:
- `rospy`: ROS Python 클라이언트 라이브러리
- `std_msgs`: 표준 메시지 타입
- `sensor_msgs`: 센서 데이터 메시지
- `cv_bridge`: OpenCV와 ROS 간 이미지 변환

## 주요 특징

1. **실시간 제어**: 모든 컨트롤러 노드는 실시간 인터랙티브 모드 지원
2. **Docker 호환**: 라즈베리파이 Docker 환경에서 동작하도록 설계
3. **모듈화 설계**: 각 기능별로 독립적인 패키지로 구성
4. **교육용 최적화**: 로보틱스 학습을 위한 명확한 구조와 주석

## 문제해결

### 일반적인 문제들
1. **권한 문제**: 스크립트 파일에 실행 권한이 있는지 확인
2. **환경 변수**: `VEHICLE_NAME` 환경 변수가 설정되어 있는지 확인  
3. **의존성**: 모든 ROS 의존성이 설치되어 있는지 확인

### 로그 확인
```bash
# ROS 로그 확인
roslog find [package_name]

# 노드 상태 확인  
rosnode list
rosnode info /node_name
```

## ROS 명령어 치트 시트

### 기본 명령어
| 명령어 | 설명 | 예시 |
|--------|------|------|
| `roscore` | ROS 마스터 시작 | `roscore` |
| `rosrun` | 패키지 노드 실행 | `rosrun turtlesim turtlesim_node` |
| `roslaunch` | Launch 파일 실행 | `roslaunch turtlesim multisim.launch` |

### 노드 관리
| 명령어 | 설명 | 예시 |
|--------|------|------|
| `rosnode list` | 실행 중인 노드 목록 | `rosnode list` |
| `rosnode info` | 노드 정보 확인 | `rosnode info /turtlesim` |
| `rosnode kill` | 노드 종료 | `rosnode kill /turtlesim` |
| `rosnode kill -a` | 모든 노드 종료 | `rosnode kill -a` |

### 토픽 관리
| 명령어 | 설명 | 예시 |
|--------|------|------|
| `rostopic list` | 토픽 목록 확인 | `rostopic list` |
| `rostopic info` | 토픽 정보 확인 | `rostopic info /cmd_vel` |
| `rostopic echo` | 토픽 메시지 확인 | `rostopic echo /cmd_vel` |
| `rostopic hz` | 토픽 주파수 확인 | `rostopic hz /cmd_vel` |
| `rostopic pub` | 토픽에 메시지 발행 | `rostopic pub /cmd_vel geometry_msgs/Twist "..."` |

### 서비스 관리
| 명령어 | 설명 | 예시 |
|--------|------|------|
| `rosservice list` | 서비스 목록 확인 | `rosservice list` |
| `rosservice info` | 서비스 정보 확인 | `rosservice info /spawn` |
| `rosservice call` | 서비스 호출 | `rosservice call /spawn 1 1 0 "turtle2"` |
| `rosservice type` | 서비스 타입 확인 | `rosservice type /spawn` |

### 패키지 관리
| 명령어 | 설명 | 예시 |
|--------|------|------|
| `rospack find` | 패키지 경로 찾기 | `rospack find turtlesim` |
| `rospack depends` | 패키지 의존성 확인 | `rospack depends turtlesim` |
| `rospack list` | 패키지 목록 확인 | `rospack list` |

### 메시지 및 타입
| 명령어 | 설명 | 예시 |
|--------|------|------|
| `rosmsg show` | 메시지 타입 구조 확인 | `rosmsg show geometry_msgs/Twist` |
| `rosmsg list` | 메시지 타입 목록 | `rosmsg list` |
| `rossrv show` | 서비스 타입 구조 확인 | `rossrv show turtlesim/Spawn` |
| `rossrv list` | 서비스 타입 목록 | `rossrv list` |

### 파라미터 서버
| 명령어 | 설명 | 예시 |
|--------|------|------|
| `rosparam list` | 파라미터 목록 확인 | `rosparam list` |
| `rosparam get` | 파라미터 값 확인 | `rosparam get /background_r` |
| `rosparam set` | 파라미터 설정 | `rosparam set /background_r 255` |
| `rosparam delete` | 파라미터 삭제 | `rosparam delete /background_r` |

### 빌드 및 워크스페이스
| 명령어 | 설명 | 예시 |
|--------|------|------|
| `catkin_make` | 워크스페이스 빌드 | `catkin_make` |
| `catkin_make --only-pkg-with-deps` | 특정 패키지만 빌드 | `catkin_make --only-pkg-with-deps my_package` |
| `source devel/setup.bash` | 환경 설정 소스 | `source devel/setup.bash` |
| `catkin_make clean` | 빌드 파일 정리 | `catkin_make clean` |

### 도구 및 시각화
| 명령어 | 설명 | 예시 |
|--------|------|------|
| `rqt` | RQT 도구 모음 실행 | `rqt` |
| `rqt_graph` | 노드 그래프 시각화 | `rqt_graph` |
| `rqt_console` | 로그 뷰어 | `rqt_console` |
| `rqt_plot` | 토픽 데이터 플롯 | `rqt_plot /turtle1/pose/x` |
| `rviz` | 3D 시각화 도구 | `rviz` |

### 로그 및 디버깅
| 명령어 | 설명 | 예시 |
|--------|------|------|
| `rosbag record` | 토픽 데이터 기록 | `rosbag record /cmd_vel /odom` |
| `rosbag play` | 기록된 데이터 재생 | `rosbag play data.bag` |
| `rosbag info` | 백 파일 정보 확인 | `rosbag info data.bag` |
