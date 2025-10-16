# Fruit Robot Project - Manipulation Gazebo Launch

TurtleBot3 Manipulation 시뮬레이션 환경에서 과일 인식 및 조작 시스템을 실행하는 통합 launch 파일입니다.

## 📋 목차

- [개요](#개요)
- [시스템 요구사항](#시스템-요구사항)
- [빠른 시작](#빠른-시작)
- [Launch 파일 설명](#launch-파일-설명)
- [실행 타임라인](#실행-타임라인)
- [Launch Arguments](#launch-arguments)
- [실행 예제](#실행-예제)
- [트러블슈팅](#트러블슈팅)

---

## 개요

`manipulation_gazebo.launch.py`는 다음 구성 요소들을 통합 실행합니다:

- 🤖 **TurtleBot3 Manipulation**: OpenManipulator-X가 장착된 TurtleBot3
- 🌍 **Gazebo 시뮬레이션**: 물리 엔진 기반 3D 시뮬레이션 환경
- 🍎 **Fruit Spawner**: 과일 모델 자동 생성
- 📹 **Ceiling Camera**: 천장 카메라 및 뷰어
- 🔌 **Camera Streamer**: ZeroMQ 기반 실시간 영상 스트리밍 및 YOLO 객체 탐지

---

## 시스템 요구사항

### 필수 패키지
```bash
# ROS 2 Humble
sudo apt install ros-humble-desktop

# TurtleBot3 패키지
sudo apt install ros-humble-turtlebot3*

# Gazebo
sudo apt install ros-humble-gazebo-ros-pkgs

# Python 의존성
pip install opencv-python pyzmq ultralytics
```

### 환경 변수
```bash
export TURTLEBOT3_MODEL=waffle
```

---

## 빠른 시작

### 1. 빌드
```bash
cd ~/turtle_ws
colcon build
source install/setup.bash
```

### 2. 실행
```bash
ros2 launch fruit manipulation_gazebo.launch.py
```

### 3. 결과 확인
- Gazebo 창이 열리고 TurtleBot3 Manipulation 로봇이 표시됩니다
- 5초 후 과일들이 생성됩니다
- 천장 카메라 뷰어 창이 표시됩니다
- 10초 후 YOLO 객체 탐지가 시작됩니다

---

## Launch 파일 설명

### 주요 기능

#### 1. **환경 설정**
```python
GAZEBO_MODEL_PATH 설정
```
- Gazebo가 fruit 패키지의 모델을 찾을 수 있도록 경로 추가
- `model://fruits/banana/...` 형식의 URI 사용 가능

#### 2. **TurtleBot3 Manipulation Gazebo**
- `turtlebot3_manipulation_gazebo/gazebo.launch.py` 포함
- OpenManipulator-X가 장착된 TurtleBot3 생성
- empty_world.world 환경 로드 (기본값)

#### 3. **Fruit Spawner** (5초 지연)
- 노드: `fruit_spawner`
- 기능: 바나나, 오렌지, 구아바 3D 모델 생성
- 위치: 로봇 앞 (-1.4, -0.0~-0.05, 0.5)
- 물리 속성: 질량, 마찰력, 감쇠 포함

#### 4. **Ceiling Camera** (5초 지연)
- Launch: `ceiling_camera.launch.py` 포함
- 카메라 생성: 천장에서 내려다보는 시점 (0, 0, 1.0)
- 뷰어 실행: 카메라 영상을 GUI 창에 표시 (추가 5초 후)

#### 5. **Camera Streamer ZeroMQ** (10초 지연)
- 노드: `camera_streamer_zeromq`
- 기능:
  - 천장 카메라 영상 수신
  - YOLO v8 객체 탐지 (과일 인식)
  - ZeroMQ PUB/SUB 패턴으로 영상 스트리밍
  - Confidence threshold: 0.4

---

## 실행 타임라인

```
t=0s    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        │  Gazebo 시뮬레이션 시작
        │  TurtleBot3 Manipulation 로봇 생성
        └─ GAZEBO_MODEL_PATH 환경 변수 설정

t=5s    ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        ├─ Fruit Spawner 실행
        │   └─ 바나나, 오렌지, 구아바 생성 🍌🍊
        └─ Ceiling Camera Spawn
            └─ 천장 카메라 모델 생성 📹

t=10s   ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        ├─ Ceiling Camera Viewer 실행
        │   └─ 카메라 영상 GUI 표시
        └─ Camera Streamer ZeroMQ 실행
            └─ YOLO 객체 탐지 시작 🎯
```

---

## Launch Arguments

### 로봇 초기 위치

| Argument | 기본값 | 설명 |
|----------|--------|------|
| `x_pose` | -2.00 | 로봇의 초기 X 위치 (m) |
| `y_pose` | -0.50 | 로봇의 초기 Y 위치 (m) |
| `z_pose` | 0.01 | 로봇의 초기 Z 위치 (m) |
| `roll` | 0.00 | 로봇의 초기 Roll 각도 (rad) |
| `pitch` | 0.00 | 로봇의 초기 Pitch 각도 (rad) |
| `yaw` | 0.00 | 로봇의 초기 Yaw 각도 (rad) |

### 시뮬레이션 설정

| Argument | 기본값 | 설명 |
|----------|--------|------|
| `world` | empty_world.world | Gazebo world 파일 경로 |
| `start_rviz` | false | RViz2 실행 여부 |
| `use_sim` | true | 시뮬레이션 모드 사용 여부 |
| `prefix` | "" | Joint/Link 이름 prefix |

---

## 실행 예제

### 기본 실행
```bash
ros2 launch fruit manipulation_gazebo.launch.py
```

### RViz2와 함께 실행
```bash
ros2 launch fruit manipulation_gazebo.launch.py start_rviz:=true
```

### 로봇 위치 변경
```bash
ros2 launch fruit manipulation_gazebo.launch.py x_pose:=0.0 y_pose:=0.0
```

### 다른 World 파일 사용
```bash
ros2 launch fruit manipulation_gazebo.launch.py \
  world:=$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/worlds/turtlebot3_world.world
```

### 모든 옵션 조합
```bash
ros2 launch fruit manipulation_gazebo.launch.py \
  start_rviz:=true \
  x_pose:=0.0 \
  y_pose:=0.0 \
  z_pose:=0.01 \
  yaw:=1.57
```

---

## 트러블슈팅

### 1. 과일이 보이지 않음

**원인**: Gazebo가 모델 경로를 찾지 못함

**해결**:
```bash
# 패키지 재빌드
colcon build --packages-select fruit
source install/setup.bash

# GAZEBO_MODEL_PATH 수동 확인
echo $GAZEBO_MODEL_PATH
```

### 2. 카메라 영상이 표시되지 않음

**원인**: 카메라가 spawn되지 않았거나 토픽이 발행되지 않음

**해결**:
```bash
# 토픽 확인
ros2 topic list | grep camera

# 카메라 토픽 수신 확인
ros2 topic echo /ceiling/ceiling_camera/image_raw --once
```

### 3. YOLO 탐지가 작동하지 않음

**원인**: 모델 파일이 없거나 경로가 잘못됨

**해결**:
```bash
# YOLO 모델 파일 확인
ls install/fruit/share/fruit/config/yolo_model/

# 로그 확인
ros2 node info /camera_streamer_zeromq
```

### 4. "Model not found" 오류

**원인**: banana/model.sdf의 URI 경로 문제

**해결**:
- `model.sdf`에서 `model://fruits/...` 형식 사용 확인
- GAZEBO_MODEL_PATH에 models 폴더 포함 확인

### 5. 과일이 계속 움직임

**원인**: 마찰력/감쇠 설정 부족

**해결**:
- model.sdf에 `<surface><friction>` 추가 확인
- `<velocity_decay>` 설정 확인
- 또는 Gazebo GUI에서 일시정지 (`Ctrl+P`)

---

## 노드 구조

```
manipulation_gazebo.launch.py
│
├─ turtlebot3_manipulation_gazebo/gazebo.launch.py
│  ├─ Gazebo Server
│  ├─ Gazebo Client
│  ├─ Robot State Publisher
│  └─ Spawn TurtleBot3 Manipulation
│
├─ fruit_spawner (5s delay)
│  └─ /spawn_entity service call
│
├─ ceiling_camera.launch.py (5s delay)
│  ├─ Spawn Ceiling Camera
│  └─ ceiling_camera_viewer (10s total)
│
└─ camera_streamer_zeromq (10s delay)
   ├─ Subscribe: /ceiling/ceiling_camera/image_raw
   ├─ YOLO Detection
   └─ Publish: ZeroMQ (tcp://*:5555)
```

---

## 주요 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/ceiling/ceiling_camera/image_raw` | sensor_msgs/Image | 천장 카메라 원본 영상 |
| `/joint_states` | sensor_msgs/JointState | 로봇 관절 상태 |
| `/cmd_vel` | geometry_msgs/Twist | 로봇 속도 명령 |
| `/spawn_entity` | gazebo_msgs/SpawnEntity | 객체 생성 서비스 |

---

## 개발자 정보

### 디렉토리 구조
```
fruit/
├── launch/
│   ├── manipulation_gazebo.launch.py   # 통합 launch 파일
│   ├── ceiling_camera.launch.py        # 천장 카메라
│   └── ...
├── fruit/
│   ├── spawner/
│   │   └── fruit_spawner.py            # 과일 생성 노드
│   ├── camera/
│   │   ├── ceiling_camera_viewer.py    # 카메라 뷰어
│   │   └── camera_streamer_zeromq.py   # ZeroMQ 스트리머
│   └── ...
├── models/
│   └── fruits/
│       ├── banana/
│       ├── orange/
│       └── guava/
└── config/
    └── yolo_model/
        └── fruits.pt                    # YOLO 모델
```

### 관련 파일
- Launch: `src/fruit/launch/manipulation_gazebo.launch.py`
- Node: `src/fruit/fruit/spawner/fruit_spawner.py`
- Node: `src/fruit/fruit/camera/camera_streamer_zeromq.py`
- Config: `src/fruit/config/yolo_config.yaml`

---

**Last Updated**: 2025-10-16

