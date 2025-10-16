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
- [웹 API를 통한 파라미터 제어](#웹-api를-통한-파라미터-제어)
- [로봇 제어 API 사용 예제](#로봇-제어-api-사용-예제)
- [트러블슈팅](#트러블슈팅)
- [노드 구조](#노드-구조)
- [주요 토픽 및 서비스](#주요-토픽-및-서비스)

---

## 개요

`manipulation_gazebo.launch.py`는 다음 구성 요소들을 통합 실행합니다:

- 🤖 **TurtleBot3 Manipulation**: OpenManipulator-X가 장착된 TurtleBot3
- 🌍 **Gazebo 시뮬레이션**: 물리 엔진 기반 3D 시뮬레이션 환경
- 🍎 **Fruit Spawner**: 과일 모델 자동 생성
- 📹 **Ceiling Camera**: 천장 카메라 및 뷰어
- 🔌 **Camera Streamer**: ZeroMQ 기반 실시간 영상 스트리밍 및 YOLO 객체 탐지
- 🌐 **Parameter Server**: 웹 브라우저에서 ROS 2 파라미터 제어 (REST API)
- 🎮 **Robot Control Server**: 로봇 전체 제어 REST API (바퀴, 팔, 그리퍼)

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
pip install opencv-python pyzmq ultralytics flask flask-cors
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
- 10초 후 Parameter Web Server가 시작됩니다 (포트 5002)
- 10초 후 Robot Control Server가 시작됩니다 (포트 5003)

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
- 동적 파라미터:
  - `use_yolo`: YOLO 사용 여부 (실시간 변경 가능)
  - `image_quality`: 이미지 품질 (1-100)
  - `streaming_fps`: 스트리밍 FPS

#### 6. **Parameter Server** (10초 지연)
- 노드: `parameter_server`
- 포트: `5002`
- 기능:
  - REST API를 통한 ROS 2 파라미터 제어
  - 웹 브라우저/앱에서 파라미터 변경 가능
  - 실시간 파라미터 업데이트 지원
  - CORS 활성화 (크로스 도메인 접근 허용)
- API 엔드포인트:
  - `POST /api/parameter/set`: 파라미터 설정
  - `GET /api/parameter/get`: 파라미터 조회
  - `POST /api/yolo/toggle`: YOLO ON/OFF
  - `GET /api/status`: 서버 상태 확인

#### 7. **Robot Control Server** (10초 지연)
- 노드: `robot_control_server`
- 포트: `5003`
- 기능:
  - REST API를 통한 로봇 전체 제어 (rosbridge 대체)
  - Windows 환경에서도 사용 가능
  - 웹/모바일 앱에서 로봇 원격 제어
  - CORS 활성화 (크로스 도메인 접근 허용)
- API 카테고리:
  - **로봇 이동**: 전진, 후진, 회전, 정지
  - **로봇 팔**: 관절 각도 제어, 홈/준비 포지션
  - **그리퍼**: 열기/닫기
  - **상태 조회**: 관절 상태, Odometry
  - **과일 스폰**: 개별/전체 과일 생성

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
        ├─ Camera Streamer ZeroMQ 실행
        │   └─ YOLO 객체 탐지 시작 🎯
        ├─ Parameter Server 실행
        │   └─ REST API 서버 시작 (포트 5002) 🌐
        └─ Robot Control Server 실행
            └─ 로봇 제어 REST API 서버 시작 (포트 5003) 🎮
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

## 웹 API를 통한 파라미터 제어

### REST API 사용법

Parameter Server가 실행 중이면 웹 브라우저나 다른 앱에서 HTTP 요청으로 파라미터를 제어할 수 있습니다.

#### 1. **서버 상태 확인**
```bash
curl http://localhost:5002/api/status
```

**응답:**
```json
{
  "success": true,
  "message": "ROS2 Parameter Server is running",
  "ros_ok": true
}
```

#### 2. **YOLO ON/OFF**
```bash
# YOLO 끄기
curl -X POST http://localhost:5002/api/yolo/toggle \
  -H "Content-Type: application/json" \
  -d '{"use_yolo": false}'

# YOLO 켜기
curl -X POST http://localhost:5002/api/yolo/toggle \
  -H "Content-Type: application/json" \
  -d '{"use_yolo": true}'
```

**응답:**
```json
{
  "success": true,
  "message": "Parameter set successfully",
  "use_yolo": false
}
```

#### 3. **이미지 품질 변경**
```bash
curl -X POST http://localhost:5002/api/parameter/set \
  -H "Content-Type: application/json" \
  -d '{
    "node_name": "/camera_streamer_zeromq",
    "param_name": "image_quality",
    "param_value": 95
  }'
```

#### 4. **파라미터 조회**
```bash
curl "http://localhost:5002/api/parameter/get?param_name=use_yolo"
```

**응답:**
```json
{
  "success": true,
  "node_name": "/camera_streamer_zeromq",
  "param_name": "use_yolo",
  "param_value": true
}
```

### 다른 네트워크에서 접속

#### 1. ROS 컴퓨터 IP 확인
```bash
hostname -I
# 예: 192.168.1.100
```

#### 2. 방화벽 포트 열기
```bash
sudo ufw allow 5002/tcp
```

#### 3. 외부에서 접속
```bash
# 다른 컴퓨터나 스마트폰에서
curl http://192.168.1.100:5002/api/status
```

### JavaScript 예제 (웹 앱용)

```javascript
// YOLO 토글
async function toggleYolo(useYolo) {
    const response = await fetch('http://192.168.1.100:5002/api/yolo/toggle', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ use_yolo: useYolo })
    });
    const data = await response.json();
    console.log('YOLO:', data.use_yolo ? 'ON' : 'OFF');
}

// 이미지 품질 변경
async function setQuality(quality) {
    const response = await fetch('http://192.168.1.100:5002/api/parameter/set', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
            node_name: '/camera_streamer_zeromq',
            param_name: 'image_quality',
            param_value: quality
        })
    });
    const data = await response.json();
    console.log('Quality set to:', quality);
}
```

---

## 로봇 제어 API 사용 예제

Robot Control Server를 사용하여 웹이나 다른 앱에서 로봇을 제어할 수 있습니다.

### 1. **로봇 이동 제어**

#### 전진
```bash
curl -X POST http://localhost:5003/api/robot/forward \
  -H "Content-Type: application/json" \
  -d '{"duration": 2.0}'
```

#### 후진
```bash
curl -X POST http://localhost:5003/api/robot/backward \
  -H "Content-Type: application/json" \
  -d '{"duration": 2.0}'
```

#### 좌회전
```bash
curl -X POST http://localhost:5003/api/robot/turn_left \
  -H "Content-Type: application/json" \
  -d '{"duration": 1.5}'
```

#### 정지
```bash
curl -X POST http://localhost:5003/api/robot/stop
```

#### 속도 제어
```bash
curl -X POST http://localhost:5003/api/robot/move \
  -H "Content-Type: application/json" \
  -d '{
    "linear_x": 0.1,
    "angular_z": 0.3
  }'
```

### 2. **로봇 팔 제어**

#### 홈 포지션
```bash
curl -X POST http://localhost:5003/api/arm/home
```

#### 준비 포지션
```bash
curl -X POST http://localhost:5003/api/arm/ready
```

#### 관절 각도 제어
```bash
curl -X POST http://localhost:5003/api/arm/move \
  -H "Content-Type: application/json" \
  -d '{
    "joint1": 0.0,
    "joint2": -1.0,
    "joint3": 0.3,
    "joint4": 0.7,
    "duration": 3.0
  }'
```

#### 단일 관절 제어
```bash
# 첫 번째 관절 (joint_index=0)
curl -X POST http://localhost:5003/api/arm/joint/0 \
  -H "Content-Type: application/json" \
  -d '{
    "angle": 1.57,
    "duration": 2.0
  }'
```

#### 현재 관절 각도 조회
```bash
curl http://localhost:5003/api/arm/current
```

**응답:**
```json
{
  "success": true,
  "joint1": 0.0,
  "joint2": -1.0,
  "joint3": 0.3,
  "joint4": 0.7
}
```

### 3. **그리퍼 제어**

#### 그리퍼 열기
```bash
curl -X POST http://localhost:5003/api/gripper/open
```

#### 그리퍼 닫기
```bash
curl -X POST http://localhost:5003/api/gripper/close
```

### 4. **로봇 상태 조회**

```bash
curl http://localhost:5003/api/robot/status
```

**응답:**
```json
{
  "success": true,
  "joint_state": {
    "names": ["wheel_left_joint", "wheel_right_joint", "joint1", "joint2", "joint3", "joint4", "gripper_left_joint", "gripper_right_joint"],
    "positions": [0.0, 0.0, 0.0, -1.0, 0.3, 0.7, 0.019, -0.019],
    "velocities": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  },
  "odom": {
    "position": {"x": -2.0, "y": -0.5, "z": 0.01},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
    "linear_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
    "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0}
  }
}
```

### 5. **과일 스폰**

#### 단일 과일 생성
```bash
curl -X POST http://localhost:5003/api/fruit/spawn \
  -H "Content-Type: application/json" \
  -d '{
    "fruit_type": "banana",
    "x": -1.5,
    "y": 0.0,
    "z": 0.5
  }'
```

#### 모든 과일 생성
```bash
curl -X POST http://localhost:5003/api/fruit/spawn/all
```

### JavaScript 예제 (웹 앱용)

```javascript
const ROBOT_API = 'http://192.168.1.100:5003';

// 로봇 전진
async function moveForward(duration = 2.0) {
    const response = await fetch(`${ROBOT_API}/api/robot/forward`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ duration })
    });
    const data = await response.json();
    console.log('Moving forward:', data);
}

// 로봇 팔 홈 포지션
async function armToHome() {
    const response = await fetch(`${ROBOT_API}/api/arm/home`, {
        method: 'POST'
    });
    const data = await response.json();
    console.log('Arm to home:', data);
}

// 그리퍼 제어
async function controlGripper(open = true) {
    const endpoint = open ? 'open' : 'close';
    const response = await fetch(`${ROBOT_API}/api/gripper/${endpoint}`, {
        method: 'POST'
    });
    const data = await response.json();
    console.log(`Gripper ${endpoint}:`, data);
}

// 로봇 상태 조회
async function getRobotStatus() {
    const response = await fetch(`${ROBOT_API}/api/robot/status`);
    const data = await response.json();
    console.log('Robot status:', data);
    return data;
}

// 관절 각도 제어
async function moveArm(joint1, joint2, joint3, joint4, duration = 3.0) {
    const response = await fetch(`${ROBOT_API}/api/arm/move`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ joint1, joint2, joint3, joint4, duration })
    });
    const data = await response.json();
    console.log('Arm moved:', data);
}
```

### Python 예제 (외부 스크립트용)

```python
import requests
import json

ROBOT_API = 'http://localhost:5003'

def move_forward(duration=2.0):
    """로봇 전진"""
    response = requests.post(
        f'{ROBOT_API}/api/robot/forward',
        json={'duration': duration}
    )
    return response.json()

def arm_to_home():
    """팔 홈 포지션"""
    response = requests.post(f'{ROBOT_API}/api/arm/home')
    return response.json()

def open_gripper():
    """그리퍼 열기"""
    response = requests.post(f'{ROBOT_API}/api/gripper/open')
    return response.json()

def close_gripper():
    """그리퍼 닫기"""
    response = requests.post(f'{ROBOT_API}/api/gripper/close')
    return response.json()

def get_robot_status():
    """로봇 상태 조회"""
    response = requests.get(f'{ROBOT_API}/api/robot/status')
    return response.json()

def move_arm(joint1, joint2, joint3, joint4, duration=3.0):
    """팔 관절 제어"""
    response = requests.post(
        f'{ROBOT_API}/api/arm/move',
        json={
            'joint1': joint1,
            'joint2': joint2,
            'joint3': joint3,
            'joint4': joint4,
            'duration': duration
        }
    )
    return response.json()

# 사용 예제
if __name__ == '__main__':
    # 로봇 상태 확인
    status = get_robot_status()
    print('Current status:', status)
    
    # 팔을 홈으로
    arm_to_home()
    
    # 전진
    move_forward(2.0)
    
    # 그리퍼 열기
    open_gripper()
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

### 6. Parameter Server 연결 안 됨

**원인**: 서버가 실행되지 않았거나 포트가 차단됨

**해결**:
```bash
# Parameter Server 실행 확인
ros2 node list | grep parameter_server

# 포트 확인
netstat -an | grep 5002

# 방화벽 포트 열기
sudo ufw allow 5002/tcp

# 서버 상태 확인
curl http://localhost:5002/api/status
```

### 7. 파라미터 변경이 적용되지 않음

**원인**: 파라미터 콜백이 없는 노드

**해결**:
- `use_yolo`, `image_quality` 등은 실시간 변경 가능 ✅
- `streaming_fps`, `zmq_address` 등은 노드 재시작 필요
- 로그에서 "파라미터 변경" 메시지 확인

### 8. Robot Control Server 연결 안 됨

**원인**: 서버가 실행되지 않았거나 포트가 차단됨

**해결**:
```bash
# Robot Control Server 실행 확인
ros2 node list | grep robot_control_server

# 포트 확인
netstat -an | grep 5003

# 방화벽 포트 열기 (Linux)
sudo ufw allow 5003/tcp

# 서버 상태 확인
curl http://localhost:5003/api/robot/status
```

### 9. 로봇이 API 명령에 반응하지 않음

**원인**: 액션 서버가 준비되지 않음

**해결**:
```bash
# 액션 서버 확인
ros2 action list

# 필요한 액션 서버:
# - /arm_controller/follow_joint_trajectory
# - /gripper_controller/gripper_cmd

# 로봇 컨트롤러 로그 확인
ros2 node info /robot_control_server
```

### 10. API 호출 시 CORS 에러 (웹 앱)

**원인**: 브라우저의 CORS 정책

**해결**:
- Robot Control Server는 CORS가 이미 활성화되어 있음
- 브라우저 콘솔에서 에러 메시지 확인
- 올바른 서버 IP 주소 사용 확인
- HTTPS 대신 HTTP 사용

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
├─ camera_streamer_zeromq (10s delay)
│  ├─ Subscribe: /ceiling/ceiling_camera/image_raw
│  ├─ YOLO Detection (동적 ON/OFF 가능)
│  └─ Publish: ZeroMQ (tcp://*:5555)
│
├─ parameter_server (10s delay)
│  ├─ REST API Server (포트 5002)
│  ├─ Endpoint: /api/parameter/set
│  ├─ Endpoint: /api/parameter/get
│  ├─ Endpoint: /api/yolo/toggle
│  └─ Endpoint: /api/status
│
└─ robot_control_server (10s delay)
   ├─ REST API Server (포트 5003)
   ├─ Robot Movement: /api/robot/*
   ├─ Arm Control: /api/arm/*
   ├─ Gripper Control: /api/gripper/*
   └─ Fruit Spawner: /api/fruit/*
```

---

## 주요 토픽 및 서비스

### ROS 2 토픽

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/ceiling/ceiling_camera/image_raw` | sensor_msgs/Image | 천장 카메라 원본 영상 |
| `/joint_states` | sensor_msgs/JointState | 로봇 관절 상태 |
| `/cmd_vel` | geometry_msgs/Twist | 로봇 속도 명령 |

### ROS 2 서비스

| 서비스 | 타입 | 설명 |
|--------|------|------|
| `/spawn_entity` | gazebo_msgs/SpawnEntity | 객체 생성 |
| `/camera_streamer_zeromq/set_parameters` | rcl_interfaces/SetParameters | 파라미터 설정 |
| `/camera_streamer_zeromq/get_parameters` | rcl_interfaces/GetParameters | 파라미터 조회 |

### REST API 엔드포인트

#### Parameter Server (포트 5002)

| 엔드포인트 | 메서드 | 설명 |
|------------|--------|------|
| `/api/status` | GET | 서버 상태 확인 |
| `/api/yolo/toggle` | POST | YOLO ON/OFF 토글 |
| `/api/parameter/set` | POST | 파라미터 설정 |
| `/api/parameter/get` | GET | 파라미터 조회 |

**베이스 URL**: `http://localhost:5002` (또는 서버 IP)

#### Robot Control Server (포트 5003)

| 엔드포인트 | 메서드 | 설명 |
|------------|--------|------|
| `/api/robot/move` | POST | 로봇 이동 (속도 제어) |
| `/api/robot/stop` | POST | 로봇 정지 |
| `/api/robot/status` | GET | 로봇 상태 조회 |
| `/api/robot/forward` | POST | 전진 |
| `/api/robot/backward` | POST | 후진 |
| `/api/robot/turn_left` | POST | 좌회전 |
| `/api/robot/turn_right` | POST | 우회전 |
| `/api/robot/home` | POST | 로봇 전체 홈 포지션 |
| `/api/arm/move` | POST | 로봇 팔 이동 (관절 각도) |
| `/api/arm/home` | POST | 팔 홈 포지션 |
| `/api/arm/ready` | POST | 팔 준비 포지션 |
| `/api/arm/current` | GET | 현재 관절 각도 조회 |
| `/api/arm/joint/<index>` | POST | 단일 관절 제어 |
| `/api/gripper/open` | POST | 그리퍼 열기 |
| `/api/gripper/close` | POST | 그리퍼 닫기 |
| `/api/fruit/spawn` | POST | 과일 생성 (개별) |
| `/api/fruit/spawn/all` | POST | 모든 과일 생성 |

**베이스 URL**: `http://localhost:5003` (또는 서버 IP)

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
│   │   └── camera_streamer_zeromq.py   # ZeroMQ 스트리머 + YOLO
│   ├── api/
│   │   ├── parameter_server.py         # REST API 서버 (파라미터 제어)
│   │   └── robot_control_server.py     # REST API 서버 (로봇 제어)
│   └── ...
├── models/
│   └── fruits/
│       ├── banana/
│       │   ├── model.sdf                # 바나나 모델 (상대경로)
│       │   └── banana_v03.obj
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
- Node: `src/fruit/fruit/api/parameter_server.py`
- Node: `src/fruit/fruit/api/robot_control_server.py` ⭐ 추가
- Config: `src/fruit/config/yolo_config.yaml`

---

**Last Updated**: 2025-10-16

---

## 추가 기능

### 동적 파라미터 제어

`camera_streamer_zeromq` 노드는 실행 중에 파라미터를 변경할 수 있습니다:

```bash
# 명령줄에서 직접 변경
ros2 param set /camera_streamer_zeromq use_yolo false
ros2 param set /camera_streamer_zeromq image_quality 95

# 또는 웹 API를 통해 변경
curl -X POST http://localhost:5002/api/yolo/toggle \
  -H "Content-Type: application/json" \
  -d '{"use_yolo": false}'
```

### 지원되는 동적 파라미터

| 파라미터 | 타입 | 기본값 | 실시간 변경 |
|----------|------|--------|------------|
| `use_yolo` | bool | true | ✅ 가능 |
| `image_quality` | int | 85 | ✅ 가능 |
| `streaming_fps` | int | 30 | ⚠️ 재시작 필요 |
| `zmq_address` | string | tcp://*:5555 | ❌ 재시작 필요 |
| `image_topic` | string | /ceiling/... | ❌ 재시작 필요 |

