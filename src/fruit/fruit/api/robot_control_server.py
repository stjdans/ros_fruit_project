#!/usr/bin/env python3
"""
REST API 기반 로봇 컨트롤러 (rosbridge 대체)
Windows 환경에서도 사용 가능
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from gazebo_msgs.srv import SpawnEntity
from flask import Flask, request, jsonify
from flask_cors import CORS
import threading
import os
import time
from ament_index_python.packages import get_package_share_directory

app = Flask(__name__)
CORS(app)

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # cmd_vel 발행자 (바퀴 제어)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 액션 클라이언트 (로봇 팔 제어)
        self.arm_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        self.gripper_action_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd'
        )
        
        # 구독자 (로봇 상태)
        self.joint_state = None
        self.odom_data = None
        
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )
        
        # Spawn 서비스 클라이언트
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        
        # 패키지 경로
        self.fruit_pkg = get_package_share_directory('fruit')
        self.models_path = os.path.join(self.fruit_pkg, 'models', 'fruits')
        
        self.get_logger().info('Robot Controller 시작 (Base + Arm + Spawner)')
    
    def joint_callback(self, msg):
        """관절 상태 업데이트"""
        self.joint_state = {
            'names': msg.name,
            'positions': list(msg.position),
            'velocities': list(msg.velocity) if len(msg.velocity) > 0 else None
        }
    
    def odom_callback(self, msg):
        """Odometry 업데이트"""
        self.odom_data = {
            'position': {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z
            },
            'orientation': {
                'x': msg.pose.pose.orientation.x,
                'y': msg.pose.pose.orientation.y,
                'z': msg.pose.pose.orientation.z,
                'w': msg.pose.pose.orientation.w
            },
            'linear_velocity': {
                'x': msg.twist.twist.linear.x,
                'y': msg.twist.twist.linear.y,
                'z': msg.twist.twist.linear.z
            },
            'angular_velocity': {
                'x': msg.twist.twist.angular.x,
                'y': msg.twist.twist.angular.y,
                'z': msg.twist.twist.angular.z
            }
        }
    
    def move_robot(self, linear_x, angular_z):
        """로봇 이동 명령 발행"""
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)
        return True
    
    def stop_robot(self):
        """로봇 정지"""
        return self.move_robot(0.0, 0.0)
    
    def move_arm(self, joint_positions, duration_sec=3.0):
        """로봇 팔 이동 (관절 각도로)"""
        # 액션 서버 대기
        if not self.arm_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Arm action server not available')
            return False
        
        # Goal 메시지 생성
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
        # 궤적 포인트 생성
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.velocities = [0.0] * len(joint_positions)
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        
        goal_msg.trajectory.points = [point]
        
        # 액션 전송
        self.get_logger().info(f'Moving arm to: {joint_positions}')
        send_goal_future = self.arm_action_client.send_goal_async(goal_msg)
        
        return True
    
    def control_gripper(self, open_gripper=True):
        """그리퍼 제어"""
        if not self.gripper_action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Gripper action server not available')
            return False
        
        # Goal 메시지 생성
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.01 if open_gripper else 0.0
        goal_msg.command.max_effort = 10.0
        
        # 액션 전송
        action = 'Opening' if open_gripper else 'Closing'
        self.get_logger().info(f'{action} gripper')
        send_goal_future = self.gripper_action_client.send_goal_async(goal_msg)
        
        return True
    
    def get_current_arm_positions(self):
        """현재 로봇 팔 관절 각도 가져오기"""
        if not self.joint_state or not self.joint_state['names']:
            return None
        
        # joint1~4 위치 추출
        arm_positions = []
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
        for joint_name in joint_names:
            try:
                idx = self.joint_state['names'].index(joint_name)
                arm_positions.append(self.joint_state['positions'][idx])
            except (ValueError, IndexError):
                return None
        
        return arm_positions
    
    def move_single_joint(self, joint_index, angle, duration_sec=2.0):
        """
        한 개 관절만 움직이기
        
        Args:
            joint_index (int): 0(joint1), 1(joint2), 2(joint3), 3(joint4)
            angle (float): 목표 각도 (rad)
            duration_sec (float): 이동 시간
        """
        # 현재 관절 각도 가져오기
        current_positions = self.get_current_arm_positions()
        
        if current_positions is None:
            self.get_logger().error('현재 관절 상태를 읽을 수 없음')
            return False
        
        # 목표 관절만 변경
        target_positions = current_positions.copy()
        target_positions[joint_index] = angle
        
        self.get_logger().info(f'Joint {joint_index+1}: {current_positions[joint_index]:.2f} → {angle:.2f}')
        
        # 이동 실행
        return self.move_arm(target_positions, duration_sec)
    
    def spawn_fruit(self, fruit_type='banana', x=-1.4, y=0.0, z=0.5):
        """과일 생성"""
        # 서비스 대기
        if not self.spawn_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Spawn service not available')
            return False, 'Spawn service not available'
        
        # 과일 설정
        fruit_configs = {
            'banana': {
                'obj_path': os.path.join(self.models_path, 'banana', 'banana_v03.obj'),
                'mass': 0.15,
                'scale': '0.005 0.005 0.005'
            },
            'orange': {
                'obj_path': os.path.join(self.models_path, 'orange', 'Orange.obj'),
                'mass': 0.2,
                'scale': '0.1 0.1 0.1'
            },
            'guava': {
                'obj_path': os.path.join(self.models_path, 'guava', 'Guava.obj'),
                'mass': 0.25,
                'scale': '0.1 0.1 0.1'
            }
        }
        
        if fruit_type not in fruit_configs:
            return False, f'Unknown fruit type: {fruit_type}'
        
        config = fruit_configs[fruit_type]
        
        # 파일 존재 확인
        if not os.path.exists(config['obj_path']):
            return False, f"OBJ file not found: {config['obj_path']}"
        
        # SDF 생성
        timestamp = int(time.time())
        model_name = f"{fruit_type}_{timestamp}"
        
        sdf = f"""
        <sdf version='1.6'>
          <model name='{model_name}'>
            <link name='link'>
              <pose>0 0 0 0 0 0</pose>
              <inertial>
                <mass>{config['mass']}</mass>
                <inertia>
                  <ixx>0.001</ixx>
                  <iyy>0.001</iyy>
                  <izz>0.001</izz>
                </inertia>
              </inertial>
              <velocity_decay>
                <linear>0.05</linear>
                <angular>0.5</angular>
              </velocity_decay>
              <collision name='collision'>
                <geometry>
                  <mesh>
                    <uri>file://{config['obj_path']}</uri>
                    <scale>{config['scale']}</scale>
                  </mesh>
                </geometry>
                <surface>
                  <friction>
                    <ode>
                      <mu>100.0</mu>
                      <mu2>100.0</mu2>
                    </ode>
                  </friction>
                </surface>
              </collision>
              <visual name='visual'>
                <geometry>
                  <mesh>
                    <uri>file://{config['obj_path']}</uri>
                    <scale>{config['scale']}</scale>
                  </mesh>
                </geometry>
              </visual>
            </link>
          </model>
        </sdf>
        """
        
        # Spawn 요청
        req = SpawnEntity.Request()
        req.name = model_name
        req.xml = sdf
        req.initial_pose.position.x = float(x)
        req.initial_pose.position.y = float(y)
        req.initial_pose.position.z = float(z)
        
        future = self.spawn_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None and future.result().success:
            self.get_logger().info(f"✓ Spawned {model_name}")
            return True, model_name
        else:
            error_msg = future.result().status_message if future.result() else "Timeout"
            self.get_logger().error(f"✗ Failed to spawn {model_name}: {error_msg}")
            return False, error_msg

# 전역 변수
robot_controller = None

@app.route('/api/robot/move', methods=['POST'])
def move():
    """로봇 이동"""
    data = request.json
    linear = data.get('linear', 0.0)
    angular = data.get('angular', 0.0)
    
    robot_controller.move_robot(linear, angular)
    
    return jsonify({
        'success': True,
        'linear': linear,
        'angular': angular
    })

@app.route('/api/robot/stop', methods=['POST'])
def stop():
    """로봇 정지"""
    robot_controller.stop_robot()
    return jsonify({'success': True, 'message': 'Robot stopped'})

@app.route('/api/robot/status', methods=['GET'])
def status():
    """로봇 상태"""
    return jsonify({
        'success': True,
        'odom': robot_controller.odom_data,
        'joints': robot_controller.joint_state
    })

@app.route('/api/robot/forward', methods=['POST'])
def forward():
    """전진"""
    speed = request.json.get('speed', 0.5)
    robot_controller.move_robot(speed, 0.0)
    return jsonify({'success': True, 'action': 'forward', 'speed': speed})

@app.route('/api/robot/backward', methods=['POST'])
def backward():
    """후진"""
    speed = request.json.get('speed', 0.5)
    robot_controller.move_robot(-speed, 0.0)
    return jsonify({'success': True, 'action': 'backward', 'speed': speed})

@app.route('/api/robot/turn_left', methods=['POST'])
def turn_left():
    """좌회전"""
    speed = request.json.get('speed', 1.0)
    robot_controller.move_robot(0.0, speed)
    return jsonify({'success': True, 'action': 'turn_left', 'speed': speed})

@app.route('/api/robot/turn_right', methods=['POST'])
def turn_right():
    """우회전"""
    speed = request.json.get('speed', 1.0)
    robot_controller.move_robot(0.0, -speed)
    return jsonify({'success': True, 'action': 'turn_right', 'speed': speed})

# ==================== 로봇 팔 제어 API ====================

@app.route('/api/arm/move', methods=['POST'])
def arm_move():
    """로봇 팔 이동 (관절 각도)"""
    data = request.json
    joint_positions = data.get('positions', [0.0, 0.0, 0.0, 0.0])
    duration = data.get('duration', 3.0)
    
    success = robot_controller.move_arm(joint_positions, duration)
    
    return jsonify({
        'success': success,
        'positions': joint_positions,
        'duration': duration
    })

@app.route('/api/arm/home', methods=['POST'])
def arm_home():
    """로봇 팔 홈 포지션"""
    success = robot_controller.move_arm([0.0, 0.0, 0.0, 0.0], 3.0)
    return jsonify({'success': success, 'action': 'home'})

@app.route('/api/arm/ready', methods=['POST'])
def arm_ready():
    """로봇 팔 준비 포지션"""
    success = robot_controller.move_arm([0.0, -1.0, 0.5, 0.5], 3.0)
    return jsonify({'success': success, 'action': 'ready'})

@app.route('/api/gripper/open', methods=['POST'])
def gripper_open():
    """그리퍼 열기"""
    success = robot_controller.control_gripper(open_gripper=True)
    return jsonify({'success': success, 'action': 'open'})

@app.route('/api/gripper/close', methods=['POST'])
def gripper_close():
    """그리퍼 닫기"""
    success = robot_controller.control_gripper(open_gripper=False)
    return jsonify({'success': success, 'action': 'close'})

@app.route('/api/arm/joint/<int:joint_index>', methods=['POST'])
def move_single_joint(joint_index):
    """한 개 관절만 움직이기"""
    if joint_index < 0 or joint_index > 3:
        return jsonify({'success': False, 'message': 'joint_index must be 0-3'}), 400
    
    data = request.json
    duration = data.get('duration', 0.3)
    
    # 현재 관절 각도 가져오기
    current_positions = robot_controller.get_current_arm_positions()
    if current_positions is None:
        return jsonify({'success': False, 'message': 'Cannot get current joint state'}), 500
    
    current_angle = current_positions[joint_index]
    
    # angle 또는 delta 파라미터 확인
    if 'delta' in data:
        # delta가 "+" 또는 "-" 문자열인 경우
        delta = data.get('delta')
        if delta == '+':
            target_angle = current_angle + 0.2
        elif delta == '-':
            target_angle = current_angle - 0.2
        else:
            return jsonify({'success': False, 'message': 'delta must be "+" or "-"'}), 400
    elif 'angle' in data:
        # 절대 각도 지정
        target_angle = data.get('angle')
    else:
        return jsonify({'success': False, 'message': 'angle or delta required'}), 400
    
    success = robot_controller.move_single_joint(joint_index, target_angle, duration)
    
    return jsonify({
        'success': success,
        'joint_index': joint_index,
        'joint_name': f'joint{joint_index+1}',
        'current_angle': current_angle,
        'target_angle': target_angle,
        'delta': target_angle - current_angle,
        'duration': duration
    })

@app.route('/api/arm/current', methods=['GET'])
def get_current_positions():
    """현재 로봇 팔 관절 각도 조회"""
    positions = robot_controller.get_current_arm_positions()
    
    if positions is None:
        return jsonify({'success': False, 'message': 'Joint state not available'}), 404
    
    return jsonify({
        'success': True,
        'positions': positions,
        'joints': {
            'joint1': positions[0],
            'joint2': positions[1],
            'joint3': positions[2],
            'joint4': positions[3]
        }
    })

@app.route('/api/robot/home', methods=['POST'])
def robot_home():
    """로봇 전체 홈 포지션 (바퀴 정지 + 팔 홈)"""
    # 1. 바퀴 정지
    robot_controller.stop_robot()
    
    # 2. 팔 홈 포지션
    arm_success = robot_controller.move_arm([0.0, 0.0, 0.0, 0.0], 3.0)
    
    return jsonify({
        'success': arm_success,
        'action': 'robot_home',
        'base': 'stopped',
        'arm': 'home_position'
    })

# ==================== 과일 생성 API ====================

@app.route('/api/fruit/spawn', methods=['POST'])
def spawn_fruit():
    """과일 생성"""
    data = request.json
    fruit_type = data.get('type', 'banana')  # banana, orange, guava
    x = data.get('x', -1.4)
    y = data.get('y', 0.0)
    z = data.get('z', 0.5)
    
    success, result = robot_controller.spawn_fruit(fruit_type, x, y, z)
    
    if success:
        return jsonify({
            'success': True,
            'fruit_type': fruit_type,
            'model_name': result,
            'position': {'x': x, 'y': y, 'z': z}
        })
    else:
        return jsonify({
            'success': False,
            'message': result
        }), 500

@app.route('/api/fruit/spawn/all', methods=['POST'])
def spawn_all_fruits():
    """모든 과일 생성 (3종류)"""
    fruits = [
        {'type': 'orange', 'x': -1.4, 'y': -0.3, 'z': 0.5},
        {'type': 'banana', 'x': -1.5, 'y': -0.5, 'z': 0.5},
        {'type': 'guava', 'x': -1.4, 'y': -0.05, 'z': 0.5}
    ]
    
    results = []
    for fruit in fruits:
        success, result = robot_controller.spawn_fruit(
            fruit['type'], fruit['x'], fruit['y'], fruit['z']
        )
        results.append({
            'type': fruit['type'],
            'success': success,
            'result': result
        })
        time.sleep(0.5)  # 각 spawn 사이 대기
    
    return jsonify({
        'success': True,
        'spawned_count': sum(1 for r in results if r['success']),
        'results': results
    })

def run_flask():
    """Flask 서버 실행"""
    app.run(host='0.0.0.0', port=5003, debug=True, use_reloader=False)

def main(args=None):
    global robot_controller
    
    rclpy.init(args=args)
    robot_controller = RobotController()
    
    # Flask 서버를 별도 스레드에서 실행
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()
    
    robot_controller.get_logger().info('='*60)
    robot_controller.get_logger().info('Robot Controller REST API 실행 중')
    robot_controller.get_logger().info('REST API: http://0.0.0.0:5001')
    robot_controller.get_logger().info('='*60)
    
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

