#!/usr/bin/env python3
# scripts/record_trajectory.py
# 로봇 팔의 움직임 궤적을 기록하는 스크립트

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import json
import os
from datetime import datetime

class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('trajectory_recorder')
        
        # 기록 데이터 저장
        self.joint_trajectory = []
        self.pose_trajectory = []
        self.recording = False
        self.start_time = None
        
        # Joint State 구독
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        # End Effector Pose 구독
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/end_effector_pose',
            self.pose_callback,
            10
        )
        
        # 타이머: 일정 주기로 기록 (100Hz)
        self.record_timer = self.create_timer(0.01, self.record_data)
        
        # 저장 디렉토리 생성
        self.save_dir = os.path.join(os.getcwd(), 'trajectories')
        os.makedirs(self.save_dir, exist_ok=True)
        
        self.get_logger().info('Trajectory Recorder initialized.')
        self.get_logger().info('Press ENTER to start/stop recording...')
        
        # 사용자 입력 대기 (별도 스레드)
        import threading
        self.input_thread = threading.Thread(target=self.user_input_handler, daemon=True)
        self.input_thread.start()
        
    def joint_callback(self, msg):
        """Joint State 수신"""
        self.current_joint_state = {
            'time': self.get_clock().now().nanoseconds / 1e9,
            'names': msg.name,
            'positions': list(msg.position),
            'velocities': list(msg.velocity) if msg.velocity else [],
            'efforts': list(msg.effort) if msg.effort else []
        }
    
    def pose_callback(self, msg):
        """End Effector Pose 수신"""
        self.current_pose = {
            'time': self.get_clock().now().nanoseconds / 1e9,
            'position': {
                'x': msg.pose.position.x,
                'y': msg.pose.position.y,
                'z': msg.pose.position.z
            },
            'orientation': {
                'x': msg.pose.orientation.x,
                'y': msg.pose.orientation.y,
                'z': msg.pose.orientation.z,
                'w': msg.pose.orientation.w
            }
        }
    
    def record_data(self):
        """기록 중일 때 데이터 저장"""
        if not self.recording:
            return
        
        # 현재 시간
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed_time = current_time - self.start_time
        
        # Joint State 기록
        if hasattr(self, 'current_joint_state'):
            data = self.current_joint_state.copy()
            data['elapsed_time'] = elapsed_time
            self.joint_trajectory.append(data)
        
        # Pose 기록
        if hasattr(self, 'current_pose'):
            data = self.current_pose.copy()
            data['elapsed_time'] = elapsed_time
            self.pose_trajectory.append(data)
    
    def user_input_handler(self):
        """사용자 입력 처리"""
        while rclpy.ok():
            input()  # ENTER 대기
            if not self.recording:
                self.start_recording()
            else:
                self.stop_recording()
    
    def start_recording(self):
        """기록 시작"""
        self.recording = True
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        self.joint_trajectory = []
        self.pose_trajectory = []
        self.get_logger().info('🔴 Recording started...')
    
    def stop_recording(self):
        """기록 중지 및 저장"""
        self.recording = False
        self.get_logger().info('⏹️  Recording stopped.')
        
        # 파일 저장
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'trajectory_{timestamp}.json'
        filepath = os.path.join(self.save_dir, filename)
        
        trajectory_data = {
            'joint_trajectory': self.joint_trajectory,
            'pose_trajectory': self.pose_trajectory,
            'duration': self.joint_trajectory[-1]['elapsed_time'] if self.joint_trajectory else 0,
            'num_points': len(self.joint_trajectory)
        }
        
        with open(filepath, 'w') as f:
            json.dump(trajectory_data, f, indent=2)
        
        self.get_logger().info(f'✅ Trajectory saved: {filepath}')
        self.get_logger().info(f'   Duration: {trajectory_data["duration"]:.2f}s, Points: {trajectory_data["num_points"]}')
        self.get_logger().info('Press ENTER to start new recording...')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Trajectory Recorder shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

