#!/usr/bin/env python3
# scripts/replay_trajectory.py
# 기록된 로봇 궤적을 재생하는 스크립트

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import json
import os
import sys
import time

class TrajectoryReplayer(Node):
    def __init__(self, trajectory_file):
        super().__init__('trajectory_replayer')
        
        # 궤적 데이터 로드
        self.trajectory_file = trajectory_file
        self.load_trajectory()
        
        # Joint Trajectory Publisher
        self.joint_traj_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        self.get_logger().info(f'Trajectory Replayer initialized.')
        self.get_logger().info(f'Loaded trajectory: {trajectory_file}')
        self.get_logger().info(f'Duration: {self.duration:.2f}s, Points: {self.num_points}')
    
    def load_trajectory(self):
        """궤적 파일 로드"""
        if not os.path.exists(self.trajectory_file):
            self.get_logger().error(f'❌ Trajectory file not found: {self.trajectory_file}')
            sys.exit(1)
        
        with open(self.trajectory_file, 'r') as f:
            data = json.load(f)
        
        self.joint_trajectory = data['joint_trajectory']
        self.pose_trajectory = data['pose_trajectory']
        self.duration = data['duration']
        self.num_points = data['num_points']
        
        if not self.joint_trajectory:
            self.get_logger().error('❌ Empty trajectory data!')
            sys.exit(1)
    
    def replay(self):
        """궤적 재생"""
        self.get_logger().info('🎬 Starting trajectory replay...')
        
        # JointTrajectory 메시지 생성
        traj_msg = JointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Joint 이름 설정 (첫 번째 포인트에서 가져오기)
        traj_msg.joint_names = self.joint_trajectory[0]['names']
        
        # 궤적 포인트 추가
        for point_data in self.joint_trajectory:
            point = JointTrajectoryPoint()
            point.positions = point_data['positions']
            
            if point_data['velocities']:
                point.velocities = point_data['velocities']
            
            if point_data['efforts']:
                point.effort = point_data['efforts']
            
            # 시간 설정
            elapsed_sec = int(point_data['elapsed_time'])
            elapsed_nanosec = int((point_data['elapsed_time'] - elapsed_sec) * 1e9)
            point.time_from_start = Duration(sec=elapsed_sec, nanosec=elapsed_nanosec)
            
            traj_msg.points.append(point)
        
        # Publish
        self.joint_traj_pub.publish(traj_msg)
        self.get_logger().info(f'✅ Trajectory published! ({len(traj_msg.points)} points)')
        self.get_logger().info(f'   Estimated duration: {self.duration:.2f}s')
        
        # 재생 완료 대기
        time.sleep(self.duration + 1.0)
        self.get_logger().info('✅ Replay completed!')
    
    def replay_step_by_step(self, step_delay=0.1):
        """스텝별로 궤적 재생 (디버깅용)"""
        self.get_logger().info('🎬 Starting step-by-step replay...')
        
        for i, point_data in enumerate(self.joint_trajectory):
            self.get_logger().info(f'Step {i+1}/{self.num_points}:')
            self.get_logger().info(f'  Positions: {point_data["positions"]}')
            self.get_logger().info(f'  Time: {point_data["elapsed_time"]:.3f}s')
            
            # 단일 포인트 Publish
            traj_msg = JointTrajectory()
            traj_msg.header.stamp = self.get_clock().now().to_msg()
            traj_msg.joint_names = point_data['names']
            
            point = JointTrajectoryPoint()
            point.positions = point_data['positions']
            point.time_from_start = Duration(sec=0, nanosec=int(step_delay * 1e9))
            
            traj_msg.points.append(point)
            self.joint_traj_pub.publish(traj_msg)
            
            time.sleep(step_delay)
        
        self.get_logger().info('✅ Step-by-step replay completed!')

def main(args=None):
    rclpy.init(args=args)
    
    # 인자 확인
    if len(sys.argv) < 2:
        print('Usage: python3 replay_trajectory.py <trajectory_file.json> [--step-by-step]')
        print('Example: python3 replay_trajectory.py trajectories/trajectory_20250101_120000.json')
        sys.exit(1)
    
    trajectory_file = sys.argv[1]
    step_by_step = '--step-by-step' in sys.argv
    
    node = TrajectoryReplayer(trajectory_file)
    
    try:
        if step_by_step:
            node.replay_step_by_step(step_delay=0.1)
        else:
            node.replay()
    except KeyboardInterrupt:
        node.get_logger().info('Trajectory Replayer interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

