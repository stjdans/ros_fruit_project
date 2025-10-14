#!/usr/bin/env python3
# fruit/manipulation/trajectory_controller.py
# FollowJointTrajectory 액션을 사용한 로봇 팔 제어 노드

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
import math


class TrajectoryController(Node):
    """FollowJointTrajectory 액션을 사용한 로봇 팔 제어 노드"""
    
    def __init__(self):
        super().__init__('trajectory_controller')
        
        # 액션 클라이언트 생성
        self._arm_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        
        self._gripper_action_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd'
        )
        
        # 현재 관절 상태 구독
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_joint_positions = {}
        self.get_logger().info('✅ TrajectoryController 초기화 완료')
        
    def joint_state_callback(self, msg):
        """현재 관절 상태 업데이트"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]
    
    # ==================== 로봇 팔 제어 ====================
    
    def move_to_joint_positions(self, joint_positions, duration_sec=3.0, wait=True):
        """
        특정 관절 위치로 이동
        
        Args:
            joint_positions (dict): {'joint1': 0.0, 'joint2': -0.5, ...}
            duration_sec (float): 이동 시간 (초)
            wait (bool): 완료까지 대기할지 여부
        """
        # 액션 서버 대기
        if not self._arm_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Arm action server not available')
            return False
        
        # Goal 메시지 생성
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        
        # 관절 이름과 위치 설정
        goal_msg.trajectory.joint_names = list(joint_positions.keys())
        
        # 궤적 포인트 생성
        point = JointTrajectoryPoint()
        point.positions = list(joint_positions.values())
        point.velocities = [0.0] * len(joint_positions)
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        
        goal_msg.trajectory.points = [point]
        
        self.get_logger().info(f'🚀 Moving to: {joint_positions}')
        
        # 액션 전송
        send_goal_future = self._arm_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.arm_feedback_callback
        )
        
        if wait:
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error('❌ Goal rejected')
                return False
            
            self.get_logger().info('✅ Goal accepted')
            
            # 결과 대기
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            
            result = result_future.result().result
            
            if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                self.get_logger().info('✅ Movement succeeded!')
                return True
            else:
                self.get_logger().error(f'❌ Movement failed: {result.error_string}')
                return False
        else:
            send_goal_future.add_done_callback(self.arm_goal_response_callback)
            return True
    
    def move_to_joint_trajectory(self, trajectory_points, joint_names=None):
        """
        복잡한 궤적으로 이동 (여러 포인트)
        
        Args:
            trajectory_points (list): [
                {'positions': [0.0, -0.5, 0.5, 0.0], 'time': 2.0},
                {'positions': [0.5, -1.0, 1.0, -0.5], 'time': 4.0},
            ]
            joint_names (list): ['joint1', 'joint2', 'joint3', 'joint4']
        """
        if joint_names is None:
            joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        
        # 액션 서버 대기
        if not self._arm_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Arm action server not available')
            return False
        
        # Goal 메시지 생성
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = joint_names
        
        # 각 포인트를 JointTrajectoryPoint로 변환
        points = []
        for traj_point in trajectory_points:
            point = JointTrajectoryPoint()
            point.positions = traj_point['positions']
            point.velocities = traj_point.get('velocities', [0.0] * len(joint_names))
            
            time_sec = traj_point['time']
            point.time_from_start = Duration(
                sec=int(time_sec),
                nanosec=int((time_sec % 1) * 1e9)
            )
            points.append(point)
        
        goal_msg.trajectory.points = points
        
        self.get_logger().info(f'🚀 Executing trajectory with {len(points)} points')
        
        # 액션 전송
        send_goal_future = self._arm_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.arm_feedback_callback
        )
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ Trajectory goal rejected')
            return False
        
        # 결과 대기
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('✅ Trajectory execution succeeded!')
            return True
        else:
            self.get_logger().error(f'❌ Trajectory failed: {result.error_string}')
            return False
    
    # ==================== 그리퍼 제어 ====================
    
    def control_gripper(self, position, max_effort=1.0, wait=True):
        """
        그리퍼 제어
        
        Args:
            position (float): 0.0 (닫힘) ~ 0.019 (열림)
            max_effort (float): 최대 힘
            wait (bool): 완료까지 대기할지 여부
        """
        # 액션 서버 대기
        if not self._gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Gripper action server not available')
            return False
        
        # Goal 메시지 생성
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort
        
        action_name = "열기" if position > 0.01 else "닫기"
        self.get_logger().info(f'🤖 그리퍼 {action_name}: position={position}')
        
        # 액션 전송
        send_goal_future = self._gripper_action_client.send_goal_async(goal_msg)
        
        if wait:
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error('❌ Gripper goal rejected')
                return False
            
            # 결과 대기
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            
            self.get_logger().info(f'✅ 그리퍼 {action_name} 완료')
            return True
        else:
            return True
    
    def open_gripper(self, wait=True):
        """그리퍼 열기"""
        return self.control_gripper(0.019, wait=wait)
    
    def close_gripper(self, wait=True):
        """그리퍼 닫기"""
        return self.control_gripper(0.0, wait=wait)
    
    # ==================== 콜백 함수 ====================
    
    def arm_feedback_callback(self, feedback_msg):
        """실시간 피드백 수신"""
        feedback = feedback_msg.feedback
        # self.get_logger().info(f'📊 Current: {feedback.actual.positions}')
    
    def arm_goal_response_callback(self, future):
        """목표가 수락되었는지 확인 (비동기)"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected')
            return
        
        self.get_logger().info('✅ Goal accepted')
        
        # 결과 대기
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.arm_get_result_callback)
    
    def arm_get_result_callback(self, future):
        """최종 결과 수신 (비동기)"""
        result = future.result().result
        
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('✅ Movement succeeded!')
        else:
            self.get_logger().error(f'❌ Movement failed: {result.error_string}')
    
    # ==================== 미리 정의된 포즈 ====================
    
    def go_home(self):
        """홈 포지션으로 이동"""
        self.get_logger().info('🏠 Going to home position...')
        return self.move_to_joint_positions({
            'joint1': 0.0,
            'joint2': 0.0,
            'joint3': 0.0,
            'joint4': 0.0
        }, duration_sec=3.0)
    
    def go_ready_pose(self):
        """준비 자세로 이동"""
        self.get_logger().info('📐 Going to ready pose...')
        return self.move_to_joint_positions({
            'joint1': 0.0,
            'joint2': -0.785,  # -45도
            'joint3': 0.785,   # 45도
            'joint4': 0.0
        }, duration_sec=3.0)
    
    def go_pick_pose(self):
        """물건을 집을 수 있는 자세로 이동"""
        self.get_logger().info('🔽 Going to pick pose...')
        return self.move_to_joint_positions({
            'joint1': 0.0,
            'joint2': -1.0,
            'joint3': 1.2,
            'joint4': -0.5
        }, duration_sec=4.0)
    
    def demo_wave(self):
        """손 흔들기 데모"""
        self.get_logger().info('👋 Starting wave demo...')
        
        trajectory = [
            {'positions': [0.0, -0.5, 0.5, 0.0], 'time': 2.0},
            {'positions': [0.5, -0.5, 0.5, 0.0], 'time': 3.0},
            {'positions': [-0.5, -0.5, 0.5, 0.0], 'time': 4.0},
            {'positions': [0.5, -0.5, 0.5, 0.0], 'time': 5.0},
            {'positions': [0.0, 0.0, 0.0, 0.0], 'time': 7.0},
        ]
        
        return self.move_to_joint_trajectory(trajectory)
    
    def get_current_joint_position(self, joint_name):
        """현재 관절 위치 반환"""
        return self.current_joint_positions.get(joint_name, 0.0)


def main(args=None):
    rclpy.init(args=args)
    controller = TrajectoryController()
    
    try:
        # 간단한 데모 실행
        print("\n" + "="*50)
        print("🤖 Trajectory Controller Demo")
        print("="*50 + "\n")
        
        # 1. 홈 포지션으로 이동
        input("Press ENTER to go to HOME position...")
        controller.go_home()
        
        # 2. 준비 자세
        input("\nPress ENTER to go to READY pose...")
        controller.go_ready_pose()
        
        # 3. 그리퍼 열기
        input("\nPress ENTER to OPEN gripper...")
        controller.open_gripper()
        
        # 4. 픽 자세
        input("\nPress ENTER to go to PICK pose...")
        controller.go_pick_pose()
        
        # 5. 그리퍼 닫기
        input("\nPress ENTER to CLOSE gripper...")
        controller.close_gripper()
        
        # 6. 준비 자세로 복귀
        input("\nPress ENTER to go back to READY pose...")
        controller.go_ready_pose()
        
        # 7. 손 흔들기 데모
        input("\nPress ENTER to start WAVE demo...")
        controller.demo_wave()
        
        # 8. 홈으로 복귀
        input("\nPress ENTER to return HOME...")
        controller.go_home()
        
        print("\n✅ Demo completed!\n")
        
    except KeyboardInterrupt:
        controller.get_logger().info('Interrupted by user')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

