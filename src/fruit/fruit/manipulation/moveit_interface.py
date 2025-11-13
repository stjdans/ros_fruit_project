# src/manipulation/moveit_interface.py

from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
import time

class MoveItInterface:
    def __init__(self, arm_group_name="manipulator", gripper_group_name="gripper"):
        # MoveIt 초기화
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.arm_group = MoveGroupCommander(arm_group_name)
        self.gripper_group = MoveGroupCommander(gripper_group_name)

        # 속도, 가속도 설정
        self.arm_group.set_max_velocity_scaling_factor(0.5)
        self.arm_group.set_max_acceleration_scaling_factor(0.5)

    # ----------------- 로봇팔 이동 -----------------
    def go_to_pose(self, pose: PoseStamped) -> bool:
        """주어진 포즈로 로봇팔 이동"""
        self.arm_group.set_pose_target(pose)
        success = self.arm_group.go(wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        return success

    def go_to_joint_positions(self, joint_positions: list) -> bool:
        """관절 위치로 이동"""
        self.arm_group.go(joint_positions, wait=True)
        self.arm_group.stop()
        return True

    # ----------------- 그리퍼 제어 -----------------
    def control_gripper(self, open_gripper=True):
        """그리퍼 열기/닫기"""
        joint_goal = self.gripper_group.get_current_joint_values()
        joint_goal[0] = 0.04 if open_gripper else 0.0
        self.gripper_group.go(joint_goal, wait=True)
        self.gripper_group.stop()
        time.sleep(0.5)

    # ----------------- 경로 및 충돌 관리 -----------------
    def add_collision_object(self, name: str, pose: PoseStamped, size=(0.1,0.1,0.1)):
        """PlanningScene에 충돌 객체 추가"""
        self.scene.add_box(name, pose, size=size)

    def remove_collision_object(self, name: str):
        self.scene.remove_world_object(name)
