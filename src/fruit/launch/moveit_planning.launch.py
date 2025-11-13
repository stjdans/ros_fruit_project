#!/usr/bin/env python3
# launch/moveit_planning.launch.py
# MoveIt2 경로 계획 및 실행 노드 실행

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 현재 워크스페이스 경로
    workspace_dir = os.getcwd()
    config_dir = os.path.join(workspace_dir, 'config', 'moveit_config')
    
    # MoveIt 설정 파일들
    robot_description_semantic = os.path.join(config_dir, 'robot.srdf')
    kinematics_yaml = os.path.join(config_dir, 'kinematics.yaml')
    joint_limits_yaml = os.path.join(config_dir, 'joint_limits.yaml')
    ompl_planning_yaml = os.path.join(config_dir, 'ompl_planning.yaml')
    moveit_controllers_yaml = os.path.join(config_dir, 'moveit_controllers.yaml')
    pilz_cartesian_limits_yaml = os.path.join(config_dir, 'pilz_cartesian_limits.yaml')
    
    return LaunchDescription([
        # MoveIt move_group 노드
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'robot_description_semantic': robot_description_semantic},
                {'kinematics': kinematics_yaml},
                {'joint_limits': joint_limits_yaml},
                {'planning_pipelines': ['ompl', 'pilz_industrial_motion_planner']},
                {'ompl': ompl_planning_yaml},
                {'pilz_cartesian_limits': pilz_cartesian_limits_yaml},
                {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'},
                {'moveit_simple_controller_manager': moveit_controllers_yaml},
            ]
        ),
        
        # RViz2 실행 (시각화)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', os.path.join(config_dir, 'moveit.rviz')] if os.path.exists(os.path.join(config_dir, 'moveit.rviz')) else []
        ),
        
        # MoveIt Interface 노드 (우리가 만든 Python 인터페이스)
        Node(
            package='fruit_robot_project',
            executable='moveit_interface.py',
            name='moveit_interface_node',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])

