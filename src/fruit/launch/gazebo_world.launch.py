#!/usr/bin/env python3
# launch/gazebo_world.launch.py
# Gazebo 월드와 OpenManipulator-X 로봇 실행

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Gazebo 월드 파일 경로 (기본 empty world 사용)
    world_file = os.path.join(get_package_share_directory('gazebo_ros'), 'worlds', 'empty.world')
    
    return LaunchDescription([
        # Gazebo 서버 및 GUI 실행
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        
        # 로봇 상태 퍼블리셔 (joint_state -> tf)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # 저울 플러그인 로드 (Gazebo에 저울 모델 추가)
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-entity', 'scale',
                 '-file', os.path.join(os.getcwd(), 'models/scale/scale.sdf'),
                 '-x', '0.0', '-y', '0.3', '-z', '0.0'],
            output='screen'
        ),
        
        # 바스켓 스폰
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-entity', 'basket',
                 '-file', os.path.join(os.getcwd(), 'models/basket/basket.sdf'),
                 '-x', '0.3', '-y', '-0.2', '-z', '0.0'],
            output='screen'
        ),
        
        # 컨테이너 스폰
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-entity', 'container',
                 '-file', os.path.join(os.getcwd(), 'models/container/container.sdf'),
                 '-x', '-0.1', '-y', '0.3', '-z', '0.0'],
            output='screen'
        ),
    ])

