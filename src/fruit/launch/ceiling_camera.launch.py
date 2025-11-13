#!/usr/bin/env python3
"""
Launch file for spawning a ceiling camera in Gazebo
천장 카메라를 Gazebo에 생성하는 런치 파일
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 패키지 경로
    fruit_pkg = get_package_share_directory('fruit')
    
    # 카메라 모델 경로
    camera_model_path = os.path.join(
        fruit_pkg, 'models', 'ceiling_camera', 'model.sdf'
    )
    
    # 카메라 위치 설정 (x, y, z) - 천장에 위치
    # z값을 높게 설정하여 천장에서 내려다보는 뷰
    camera_x = '0.0'  # 중앙
    camera_y = '0.0'  # 중앙
    camera_z = '1.0'  # 바닥에서 3m 높이
    
    # 카메라 회전 (roll, pitch, yaw)
    # pitch를 -1.57 (약 -90도)로 설정하여 아래를 향하도록
    camera_roll = '0.0'
    camera_pitch = '0.5'  # -90도 (아래를 향함)
    camera_yaw = '-3.0'
    
    return LaunchDescription([
        # 천장 카메라 spawn
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-entity', 'ceiling_camera',
                '-file', camera_model_path,
                '-x', camera_x,
                '-y', camera_y,
                '-z', camera_z,
                '-R', camera_roll,
                '-P', camera_pitch,
                '-Y', camera_yaw
            ],
            output='screen',
            shell=False
        ),
              # 3. 5초 대기 후 카메라 뷰어 실행 (카메라 spawn 후 대기)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='fruit',
                    executable='ceiling_camera_viewer',
                    name='ceiling_camera_viewer',
                    output='screen',
                    parameters=[{
                        'camera_topic': '/ceiling/ceiling_camera/image_raw'
                    }]
                ),
            ]
        ),
    ])
