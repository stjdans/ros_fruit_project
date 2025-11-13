#!/usr/bin/env python3
# launch/full_system.launch.py
# 전체 과일 로봇 시스템 통합 실행 파일

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 현재 워크스페이스 경로
    workspace_dir = os.getcwd()
    launch_dir = os.path.join(workspace_dir, 'launch')
    
    return LaunchDescription([
        # 1. Gazebo 월드 실행 (로봇, 저울, 바스켓, 컨테이너)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'gazebo_world.launch.py')
            )
        ),
        
        # 2. MoveIt 경로 계획 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'moveit_planning.launch.py')
            )
        ),
        
        # 3. 과일 스폰 노드
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'fruit_spawner.launch.py')
            )
        ),
        
        # 4. YOLO 감지 노드
        Node(
            package='fruit_robot_project',
            executable='yolo_detector.py',
            name='yolo_detector',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'model_path': os.path.join(workspace_dir, 'config/yolo_model/mixed_fruits.pt')},
                {'classes_path': os.path.join(workspace_dir, 'config/yolo_model/classes.yaml')}
            ]
        ),
        
        # 5. Pick & Place 노드
        Node(
            package='fruit_robot_project',
            executable='pick_and_place.py',
            name='pick_and_place_node',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'targets_config': os.path.join(workspace_dir, 'config/targets.yaml')}
            ]
        ),
        
        # 6. 가격 계산 노드
        Node(
            package='fruit_robot_project',
            executable='price_calculator.py',
            name='price_calculator',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'price_table': os.path.join(workspace_dir, 'config/price_table.yaml')}
            ]
        ),
        
        # 7. 주문 관리 노드
        Node(
            package='fruit_robot_project',
            executable='order_manager.py',
            name='order_manager',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        
        # 8. Dashboard (Flask 웹 서버)
        Node(
            package='fruit_robot_project',
            executable='app.py',
            name='dashboard',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])

