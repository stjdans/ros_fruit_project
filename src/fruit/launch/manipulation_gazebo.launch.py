#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch file to run turtlebot3_manipulation_gazebo gazebo.launch.py
    """
    
    # Get fruit package path and set GAZEBO_MODEL_PATH
    fruit_pkg = get_package_share_directory('fruit')
    models_path = os.path.join(fruit_pkg, 'models')
    
    # Get existing GAZEBO_MODEL_PATH and append our models path
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if existing_model_path:
        gazebo_model_path = f"{models_path}:{existing_model_path}"
    else:
        gazebo_model_path = models_path
    
    # Set GAZEBO_MODEL_PATH environment variable
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=gazebo_model_path
    )
    
    # Declare launch arguments that can be passed to the gazebo.launch.py
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='false',
        description='Whether to execute rviz2'
    )
    
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='""',
        description='Prefix of the joint and link names'
    )
    
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Start robot in Gazebo simulation'
    )
    
    x_pose_arg = DeclareLaunchArgument(
        'x_pose',
        default_value='-2.00',
        description='Initial x position of turtlebot3'
    )
    
    y_pose_arg = DeclareLaunchArgument(
        'y_pose',
        default_value='-0.50',
        description='Initial y position of turtlebot3'
    )
    
    z_pose_arg = DeclareLaunchArgument(
        'z_pose',
        default_value='0.01',
        description='Initial z position of turtlebot3'
    )
    
    roll_arg = DeclareLaunchArgument(
        'roll',
        default_value='0.00',
        description='Initial roll orientation of turtlebot3'
    )
    
    pitch_arg = DeclareLaunchArgument(
        'pitch',
        default_value='0.00',
        description='Initial pitch orientation of turtlebot3'
    )
    
    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.00',
        description='Initial yaw orientation of turtlebot3'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            FindPackageShare('turtlebot3_gazebo'),
            'worlds',
            'empty_world.world'
        ]),
        description='Path to the Gazebo world file'
    )
    
    # Include the turtlebot3_manipulation_gazebo gazebo.launch.py
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_manipulation_gazebo'),
                'launch',
                'gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'start_rviz': LaunchConfiguration('start_rviz'),
            'prefix': LaunchConfiguration('prefix'),
            'use_sim': LaunchConfiguration('use_sim'),
            'world': LaunchConfiguration('world'),
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'z_pose': LaunchConfiguration('z_pose'),
            'roll': LaunchConfiguration('roll'),
            'pitch': LaunchConfiguration('pitch'),
            'yaw': LaunchConfiguration('yaw'),
        }.items(),
    )
    
    # Fruit spawner node
    fruit_spawner_node = Node(
        package='fruit',
        executable='fruit_spawner',
        name='fruit_spawner',
        output='screen'
    )
    
    # Delay fruit spawner execution by 5 seconds
    delayed_fruit_spawner = TimerAction(
        period=5.0,
        actions=[fruit_spawner_node]
    )
    
    # Include ceiling camera launch
    ceiling_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('fruit'),
                'launch',
                'ceiling_camera.launch.py'
            ])
        )
    )
    
    # Delay ceiling camera launch execution by 5 seconds
    delayed_ceiling_camera = TimerAction(
        period=5.0,
        actions=[ceiling_camera_launch]
    )
    
    # Camera streamer zeromq node
    camera_streamer_zeromq_node = Node(
        package='fruit',
        executable='camera_streamer_zeromq',
        name='camera_streamer_zeromq',
        output='screen'
    )
    
    # Delay camera streamer zeromq execution by 10 seconds
    delayed_camera_streamer = TimerAction(
        period=10.0,
        actions=[camera_streamer_zeromq_node]
    )
    
    # Parameter server node (REST API for web control)
    parameter_server_node = Node(
        package='fruit',
        executable='parameter_server',
        name='parameter_server',
        output='screen'
    )
    
    # Delay parameter server execution by 10 seconds
    delayed_parameter_server = TimerAction(
        period=10.0,
        actions=[parameter_server_node]
    )
    
    # Robot control server node (Main robot control API)
    robot_control_server_node = Node(
        package='fruit',
        executable='robot_control_server',
        name='robot_control_server',
        output='screen'
    )
    
    # Delay robot control server execution by 10 seconds
    delayed_robot_control_server = TimerAction(
        period=10.0,
        actions=[robot_control_server_node]
    )
    
    return LaunchDescription([
        # Set environment variable first
        set_gazebo_model_path,
        
        # Launch arguments
        start_rviz_arg,
        prefix_arg,
        use_sim_arg,
        world_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        
        # Launch files and nodes
        gazebo_launch,
        delayed_fruit_spawner,
        delayed_ceiling_camera,
        delayed_camera_streamer,
        delayed_parameter_server,
        delayed_robot_control_server,
    ])

