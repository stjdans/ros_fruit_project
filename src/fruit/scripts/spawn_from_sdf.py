#!/usr/bin/env python3
"""외부 model.sdf 파일을 사용하여 모델 스폰"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import os

def main():
    rclpy.init()
    node = Node('sdf_spawner')
    
    # Gazebo spawn 서비스 클라이언트 생성
    spawn_client = node.create_client(SpawnEntity, '/spawn_entity')
    
    # 서비스가 준비될 때까지 대기
    node.get_logger().info('Waiting for /spawn_entity service...')
    while not spawn_client.wait_for_service(timeout_sec=2.0):
        node.get_logger().info('Service not available, waiting...')
    
    node.get_logger().info('Service available!')
    
    # SDF 파일 경로
    workspace_path = os.path.expanduser('~/turtle_ws')
    sdf_file_path = os.path.join(
        workspace_path,
        'src/fruit/models/fruits/banana/model.sdf'
    )
    
    node.get_logger().info(f'Reading SDF file: {sdf_file_path}')
    
    # SDF 파일 읽기
    try:
        with open(sdf_file_path, 'r') as f:
            sdf_content = f.read()
        node.get_logger().info('✓ SDF file loaded successfully')
    except FileNotFoundError:
        node.get_logger().error(f'✗ SDF file not found: {sdf_file_path}')
        node.destroy_node()
        rclpy.shutdown()
        return
    except Exception as e:
        node.get_logger().error(f'✗ Error reading SDF file: {str(e)}')
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # Spawn 요청 생성
    req = SpawnEntity.Request()
    req.name = 'banana_model'
    req.xml = sdf_content
    req.initial_pose.position.x = 0.0
    req.initial_pose.position.y = 0.0
    req.initial_pose.position.z = 5.0
    req.initial_pose.orientation.w = 1.0
    
    node.get_logger().info(f'Spawning banana at ({req.initial_pose.position.x}, {req.initial_pose.position.y}, {req.initial_pose.position.z})')
    
    # 서비스 호출
    future = spawn_client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    
    # 결과 확인
    if future.result() is not None and future.result().success:
        node.get_logger().info('✓ Banana model spawned successfully!')
    else:
        error_msg = future.result().status_message if future.result() else "Timeout or no response"
        node.get_logger().error(f'✗ Failed to spawn model: {error_msg}')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
