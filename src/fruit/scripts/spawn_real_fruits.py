#!/usr/bin/env python3
"""실제 3D 모델 과일 스폰 스크립트"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import os
import time

def main():
    rclpy.init()
    node = Node('real_fruit_spawner')
    
    # 현재 워크스페이스 경로
    ws_path = os.path.expanduser('~/fruit_robot_project_ws')
    
    # Gazebo spawn 서비스 클라이언트
    spawn_client = node.create_client(SpawnEntity, '/spawn_entity')
    
    node.get_logger().info('Waiting for spawn_entity service...')
    while not spawn_client.wait_for_service(timeout_sec=2.0):
        node.get_logger().info('Service not available, waiting...')
    
    node.get_logger().info('Service available! Spawning real fruit models...')
    
    # 과일 3개 생성 - 절대 경로 사용
    fruits = [
        {
            'name': 'orange',
            'obj_path': f'{ws_path}/models/fruits/orange/Orange.obj',
            'mtl_path': f'{ws_path}/models/fruits/orange/Orange.mtl',
            'x': -0.5, 'y': -0.5, 'z': 0.0,
            'mass': 0.2,
            'scale': '0.02 0.02 0.02',
            'color': '1 0.5 0 1'
        },
        {
            'name': 'banana',
            'obj_path': f'{ws_path}/models/fruits/banana/Banana.obj',
            'mtl_path': f'{ws_path}/models/fruits/banana/Banana.mtl',
            'x': -0.5, 'y': -0.6, 'z': 0.0,
            'mass': 0.15,
            'scale': '0.02 0.02 0.02',
            'color': '1 1 0 1'
        },
        {
            'name': 'guava',
            'obj_path': f'{ws_path}/models/fruits/guava/Guava.obj',
            'mtl_path': f'{ws_path}/models/fruits/guava/Guava.mtl',
            'x': -0.5, 'y': -0.45, 'z': 0.0,
            'mass': 0.25,
            'scale': '0.02 0.02 0.02',
            'color': '0 1 0 1'
        },
    ]
    
    # 타임스탬프 추가로 이름 충돌 방지
    timestamp = int(time.time())
    
    for i, fruit in enumerate(fruits):
        # 파일 존재 확인
        if not os.path.exists(fruit['obj_path']):
            node.get_logger().error(f"OBJ file not found: {fruit['obj_path']}")
            continue
            
        node.get_logger().info(f"Loading {fruit['name']} from {fruit['obj_path']}")
        
        # 절대 경로로 SDF 생성
        sdf = f"""
        <sdf version='1.6'>
          <model name='{fruit['name']}_{timestamp}_{i}'>
            <link name='link'>
              <pose>0 0 0 0 0 0</pose>
              <inertial>
                <mass>{fruit['mass']}</mass>
                <inertia>
                  <ixx>0.001</ixx>
                  <iyy>0.001</iyy>
                  <izz>0.001</izz>
                </inertia>
              </inertial>
              <collision name='collision'>
                <geometry>
                  <mesh>
                    <uri>file://{fruit['obj_path']}</uri>
                    <scale>{fruit['scale']}</scale>
                  </mesh>
                </geometry>
              </collision>
              <visual name='visual'>
                <geometry>
                  <mesh>
                    <uri>file://{fruit['obj_path']}</uri>
                    <scale>{fruit['scale']}</scale>
                  </mesh>
                </geometry>
                <material>
                  <ambient>{fruit['color']}</ambient>
                  <diffuse>{fruit['color']}</diffuse>
                </material>
              </visual>
            </link>
          </model>
        </sdf>
        """
        
        req = SpawnEntity.Request()
        req.name = f"{fruit['name']}_{timestamp}_{i}"
        req.xml = sdf
        req.initial_pose.position.x = fruit['x']
        req.initial_pose.position.y = fruit['y']
        req.initial_pose.position.z = fruit['z']
        
        future = spawn_client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        
        if future.result() is not None and future.result().success:
            node.get_logger().info(f"✓ Spawned {fruit['name']}_{timestamp}_{i} with 3D model")
        else:
            error_msg = future.result().status_message if future.result() else "Timeout or no response"
            node.get_logger().error(f"✗ Failed to spawn {fruit['name']}_{timestamp}_{i}: {error_msg}")
    
    node.get_logger().info('All real fruit models spawned!')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

