#!/usr/bin/env python3
"""간단한 과일 스폰 스크립트 - Gazebo에 바로 과일 생성"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import time

def main():
    rclpy.init()
    node = Node('simple_fruit_spawner')
    
    # Gazebo spawn 서비스 클라이언트
    spawn_client = node.create_client(SpawnEntity, '/spawn_entity')
    
    node.get_logger().info('Waiting for spawn_entity service...')
    while not spawn_client.wait_for_service(timeout_sec=2.0):
        node.get_logger().info('Service not available, waiting...')
    
    node.get_logger().info('Service available! Spawning fruits...')
    
    # 과일 3개 생성
    fruits = [
        {'name': 'orange', 'x': 0.3, 'y': 0.1, 'z': 0.5, 'color': '1 0.5 0 1', 'radius': 0.04, 'mass': 0.2},
        {'name': 'banana', 'x': 0.35, 'y': 0.15, 'z': 0.5, 'color': '1 1 0 1', 'radius': 0.05, 'mass': 0.15},
        {'name': 'guava', 'x': 0.25, 'y': 0.05, 'z': 0.5, 'color': '0 1 0 1', 'radius': 0.05, 'mass': 0.25},
    ]
    
    for i, fruit in enumerate(fruits):
        sdf = f"""
        <sdf version='1.6'>
          <model name='{fruit['name']}_{i}'>
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
                  <sphere><radius>{fruit['radius']}</radius></sphere>
                </geometry>
              </collision>
              <visual name='visual'>
                <geometry>
                  <sphere><radius>{fruit['radius']}</radius></sphere>
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
        req.name = f"{fruit['name']}_{i}"
        req.xml = sdf
        req.initial_pose.position.x = fruit['x']
        req.initial_pose.position.y = fruit['y']
        req.initial_pose.position.z = fruit['z']
        
        future = spawn_client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        
        if future.result() is not None and future.result().success:
            node.get_logger().info(f"✓ Spawned {fruit['name']}_{i}")
        else:
            node.get_logger().error(f"✗ Failed to spawn {fruit['name']}_{i}")
        
        time.sleep(0.5)
    
    node.get_logger().info('All fruits spawned!')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

