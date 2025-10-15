#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from std_srvs.srv import Empty
import random
import time
import os

class FruitSpawner(Node):
    def __init__(self):
        super().__init__('fruit_spawner')

        # 현재 워크스페이스 경로
        self.ws_path = os.path.expanduser('~/turtle_ws')
        self.get_logger().info(f'ws_path : {self.ws_path}')
        
        # Gazebo spawn 서비스 클라이언트
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        
        self.get_logger().info('Waiting for spawn_entity service...')
        while not self.spawn_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.get_logger().info('Service available! Spawning real fruit models...')
        
        # 과일 3개 생성 - 절대 경로 사용
        self.fruits = [
            {
                'name': 'orange',
                'obj_path': f'{self.ws_path}/src/fruit/models/fruits/orange/Orange.obj',
                'mtl_path': f'{self.ws_path}/src/fruit/models/fruits/orange/Orange.mtl',
                'x': -1.5, 'y': -0.5, 'z': 0.1,
                'mass': 0.2,
                'scale': '0.02 0.02 0.02',
                'color': '1 0.5 0 1'
            },
            {
                'name': 'banana',
                'obj_path': f'{self.ws_path}/src/fruit/models/fruits/banana/banana_v03.obj',
                'mtl_path': f'{self.ws_path}/src/fruit/models/fruits/banana/banana_v03.mtl',
                'x': -1.5, 'y': -0.6, 'z': 0.1,
                'mass': 0.15,
                'scale': '0.02 0.02 0.02',
                'color': '1 1 0 1'
            },
            {
                'name': 'guava',
                'obj_path': f'{self.ws_path}/src/fruit/models/fruits/guava/Guava.obj',
                'mtl_path': f'{self.ws_path}/src/fruit/models/fruits/guava/Guava.mtl',
                'x': -1.5, 'y': -0.45, 'z': 0.1,
                'mass': 0.25,
                'scale': '0.02 0.02 0.02',
                'color': '0 1 0 1'
            },
        ]
        
        # # 초기 스폰 (3초 후 한 번만)
        self.timer = self.create_timer(3.0, self.spawn_all_fruits)
        self.spawned = False
        
    def spawn_all_fruits(self):
        """모든 과일 스폰 (한 번만 실행)"""
        if self.spawned:
            return
        
        # self.spawned = True
        
        # 타임스탬프 추가로 이름 충돌 방지
        timestamp = int(time.time())
        
        for i, fruit in enumerate(self.fruits):
            # 파일 존재 확인
            if not os.path.exists(fruit['obj_path']):
                self.get_logger().error(f"OBJ file not found: {fruit['obj_path']}")
                continue
                
            self.get_logger().info(f"Loading {fruit['name']} from {fruit['obj_path']}")
            
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
            
            future = self.spawn_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None and future.result().success:
                self.get_logger().info(f"✓ Spawned {fruit['name']}_{timestamp}_{i} with 3D model")
            else:
                error_msg = future.result().status_message if future.result() else "Timeout or no response"
                self.get_logger().error(f"✗ Failed to spawn {fruit['name']}_{timestamp}_{i}: {error_msg}")
        
        # # 타이머 취소
        # self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = FruitSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
