#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from std_srvs.srv import Empty
import random
import time
import os
from ament_index_python.packages import get_package_share_directory

class FruitSpawner(Node):
    def __init__(self):
        super().__init__('fruit_spawner')

        # 패키지 경로 가져오기 (상대 경로)
        fruit_pkg = get_package_share_directory('fruit')
        models_path = os.path.join(fruit_pkg, 'models', 'fruits')
        self.get_logger().info(f'models_path : {models_path}')
        
        # Gazebo spawn 서비스 클라이언트
        self.spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')
        
        self.get_logger().info('Waiting for spawn_entity service...')
        while not self.spawn_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.get_logger().info('Service available! Spawning real fruit models...')
        
        # 과일 3개 생성 - 상대 경로 사용 (패키지 기반)
        self.fruits = [
            {
                'name': 'orange',
                'obj_path': os.path.join(models_path, 'orange', 'Orange.obj'),
                'mtl_path': os.path.join(models_path, 'orange', 'Orange.mtl'),
                'x': -1.4, 'y': -0.3, 'z': 0.5,
                'mass': 0.2,
                'scale': '0.1 0.1 0.1',
                'color': '1 0.5 0 1'
            },
            {
                'name': 'banana',
                'obj_path': os.path.join(models_path, 'banana', 'banana_v03.obj'),
                'mtl_path': os.path.join(models_path, 'banana', 'banana_v03.mtl'),
                'x': -1.5, 'y': -0.5, 'z': 0.5,
                'mass': 0.15,
                'scale': '0.005 0.005 0.005',
                'color': '1 1 0 1'
            },
            {
                'name': 'guava',
                'obj_path': os.path.join(models_path, 'guava', 'Guava.obj'),
                'mtl_path': os.path.join(models_path, 'guava', 'Guava.mtl'),
                'x': -1.4, 'y': -0.05, 'z': 0.5,
                'mass': 0.25,
                'scale': '0.1 0.1 0.1',
                'color': '0 1 0 1'
            },
        ]
        
        time.sleep(3)
        self.spawn_all_fruits()
        
        
    def spawn_all_fruits(self):
        """모든 과일 스폰 (한 번만 실행)"""
        
        # 타임스탬프 추가로 이름 충돌 방지
        timestamp = int(time.time())
        
        for i, fruit in enumerate(self.fruits):
            # 파일 존재 확인
            if not os.path.exists(fruit['obj_path']):
                self.get_logger().error(f"OBJ file not found: {fruit['obj_path']}")
                continue
                
            self.get_logger().info(f"Loading {fruit['name']} from {fruit['obj_path']}")
            
            # SDF 생성
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
                   <!-- 감쇠 추가 (낮은 값으로 자연스러운 낙하) -->
                  <velocity_decay>
                    <linear>0.05</linear>
                    <angular>0.5</angular>
                  </velocity_decay>
                  <collision name='collision'>
                    <geometry>
                      <mesh>
                        <uri>file://{fruit['obj_path']}</uri>
                        <scale>{fruit['scale']}</scale>
                      </mesh>
                    </geometry>
                  
                    <!-- 마찰력 추가 -->
                    <surface>
                      <friction>
                        <ode>
                          <mu>100.0</mu>   <!-- 높은 마찰 계수 -->
                          <mu2>100.0</mu2>
                        </ode>
                      </friction>
                    </surface>
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
                    # <material>
                    #   <ambient>{fruit['color']}</ambient>
                    #   <diffuse>{fruit['color']}</diffuse>
                    # </material>
            
            
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

def main(args=None):
    rclpy.init(args=args)
    node = FruitSpawner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
