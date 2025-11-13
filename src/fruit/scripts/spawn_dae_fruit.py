#!/usr/bin/env python3
"""
DAE 모델 과일 스폰 스크립트
- fruits.dae 파일을 사용하여 Gazebo 환경에 과일 모델을 생성합니다.
- ROS2 Gazebo spawn_entity 서비스를 사용합니다.
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import os
import time
from ament_index_python.packages import get_package_share_directory

def main():
    # ROS2 초기화
    rclpy.init()
    
    # ROS2 노드 생성 (노드 이름: dae_fruit_spawner)
    node = Node('dae_fruit_spawner')
    
    # 패키지 경로 가져오기 (상대 경로)
    fruit_pkg = get_package_share_directory('fruit')
    dae_path = os.path.join(fruit_pkg, 'models', 'fruits', 'fruits.dae')
    
    # DAE 파일이 실제로 존재하는지 확인
    # 파일이 없으면 에러 메시지 출력 후 종료
    if not os.path.exists(dae_path):
        node.get_logger().error(f"DAE file not found: {dae_path}")
        return
    
    node.get_logger().info(f"Found DAE file: {dae_path}")
    
    # Gazebo의 spawn_entity 서비스 클라이언트 생성
    # 이 서비스를 통해 Gazebo 시뮬레이션에 새로운 모델을 추가할 수 있습니다.
    spawn_client = node.create_client(SpawnEntity, '/spawn_entity')
    
    # 서비스가 준비될 때까지 대기
    node.get_logger().info('Waiting for spawn_entity service...')
    while not spawn_client.wait_for_service(timeout_sec=2.0):
        node.get_logger().info('Service not available, waiting...')
    
    node.get_logger().info('Service available! Spawning DAE fruit model...')
    
    # 현재 시간을 타임스탬프로 사용하여 모델 이름 충돌 방지
    # 같은 이름의 모델이 이미 존재하면 스폰이 실패하므로 고유한 이름 생성
    timestamp = int(time.time())
    
    # 스폰할 과일들의 위치와 크기 정의
    # x, y, z: Gazebo 월드 좌표계에서의 위치 (단위: 미터)
    # scale: 모델의 크기 비율 (0.02 = 원본 크기의 2%)
    fruits = [
        {'x': 0.3, 'y': 0.0, 'z': 0.5, 'scale': 0.02},    # 첫 번째 과일
        {'x': 0.4, 'y': 0.1, 'z': 0.5, 'scale': 0.02},    # 두 번째 과일
        {'x': 0.35, 'y': -0.1, 'z': 0.5, 'scale': 0.02},  # 세 번째 과일
    ]
    
    # 각 과일을 순차적으로 스폰
    for i, fruit in enumerate(fruits):
        # SDF (Simulation Description Format) 형식으로 모델 정의
        # SDF는 Gazebo에서 로봇과 환경을 정의하는 XML 기반 포맷입니다.
        sdf = f"""
        <sdf version='1.6'>
          <model name='dae_fruit_{timestamp}_{i}'>
            <link name='link'>
              <!-- pose: x y z roll pitch yaw (위치와 방향) -->
              <pose>0 0 0 0 0 0</pose>
              
              <!-- 물리 속성 정의 -->
              <inertial>
                <mass>0.2</mass>  <!-- 질량: 0.2kg (200g) -->
                <inertia>
                  <!-- 관성 모멘트 (물체의 회전 저항) -->
                  <ixx>0.001</ixx>
                  <iyy>0.001</iyy>
                  <izz>0.001</izz>
                </inertia>
              </inertial>
              
              <!-- 충돌 영역 정의 (물리 시뮬레이션용) -->
              <collision name='collision'>
                <geometry>
                  <mesh>
                    <uri>file://{dae_path}</uri>  <!-- DAE 파일 경로 -->
                    <scale>{fruit['scale']} {fruit['scale']} {fruit['scale']}</scale>  <!-- 크기 조절 -->
                  </mesh>
                </geometry>
              </collision>
              
              <!-- 시각적 표현 정의 (렌더링용) -->
              <visual name='visual'>
                <geometry>
                  <mesh>
                    <uri>file://{dae_path}</uri>  <!-- DAE 파일 경로 -->
                    <scale>{fruit['scale']} {fruit['scale']} {fruit['scale']}</scale>  <!-- 크기 조절 -->
                  </mesh>
                </geometry>
              </visual>
            </link>
          </model>
        </sdf>
        """
        
        # SpawnEntity 서비스 요청 생성
        req = SpawnEntity.Request()
        req.name = f"dae_fruit_{timestamp}_{i}"  # 모델의 고유 이름
        req.xml = sdf  # 위에서 정의한 SDF XML 문자열
        
        # 초기 위치 설정 (Gazebo 월드 좌표계 기준)
        req.initial_pose.position.x = fruit['x']
        req.initial_pose.position.y = fruit['y']
        req.initial_pose.position.z = fruit['z']
        
        # 비동기 서비스 호출 (서비스 응답을 기다리지 않고 다음 코드 실행)
        future = spawn_client.call_async(req)
        
        # 서비스 응답이 올 때까지 대기 (최대 5초)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        
        # 결과 확인 및 로그 출력
        if future.result() is not None and future.result().success:
            node.get_logger().info(f"✓ Spawned dae_fruit_{timestamp}_{i} with DAE model")
        else:
            # 실패 시 에러 메시지 출력
            error_msg = future.result().status_message if future.result() else "Timeout or no response"
            node.get_logger().error(f"✗ Failed to spawn dae_fruit_{timestamp}_{i}: {error_msg}")
        
        # 다음 과일 스폰 전 0.5초 대기 (Gazebo 안정화를 위함)
        time.sleep(0.5)
    
    # 모든 과일 스폰 완료
    node.get_logger().info('All DAE fruit models spawned!')
    
    # ROS2 노드 정리 및 종료
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

