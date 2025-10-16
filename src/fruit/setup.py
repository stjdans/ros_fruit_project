from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'fruit'

def get_data_files(directory):
    """재귀적으로 디렉토리의 모든 파일을 data_files 형식으로 반환"""
    data_files = []
    for root, dirs, files in os.walk(directory):
        if files:
            # 상대 경로로 변환
            rel_root = os.path.relpath(root)
            # share/package_name/... 형식으로 목적지 경로 생성
            dest_path = os.path.join('share', package_name, rel_root)
            # 해당 디렉토리의 모든 파일 경로 리스트
            file_paths = [os.path.join(root, f) for f in files]
            data_files.append((dest_path, file_paths))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] 
    + get_data_files('launch')
    + get_data_files('scripts') 
    + get_data_files('config') 
    + get_data_files('models'),
    install_requires=[
        'setuptools',
        'pyzmq>=22.0.0',
        'redis>=5.0.0',
        'websockets>=12.0',
        'flask>=2.0.0',
        'flask-cors>=3.0.0',
    ],
    zip_safe=True,
    maintainer='ssm',
    maintainer_email='ssm@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Perception nodes
            'yolo_detector = fruit.perception.yolo_detector:main',
            'camera_node = fruit.perception.camera_node:main',
            
            # Camera nodes
            'camera_viewer = fruit.camera.camera_viewer:main',
            'ceiling_camera_viewer = fruit.camera.ceiling_camera_viewer:main',
            'camera_streamer = fruit.camera.camera_streamer:main',
            'camera_streamer_websocket = fruit.camera.camera_streamer_websocket:main',
            'camera_streamer_gstreamer = fruit.camera.camera_streamer_gstreamer:main',
            'camera_streamer_redis = fruit.camera.camera_streamer_redis:main',
            'camera_streamer_zeromq = fruit.camera.camera_streamer_zeromq:main',
            
            # Manipulation nodes
            'moveit_interface = fruit.manipulation.moveit_interface:main',
            'pick_and_place = fruit.manipulation.pick_and_place:main',
            'gripper_control = fruit.manipulation.gripper_control:main',
            'trajectory_controller = fruit.manipulation.trajectory_controller:main',
            
            # Spawner
            'fruit_spawner = fruit.spawner.fruit_spawner:main',
            'spawn_from_sdf = fruit.spawner.spawn_from_sdf:main',
            
            # Weighing
            'price_calculator = fruit.weighing.price_calculator:main',
            
            # Orders
            'order_manager = fruit.order.order_manager:main',
            
            # Dashboard
            'dashboard = fruit.dashboard.app:main',
            
            # API Server
            'parameter_server = fruit.api.parameter_server:main',
        ],
    },
)
