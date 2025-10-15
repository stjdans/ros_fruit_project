from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'fruit'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config/moveit_config'), glob('config/moveit_config/*.yaml')),
        (os.path.join('share', package_name, 'config/moveit_config'), glob('config/moveit_config/*.srdf')),
        (os.path.join('share', package_name, 'config/yolo_model'), glob('config/yolo_model/*.yaml')),
        (os.path.join('share', package_name, 'config/yolo_model'), glob('config/yolo_model/*.pt')),
        
        # Install models
        (os.path.join('share', package_name, 'models/sdf'), glob('models/sdf/*.sdf')),
        (os.path.join('share', package_name, 'models/fruits/banana'), glob('models/fruits/banana/*')),
        (os.path.join('share', package_name, 'models/fruits/orange'), glob('models/fruits/orange/*')),
        (os.path.join('share', package_name, 'models/basket'), glob('models/basket/*.obj')),
        (os.path.join('share', package_name, 'models/container'), glob('models/container/*')),
        (os.path.join('share', package_name, 'models/ceiling_camera'), glob('models/ceiling_camera/*')),

        # Install scripts
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=[
        'setuptools',
        'pyzmq>=22.0.0',
        'redis>=5.0.0',
        'websockets>=12.0',
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
        ],
    },
)
