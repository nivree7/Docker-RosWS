from setuptools import setup
from glob import glob
import os


package_name = 'camera_gated_clustering'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'launch/components'), glob('launch/components/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
    (os.path.join('share', package_name, 'config'), glob('config/*.xml')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeff',
    maintainer_email='jaychang0501@gmail.com',
    description='Camera-gated LiDAR clustering fusion node (Python).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_gated_clustering_node = camera_gated_clustering.camera_gated_clustering_node:main',
            'lidar_camera_fusion_node = camera_gated_clustering.lidar_camera_fusion_node:main',
        ],
    },
)
