from glob import glob

from setuptools import find_packages, setup

package_name = 'fifth_scale_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/camera_info', glob('config/camera_info/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/udev', glob('udev/*.rules')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hyeonjin Woo',
    maintainer_email='guswls264664@gmail.com',
    description='1/5 HENES T870 자율주행 통합 실행: 센서·카메라 설정, 센서 위치(extrinsics), 차량·리플레이 launch, RViz, 바퀴 주행 거리계, 차량 마커, bag 도구',
    license='AGPL-3.0-only',
    entry_points={
        'console_scripts': [
            'vehicle_marker = fifth_scale_bringup.vehicle_marker:main',
            'wheel_odometry = fifth_scale_bringup.wheel_odometry:main',
            'bag_retime = fifth_scale_bringup.bag_retime:main',
        ],
    },
)
