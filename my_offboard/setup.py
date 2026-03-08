from setuptools import setup
import os

package_name = 'my_offboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ROS2 패키지 인덱스에 설치 마커 추가
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml 설치
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ljh',
    maintainer_email='ljh@example.com',
    description='Custom offboard control node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_offboard_control = my_offboard.my_offboard_control:main',
            'joy_drone_control = my_offboard.joy_drone_control:main',
            'kml_offboard = my_offboard.kml_offboard:main',
            'input_waypoint = my_offboard.input_waypoint:main',
        ],
    },
)

