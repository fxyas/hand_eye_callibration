from setuptools import find_packages, setup

package_name = 'universal_handeye'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'config/handeye.yaml',
            'config/goal_publisher.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/handeye_calibration.launch.py',
            'launch/aruco_goal_publisher.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fayas',
    maintainer_email='fayaspallikkara@gmail.com',
    description='Universal ROS 2 hand-eye calibration system for robot-camera calibration',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'handeye_node = universal_handeye.main:main',
            'goal_publisher_node = universal_handeye.goal_pub:main',
            'goal_republisher_node = universal_handeye.quest_republisher:main',
        ],
    },
)
