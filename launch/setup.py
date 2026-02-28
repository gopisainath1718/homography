from setuptools import setup

package_name = 'littleslam_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # Packages should be wrapped in quotes
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),  # Fix the concatenation
        ('share/' + package_name, ['launch/lidar_visualizer_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Advaith',
    maintainer_email='your_email@example.com',
    description='Lidar Visualization Node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_visualizer = littleslam_ros2.lidar_visualizer:main',
        ],
    },
)

