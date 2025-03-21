from setuptools import find_packages, setup
from glob import glob

package_name = 'ros2_dronecan'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name, glob("launch/*launch.[pxy][yma]*")),
        ('share/' + package_name + '/dsdl/com/zxdynamics/controller', glob("dsdl/**/*.uavcan", recursive=True)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='belovictor',
    maintainer_email='belovictor@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ros2_dronecan_node = ros2_dronecan.ros2_dronecan_node:main",
        ],
    },
)
