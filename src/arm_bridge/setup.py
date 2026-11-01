from glob import glob

from setuptools import find_packages, setup

package_name = 'arm_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zachw',
    maintainer_email='zachw@todo.todo',
    description='Serial link between ROS and the arm firmware, and a fake board.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_bridge = arm_bridge.bridge_node:main',
            'fake_board = arm_bridge.fake_board:main',
        ],
    },
)
