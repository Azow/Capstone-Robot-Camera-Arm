from setuptools import find_packages, setup
from glob import glob

package_name = 'arm_description_v2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', ['urdf/camera_arm_v2.urdf', 'urdf/scene.urdf']),
        ('share/' + package_name + '/meshes', glob('meshes/*.STL')),
        ('share/' + package_name + '/launch', ['launch/display.launch.py']),
        ('share/' + package_name + '/launch', ['launch/display.rviz'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zachw',
    maintainer_email='zachw@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_camera = arm_description_v2.fake_camera:main',
        ],
    },
)
