from glob import glob

from setuptools import find_packages, setup

package_name = 'camera_path_editor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zachw',
    maintainer_email='zachw@todo.todo',
    description='Camera path editor panel and visualization node.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_node = camera_path_editor.path_node:main',
            'path_panel = camera_path_editor.path_panel:main',
        ],
    },
)
