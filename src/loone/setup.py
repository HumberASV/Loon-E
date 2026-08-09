import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'loone'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.json')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'model'), glob('model/*.onnx')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='humberasv',
    maintainer_email='mechatronicsclub@humber.ca',
    description='Loon-E operation',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'phone = loone.phone:main',
            'task = loone.task:main',
            'path_planning = loone.path_planning:main',
            'motor = loone.motor:main',
            'battery_node = loone.battery_node:main',
        ],
    },
)
