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
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
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
            'mapping = loone.mapping:main',
            'path_planning = loone.path_planning:main',
            'motor = loone.motor:main',
        ],
    },
)
