from setuptools import find_packages, setup

package_name = 'loon_e_coms'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/coms_launch.py']),
        ('share/' + package_name + '/config', ['config/coms.yaml']),
    ],
    install_requires=[
        'setuptools',
        'Pillow>=10.0.0',
        'paho-mqtt>=1.6.1',
        'PyYAML>=5.3',
    ],
    zip_safe=True,
    author='Carson Fujita',
    author_email='50027005+TheFujirose@users.noreply.github.com',
    maintainer='Carson Fujita',
    maintainer_email='50027005+TheFujirose@users.noreply.github.com',
    description='This node handles all inter-node communication within the ROS ecosystem and also interfaces with external systems via MQTT.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'coms = loon_e_coms.coms:main',
        ],
    },
)
