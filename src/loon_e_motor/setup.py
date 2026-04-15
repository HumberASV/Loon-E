from setuptools import find_packages, setup

package_name = 'loon_e_motor'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Amelia Soon',
    author_email='amelia.soon@hotmail.com',
    maintainer='Carson Fujita',
    maintainer_email='50027005+TheFujirose@users.noreply.github.com',    
    description='Motor control package for Loon-E',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_control = loon_e_motor.motor_control:main',
        ],
    },
)
