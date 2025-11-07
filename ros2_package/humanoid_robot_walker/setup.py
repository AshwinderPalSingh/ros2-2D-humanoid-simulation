from setuptools import setup
import os
from glob import glob

package_name = 'humanoid_robot_walker'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AshwinderPalSingh',
    maintainer_email='',
    description='2D Humanoid robot walker with keyboard and coordinate control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = humanoid_robot_walker.robot_controller:main',
        ],
    },
)
