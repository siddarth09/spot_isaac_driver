from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'spot_isaac_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # --- standard package index registration ---
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # --- include launch and policy directories ---
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'policy'), glob('policy/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='siddarth.dayasagar@gmail.com',
    description='Spot RL policy ROS2 driver for Isaac Sim integration',
    license='Apache License 2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # Make sure your Python file has a main() function
            'spot_controller = spot_isaac_driver.spot_full_body_controller:main',
        ],
    },
)
