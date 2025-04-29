from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'agent_interactions'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lachlan Richards',
    maintainer_email='lric0018@student.monash.edu',
    description='TODO: Package description',
    license='GPL-3.0-only',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agent_interactions = agent_interactions.schemes:main',
            # 'test = waypoint_controller.test:main'
        ],
    },
)
