from setuptools import setup
import os
from glob import glob

package_name = 'multi_turtles'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jose Santos',
    maintainer_email='jmsantos@espol.edu.ec',
    description='Multiple turtlesim nodes launcher',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_movements = multi_turtles.turtle_movements:main'
        ],
    },
)