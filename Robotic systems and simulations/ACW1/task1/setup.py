from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'task1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share/' + package_name), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wiktor',
    maintainer_email='wiktor@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'prey_node = acw1.prey_node:main',
            'predator_node = acw1.predator_node:main',
            'spawner = acw1.spawner:main'
        ],
    },
)
