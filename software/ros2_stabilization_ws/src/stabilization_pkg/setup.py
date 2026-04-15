import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'stabilization_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andre',
    maintainer_email='andrew.carter1@ufl.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gimbal_node = stabilization_pkg.gimbal_node:main',
            'rolling_shutter_node = stabilization_pkg.rolling_shutter_node:main',
        ],
    },
)
