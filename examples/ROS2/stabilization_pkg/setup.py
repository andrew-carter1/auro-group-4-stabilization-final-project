from setuptools import find_packages, setup

package_name = 'stabilization_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/stabilization_launch.py']),
        ('share/' + package_name + '/models', ['stabilization_pkg/models/haarcascade_frontalface_default.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adria',
    maintainer_email='adriangalindoalvarez@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'stabilization_node = stabilization_pkg.realtime_stabilization:main',
        'face_detection_node = stabilization_pkg.face_detection_node:main',
    ],
},
)
