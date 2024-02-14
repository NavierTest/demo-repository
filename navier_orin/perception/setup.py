from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='navier_orin',
    maintainer_email='navier_orin@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'projectLidarOnImage = perception.project_LiDAR_point_cloud_onto_2D_image:main',
            'timestampTest = perception.timestamp_test:main',
            'bBox_info = perception.bBox_info:main',
            'bBox_test = perception.project_LiDAR_test_file:main'
        ],
    },
)