from setuptools import setup

package_name = 'thruster_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='navier_orin',
    maintainer_email='navier@usn.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rcSubscriber = thruster_control.rcInput:main',
            'rcAllocateThrustPublisher = thruster_control.allocate_thrust:main'
        ],
    },
)