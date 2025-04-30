from setuptools import find_packages, setup

package_name = 'sim_diff_drive_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fectec',
    maintainer_email='fectec151@gmail.com',
    description='This package implements control algorithms for the CoppeliaSim differential drive simulation robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_point_controller = sim_diff_drive_control.pid_point_controller:main',
            'odometry_localization = sim_diff_drive_control.odometry_localization:main'
        ],
    },
)