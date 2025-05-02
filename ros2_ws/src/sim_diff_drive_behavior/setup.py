from setuptools import find_packages, setup

package_name = 'sim_diff_drive_behavior'

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
    description='Behavior descriptions for the simulated differential-drive robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'local_trajectory_sphere_fsm = sim_diff_drive_behavior.local_trajectory_sphere_fsm:main',
            'sphere_color_orchestrator = sim_diff_drive_behavior.sphere_color_orchestrator:main',
        ],
    },
)