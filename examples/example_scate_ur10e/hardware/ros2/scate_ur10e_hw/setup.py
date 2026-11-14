"""SCATE hardware ROS 2 package. The executor logic itself (scate_hw) stays in
examples/example_scate_ur10e/hardware and is imported from the GTDynamics
checkout at run time (parameter scate_hw_path / gtdynamics), so the laptop
simulation and the lab run share one copy."""

from setuptools import setup

package_name = 'scate_ur10e_hw'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/scate_hardware.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Harrison Jenkins',
    maintainer_email='hjj225@lehigh.edu',
    description='SCATE hardware runs on the UR10e',
    license='BSD',
    entry_points={
        'console_scripts': [
            'ur_bridge = scate_ur10e_hw.ur_bridge:main',
            'fake_ur = scate_ur10e_hw.fake_ur:main',
            'scate_executor = scate_ur10e_hw.executor_node:main',
        ],
    },
)
