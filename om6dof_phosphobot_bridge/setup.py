from setuptools import setup

package_name = 'om6dof_phosphobot_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='biancanobelia@gmail.com',
    description='HTTP bridge exposing the OM6DOF arm to phosphobot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bridge_node = om6dof_phosphobot_bridge.bridge_node:main',
        ],
    },
)
