from glob import glob
from setuptools import setup

package_name = 'om6dof_pick_and_place_gemini'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config',
         glob('config/*.yaml') + glob('config/*.rviz')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='biancanobelia@gmail.com',
    description='GraspNet-style grasp detection with Gemini reasoning for OM6DOF.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gemini_pick_node = om6dof_pick_and_place_gemini.gemini_pick_node:main',
            'gemini_probe = om6dof_pick_and_place_gemini.gemini_client:main',
            'rgbd_viewer = om6dof_pick_and_place_gemini.rgbd_viewer:main',
            'target_grasp_viewer = '
            'om6dof_pick_and_place_gemini.target_grasp_viewer:main',
        ],
    },
)
