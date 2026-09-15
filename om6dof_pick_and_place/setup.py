from glob import glob
from setuptools import setup

package_name = 'om6dof_pick_and_place'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml') + glob('config/*.rviz')),
        ('share/' + package_name + '/systemd', glob('systemd/*.service')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='unitree',
    maintainer_email='biancanobelia@gmail.com',
    description='Pick-and-place state machine for OM6DOF via MoveGroup action.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pick_place_node = om6dof_pick_and_place.pick_place_node:main',
            'apriltag_detector = om6dof_pick_and_place.apriltag_detector:main',
            'tag_pick_place_node = om6dof_pick_and_place.tag_pick_place_node:main',
            'qr_follower_node = om6dof_pick_and_place.qr_follower_node:main',
            'coordinate_debug_node = om6dof_pick_and_place.coordinate_debug_node:main',
            'tag_calibrate = om6dof_pick_and_place.tag_calibrate:main',
            'ddgng_hand_eye_calibrate = om6dof_pick_and_place.hand_eye_calibrate:main',
            'center_depth_urdf_check = om6dof_pick_and_place.center_depth_urdf_check:main',
            'calib_gui = om6dof_pick_and_place.calib_gui:main',
            'direct_pick_node = om6dof_pick_and_place.direct_pick_node:main',
            'graph_pick_node = om6dof_pick_and_place.graph_pick_node:main',
            'boxer_pick_node = om6dof_pick_and_place.boxer_pick_node:main',
            'boxer_pick_gui = om6dof_pick_and_place.boxer_pick_gui:main',
        ],
    },
)
