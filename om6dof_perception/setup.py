from glob import glob

from setuptools import setup

package_name = "om6dof_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md", "requirements.txt"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/docs", glob("docs/*.md")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
        ("lib/" + package_name, ["scripts/boxer_adapter.py",
                                "scripts/barath_boxer3d_adapter.py",
                                "scripts/boxer_pick_geometry.py"]),
        ("share/" + package_name + "/systemd", glob("systemd/*.service")),
        ("share/" + package_name + "/sudoers", glob("sudoers/*")),
    ],
    install_requires=["setuptools"],
    scripts=["scripts/boxer_preview.py", "scripts/barath_boxer3d_preview.py",
             "scripts/yolo3d_pickup_bridge.py", "event_camera_simulator.py",
             "realsense_rgb_viewer.py"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="unitree",
    maintainer_email="biancanobelia@gmail.com",
    description="OM6DOF RGB-D perception, BOXER, and camera utilities",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "perception_node = om6dof_perception.perception_node:main",
            "perception_view = om6dof_perception.perception_view:main",
            "world_projection_node = om6dof_perception.world_projection:main",
        ],
    },
)
