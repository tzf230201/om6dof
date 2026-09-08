from glob import glob

from setuptools import setup

package_name = "om6dof_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["CHANGELOG.rst", "README.md", "package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/systemd", glob("systemd/*.service")),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="unitree",
    maintainer_email="biancanobelia@gmail.com",
    description="Selectable teleoperation input for OM6DOF.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "teleop_node = om6dof_teleop.teleop_node:main",
        ],
    },
)
