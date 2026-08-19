from setuptools import setup

package_name = "om6dof_gravity_comp"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/gravity_comp.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="unitree",
    maintainer_email="biancanobelia@gmail.com",
    description="Gravity torque model for the OM6DOF leader arm.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "gravity_comp_node = om6dof_gravity_comp.gravity_comp_node:main",
        ],
    },
)
