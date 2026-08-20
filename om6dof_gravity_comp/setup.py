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
            "calibrate = om6dof_gravity_comp.calibrate:main",
            "push_test = om6dof_gravity_comp.push_test:main",
            "identification_logger = om6dof_gravity_comp.logger:main",
            "excitation = om6dof_gravity_comp.excitation:main",
            "static_sweep = om6dof_gravity_comp.static_sweep:main",
            "identify = om6dof_gravity_comp.identify:main",
            "identify_b = om6dof_gravity_comp.identify_b:main",
            "identify_static = om6dof_gravity_comp.identify_static:main",
            "joint_check = om6dof_gravity_comp.joint_check:main",
            "payload_check = om6dof_gravity_comp.payload_check:main",
            "evaluate = om6dof_gravity_comp.evaluate:main",
            "current_estimator = om6dof_gravity_comp.estimator:main",
            "gravity_compensation = om6dof_gravity_comp.compensation:main",
        ],
    },
)
