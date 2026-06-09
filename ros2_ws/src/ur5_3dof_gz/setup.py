import os
from glob import glob
from setuptools import setup

PACKAGE_NAME = "ur5_3dof_gz"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=[PACKAGE_NAME],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + PACKAGE_NAME]),
        ("share/" + PACKAGE_NAME, ["package.xml"]),
        (os.path.join("share", PACKAGE_NAME, "launch"),  glob("launch/*.launch.py")),
        (os.path.join("share", PACKAGE_NAME, "urdf"),    glob("urdf/*.xacro")),
        (os.path.join("share", PACKAGE_NAME, "config"),  glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Itai Groisman",
    maintainer_email="itaigro@gmail.com",
    description="Gazebo Harmonic simulation of the 3-DOF UR5 with PID and CTC control.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "torque_bridge   = ur5_3dof_gz.torque_bridge_node:main",
            "ur5_pid_node    = ur5_3dof_gz.pid_node:main",
            "ur5_ctc_node    = ur5_3dof_gz.ctc_node:main",
            "ur5_logger_node = ur5_3dof_gz.logger_node:main",
        ],
    },
)
