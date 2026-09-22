import os
from glob import glob
from setuptools import find_packages, setup

package_name = "swarm_stack"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=True,
    maintainer="your_name",
    maintainer_email="you@example.com",
    description="Comms sim, mission/role management, fault injection and metrics logging.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "comms_sim_node = swarm_stack.comms_sim_node:main",
            "mission_manager_node = swarm_stack.mission_manager_node:main",
            "metrics_logger_node = swarm_stack.metrics_logger_node:main",
            "inject_fault = swarm_stack.inject_fault:main",
        ],
    },
)
