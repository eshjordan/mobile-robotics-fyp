from glob import glob
import os
from setuptools import find_packages, setup

package_name = "agent_local_comms_server"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jordan Esh",
    maintainer_email="esh.jordan@gmail.com",
    description="Controller package to manage local inter-agent communication via Wi-Fi",
    license="GPL-3.0-only",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "agent_local_comms_server = agent_local_comms_server.agent_local_comms_server:main",
            "test_robot = agent_local_comms_server.test_robot:main",
        ],
    },
)
