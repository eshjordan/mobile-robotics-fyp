from setuptools import find_packages, setup
import os
from glob import glob

package_name = "waypoint_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            os.path.join("share", "ament_index", "resource_index", "packages"),
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="atoo0005@student.monash.edu",
    description="TODO: Package description",
    license="GPL-3.0-only",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["controller = waypoint_controller.controller:main"],
    },
)
