import os
from glob import glob
from setuptools import find_packages, setup

package_name = "dagger_bringup"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "params"), glob("params/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jetson",
    maintainer_email="jetson@todo.todo",
    description="Bringup, launch composition, and command-velocity safety arbitration for the Dagger UGV",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cmd_vel_safety = dagger_bringup.cmd_vel_safety:main",
            "cmd_vel_to_rc = dagger_bringup.cmd_vel_to_rc:main",
            "system_ready = dagger_bringup.system_ready:main",
            "gps_relay = dagger_bringup.gps_relay:main",
            "self_filter = dagger_bringup.self_filter:main",
            "extnav_yaw_relay = dagger_bringup.extnav_yaw_relay:main",
        ],
    },
)
