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
        (os.path.join("share", package_name, "config"), glob("config/*.xml")),
        (os.path.join("share", package_name, "behavior_trees"), glob("behavior_trees/*.xml")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
        (os.path.join("share", package_name, "web"), glob("web/*")),
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
            "extnav_relay = dagger_bringup.extnav_relay:main",
            "nav_slow_zone = dagger_bringup.nav_slow_zone:main",
            "cmd_vel_safety = dagger_bringup.cmd_vel_safety:main",
            "cmd_vel_to_rc = dagger_bringup.cmd_vel_to_rc:main",
            "system_ready = dagger_bringup.system_ready:main",
            "gps_relay = dagger_bringup.gps_relay:main",
            "self_filter = dagger_bringup.self_filter:main",
            "extnav_yaw_relay = dagger_bringup.extnav_yaw_relay:main",
            "fcu_odom_relay = dagger_bringup.fcu_odom_relay:main",
            "gps_map_odom = dagger_bringup.gps_map_odom:main",
            "gps_heading_imu = dagger_bringup.gps_heading_imu:main",
            "scan_watchdog = dagger_bringup.scan_watchdog:main",
            "person_detector = dagger_bringup.person_detector:main",
            "target_selector = dagger_bringup.target_selector:main",
            "follow_controller = dagger_bringup.follow_controller:main",
            "slam_pose_tf = dagger_bringup.slam_pose_tf:main",
            "slam_geo_init = dagger_bringup.slam_geo_init:main",
            "goal_pose_relay = dagger_bringup.goal_pose_relay:main",
            "waypoint_queue = dagger_bringup.waypoint_queue:main",
            "person_tracker = dagger_bringup.person_tracker:main",
            "marker_tracker = dagger_bringup.marker_tracker:main",
            "ui_server = dagger_bringup.ui_server:main",
            "fcu_config = dagger_bringup.fcu_config:main",
        ],
    },
)
