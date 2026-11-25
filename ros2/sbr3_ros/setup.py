from glob import glob

from setuptools import setup

package_name = "sbr3_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nespotts",
    maintainer_email="nespotts@users.noreply.github.com",
    description="ROS 2 adapter for the SBR3 balancing robot bridge",
    license="MIT",
    entry_points={
        "console_scripts": [
            "adapter = sbr3_ros.adapter:main",
            "goal_test = sbr3_ros.goal_test:main",
            "bag_tf_filter = sbr3_ros.bag_tf_filter:main",
            "loc_monitor = sbr3_ros.loc_monitor:main",
            "pose_logger = sbr3_ros.pose_logger:main",
            "go_to = sbr3_ros.goal_test:main",
            "patrol = sbr3_ros.patrol:main",
            "keepout_mask = sbr3_ros.keepout_mask:main",
            "explore = sbr3_ros.explore:main",
        ],
    },
)
