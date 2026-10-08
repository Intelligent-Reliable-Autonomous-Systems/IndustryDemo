from glob import glob
import os

from setuptools import find_packages, setup

package_name = "tabletop_camera"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),

        ("share/" + package_name + "/config", ["config/camera_info.yaml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
                (
            "share/" + package_name + "/data/apriltag",
            glob("data/apriltag/*.png"),
        ),

    ],
    install_requires=["setuptools", "transforms3d", "numpy"],
    zip_safe=True,
    maintainer="jjewett",
    maintainer_email="jewettje@oregonstate.edu",
    description="An external realsense RGB-D camera for detecting objects in a manipulation workspace",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "table_scene = tabletop_camera.table_scene_node:main",
            "table_poses_tf = tabletop_camera.table_poses_tf:main",
            "pose_markers = tabletop_camera.pose_markers:main",
            "side_grasp_action = tabletop_camera.side_grasp_action:main",
            "teleop_kinova = tabletop_camera.kinova_teleop:main",
            "kinova_ee_pub = tabletop_camera.kinova_ee_publisher:main",
        ],
    },
)
