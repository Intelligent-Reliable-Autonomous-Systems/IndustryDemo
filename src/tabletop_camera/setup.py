from setuptools import find_packages, setup

package_name = "tabletop_camera"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/rgbd_april.launch.py",
                "launch/rgbd_april_poses.launch.py",
                "launch/table_scene.launch.py",
                "launch/side_grasp.launch.py",
            ],
        ),
        ("share/" + package_name + "/config", ["config/camera_info.yaml"]),
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
            "latest_rgbd_service = tabletop_camera.latest_rgbd_service:main",
            "camera_tf = tabletop_camera.camera_tf:main",
            "table_scene = tabletop_camera.table_scene_node:main",
            "tag11_poses_tf = tabletop_camera.tag11_poses_tf:main",
            "side_grasp_action = tabletop_camera.side_grasp_action:main",
        ],
    },
)
