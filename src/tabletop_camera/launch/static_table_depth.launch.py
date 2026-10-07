from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world_frame = LaunchConfiguration("world_frame")
    camera_frame = LaunchConfiguration("camera_frame")
    camera_x = LaunchConfiguration("camera_x")
    camera_y = LaunchConfiguration("camera_y")
    camera_z = LaunchConfiguration("camera_z")
    camera_qx = LaunchConfiguration("camera_qx")
    camera_qy = LaunchConfiguration("camera_qy")
    camera_qz = LaunchConfiguration("camera_qz")
    camera_qw = LaunchConfiguration("camera_qw")

    return LaunchDescription(
        [
            DeclareLaunchArgument("world_frame", default_value="world"),
            DeclareLaunchArgument("camera_frame", default_value="table_camera_link"),
            DeclareLaunchArgument("camera_x", default_value="0.0"),
            DeclareLaunchArgument("camera_y", default_value="0.0"),
            DeclareLaunchArgument("camera_z", default_value="0.0"),
            DeclareLaunchArgument("camera_qx", default_value="0.0"),
            DeclareLaunchArgument("camera_qy", default_value="0.0"),
            DeclareLaunchArgument("camera_qz", default_value="0.0"),
            DeclareLaunchArgument("camera_qw", default_value="1.0"),
            Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                name="realsense",
                namespace="table_camera",
                parameters=[
                    {
                        "enable_color": True,
                        "enable_depth": True,
                        "enable_infra1": False,
                        "enable_infra2": False,
                        "rgb_camera.color_profile": "640x480x30",
                        "depth_module.depth_profile": "640x480x30",
                        "align_depth.enable": True,
                        "pointcloud.enable": True,
                        "enable_gyro": False,
                        "enable_accel": False,
                        "base_frame_id": camera_frame,
                    }
                ],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="table_camera_static_tf_pub",
                arguments=[
                    camera_x,
                    camera_y,
                    camera_z,
                    camera_qx,
                    camera_qy,
                    camera_qz,
                    camera_qw,
                    world_frame,
                    camera_frame,
                ],
            ),
            Node(
                package="tabletop_camera",
                executable="latest_rgbd_service",
                name="latest_rgbd_service",
                namespace="table_camera",
                output="screen",
                parameters=[
                    {
                        "rgb_topic": "/table_camera/realsense/color/image_raw/compressed",
                        "depth_topic": "/table_camera/realsense/aligned_depth_to_color/image_raw",
                        "camera_info_topic": "/table_camera/realsense/aligned_depth_to_color/camera_info",
                        "service_name": "/table_camera/realsense/get_latest_frame",
                        "world_frame": world_frame,
                        "camera_frame": camera_frame,
                    }
                ],
            ),
        ]
    )
