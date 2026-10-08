from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    rgb_topic = "/table_camera/realsense/color/image_raw"
    camera_info_topic = "/table_camera/realsense/aligned_depth_to_color/camera_info"
    kinova_base_link_frame = "base_link"
    tag3_anchor_frame = "tag3_anchor"
    camera_frame = "camera_color_frame"
    tag3_frame = "tag36h11:3"
    tag11_frame = "tag36h11:11"

    return LaunchDescription(
        [
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
                package="apriltag_ros",
                executable="apriltag_node",
                name="apriltag_node",
                namespace="table_camera",
                parameters=[
                    {
                        "image_transport": "compressed",
                        "family": "36h11",
                        "size": 0.033336,
                        "max_hamming": 0,
                        "detector": {
                            "threads": 1,
                            "decimate": 2.0,
                            "blur": 0.0,
                            "refine": True,
                            "sharpening": 0.25,
                            "debug": False,
                        },
                        "tag": {
                            "ids": [3, 11],
                            "sizes": [0.1, 0.036],
                        },
                    }
                ],
                remappings=[
                    ("image_rect", rgb_topic),
                    ("camera_info", camera_info_topic),
                ],
            ),
            # Known static transform: Kinova base <-> AprilTag ID 3
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tag3_anchor_static_tf_pub",
                arguments=[
                    "0.12",  # x
                    "0.005",  # y
                    "0.0",  # z
                    "0.0",  # qx
                    "0.0",  # qy
                    "0.7071068",  # qz  (90° about z: tag +y → base -x)
                    "0.7071068",  # qw
                    kinova_base_link_frame,
                    tag3_anchor_frame,
                ],
            ),
            # Chains tag3 + tag11 detections to publish three poses in base_link.
            # Skips quietly when tag 3 (or tag 11) is not detected.
            Node(
                package="tabletop_camera",
                executable="tag11_poses_tf",
                name="tag11_poses_tf",
                namespace="table_camera",
                parameters=[
                    {
                        "kinova_base_link_frame": kinova_base_link_frame,
                        "tag3_anchor_frame": tag3_anchor_frame,
                        "camera_frame": camera_frame,
                        "tag3_frame": tag3_frame,
                        "tag11_frame": tag11_frame,
                        "publish_rate_hz": 10.0,
                        # Three poses hardcoded in the tag-11 frame (xyz + xyzw).
                        # Replace with your calibrated values.
                        "pose_0_xyz": [0.0, 0.0, 0.0],
                        "pose_0_xyzw": [0.0, 0.0, 0.0, 1.0],
                        "pose_0_frame": "tag11_pose_0",
                        "pose_1_xyz": [0.05, 0.0, 0.0],
                        "pose_1_xyzw": [0.0, 0.0, 0.0, 1.0],
                        "pose_1_frame": "tag11_pose_1",
                        "pose_2_xyz": [0.0, 0.05, 0.0],
                        "pose_2_xyzw": [0.0, 0.0, 0.0, 1.0],
                        "pose_2_frame": "tag11_pose_2",
                    }
                ],
                output="screen",
            ),
        ]
    )
