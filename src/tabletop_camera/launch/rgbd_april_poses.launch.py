from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    rgb_topic = "/table_camera/realsense/color/image_raw"
    camera_info_topic = "/table_camera/realsense/aligned_depth_to_color/camera_info"
    arm_base_link_frame = "base_link"
    jackal_anchor_frame = "jackal_anchor"
    camera_frame = "camera_color_frame"
    # Root of the realsense TF tree; table_poses_tf parents it to arm_base_link_frame
    camera_root_frame = "camera_camera_color_frame"
    jackal_frame = "tag16h5:1"
    table_frame = "tag16h5:0"

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
                        "enable_depth": False,
                        "enable_infra1": False,
                        "enable_infra2": False,
                        "rgb_camera.color_profile": "640x480x30",
                        "depth_module.depth_profile": "640x480x30",
                        "align_depth.enable": False,
                        "pointcloud.enable": False,
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
                        "image_transport": "raw",     
                        "family": "16h5",
                        "size": 0.1,
                        "max_hamming": 1,            
                        "detector": {
                            "threads": 4,              
                            "decimate": 1.0,           
                            "blur": 0.0,
                            "refine": True,
                            "sharpening": 0.25,
                            "debug": False,
                        },
                        "tag": {
                            "ids": [1, 0],
                            "sizes": [0.1, 0.1],
                        },
                    }
                ],

                remappings=[
                    ("image_rect", rgb_topic),
                    ("camera_info", camera_info_topic),
                ],
            ),
            # Known static transform: Kinova base to jackal apriltag
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="jackal_anchor_static_tf_pub",
                arguments=[
                    "0.0",  # x
                    "0.0",  # y
                    "0.0",  # z
                    "0.0",  # qx
                    "0.0",  # qy
                    #"0.0", #qz
                    #"1.0", #qw
                    "0.7071068",  # qz
                    "-0.7071068",  # qw
                    arm_base_link_frame,
                    jackal_anchor_frame,
                ],
            ),
            # Chains jackal + table detections to publish three poses in base_link.
            # Skips quietly when jackal tag or table tag are not detected.
            Node(
                package="tabletop_camera",
                executable="table_poses_tf",
                name="table_poses_tf",
                namespace="table_camera",
                parameters=[
                    {
                        "kinova_base_link_frame": arm_base_link_frame,
                        "jackal_anchor_frame": jackal_anchor_frame,
                        "camera_frame": camera_root_frame,
                        "jackal_frame": jackal_frame,
                        "table_frame": table_frame,
                        "publish_rate_hz": 10.0,
                        # Three poses hardcoded in the table frame (xyz + xyzw).
                        # Replace with your calibrated values.
                        "obj_0_xyz": [0.0, 0.18, 0.05],
                        "obj_0_xyzw": [0.0, 0.0, 0.0, 1.0],
                        "obj_0_frame": "table_pose_0",
                        "obj_1_xyz": [0.05, 0.36, 0.05],
                        "obj_1_xyzw": [0.0, 0.0, 0.0, 1.0],
                        "obj_1_frame": "table_pose_1",
                        "obj_2_xyz": [0.0, 0.52, 0.05],
                        "obj_2_xyzw": [0.0, 0.0, 0.0, 1.0],
                        "obj_2_frame": "table_pose_2",
                    }
                ],
                output="screen",
            ),
            Node(
                package="tabletop_camera",
                executable="pose_markers",
                name="pose_markers",
                namespace="table_camera",
                parameters=[
                    {
                        "base_frame": arm_base_link_frame,
                        "jackal_frame": jackal_anchor_frame,
                        "table_frame": "table_in_base",
                        "obj_0_frame": "table_pose_0",
                        "obj_1_frame": "table_pose_1",
                        "obj_2_frame": "table_pose_2",
                        "publish_rate_hz": 10.0,
                    }
                ],
                output="screen",
            ),
        ]
    )
