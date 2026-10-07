from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    base_frame = "kinova/base_link"
    tag0_frame = "tag0_anchor"

    return LaunchDescription(
        [
            # Publish the known tag0 anchor position so the viz node can look it up
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="tag0_anchor_static_tf_pub",
                arguments=[
                    "0.1",
                    "0.0",
                    "0.0",
                    "0.0",
                    "0.0",
                    "0.7071068",
                    "0.7071068",
                    base_frame,
                    tag0_frame,
                ],
            ),
            Node(
                package="tabletop_camera",
                executable="table_scene",
                name="table_scene_visualizer",
                parameters=[
                    {
                        "base_frame": base_frame,
                        "tag0_frame": tag0_frame,
                        "tag_size": 0.1,
                        "tag_resolution": 20,
                        "publish_rate_hz": 2.0,
                        "table_leg_height": 0.73,
                    }
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                # arguments=['-d', 'default.rviz'],
            ),
        ]
    )
