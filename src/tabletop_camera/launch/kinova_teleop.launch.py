from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ee_topic_name =  "kinova_ee_pose"

    return LaunchDescription(
        [
            Node(
                package="tabletop_camera",
                executable="kinova_ee_pub",
                name="kinova_ee_pub",
                namespace="kinova",
                parameters=[
                    {
                    "ee_topic_name": ee_topic_name
                    }
                ],
                output="screen",
            ),
             Node(
                            package="tabletop_camera",
                            executable="teleop_kinova",
                            name="teleop_kinova",
                            namespace="kinova",
                            parameters=[
                                {
                                "ee_topic_name": "kinova_ee_pose"
                                }
                            ],
                            output="screen",
                        )
    
        ]
    )
