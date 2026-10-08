"""
gen3.launch.py

Main launch file for Kinova Gen3 Arm with Robotiq 2F 85 gripper

Written by Will Solow, 2025. IRAS Lab.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    # Packages to load
    pkg_kortex_vision = get_package_share_directory("kinova_vision")
    pkg_gen3py = get_package_share_directory("gen3_py")

    # Variables
    use_fake_hardware = LaunchConfiguration("use_fake_hardware", default="true")
    robot_ip = LaunchConfiguration("robot_ip", default="192.168.8.10")
    robot_controller = LaunchConfiguration("robot_controller")
    rviz_config_file_name = LaunchConfiguration("rviz_config_file")
    launch_rviz = LaunchConfiguration("launch_rviz")


    moveit_controllers = Path(get_package_share_directory("gen3_py")) / "config" / "moveit_controllers.yaml"

    robot_path = Path(get_package_share_directory("gen3_py")) / "robot" / "gen3.xacro"


    # Kinova Arm Launch Description
    kinova_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_gen3py, "launch", "kortex_gen3.launch.py"])),
        launch_arguments={
            "use_fake_hardware": use_fake_hardware,
            "robot_ip": robot_ip,
            "gripper": "robotiq_2f_85",
            "vision": "true",
            "robot_controller": robot_controller,
        }.items(),
    )

    kinova_vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_kortex_vision, "launch", "kinova_vision.launch.py"])),
        launch_arguments={
            "device": robot_ip,
        }.items(),
        condition=IfCondition(LaunchConfiguration("vision")),
    )


    moveit_config = (
        MoveItConfigsBuilder("gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config")
        .robot_description(
            file_path=robot_path,
            mappings={
                "use_fake_hardware": use_fake_hardware,
                "robot_ip": robot_ip,
                "gripper": "robotiq_2f_85",
                "gripper_joint_name": "robotiq_85_left_knuckle_joint",
                "dof": "7",
                "gripper_max_velocity": "100",
                "gripper_max_force": "100",
                "use_internal_bus_gripper_comm": "true",
            },
        )
        .trajectory_execution(file_path=moveit_controllers)
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
        ],
    )

    rviz_config_file = PathJoinSubstitution([pkg_gen3py, "rviz", rviz_config_file_name])

    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )


    nodes_to_launch = [
        kinova_arm_launch,
        kinova_vision_launch,
        move_group_node,
        rviz_node,
    ]

    return nodes_to_launch


def generate_launch_description():
    declared_arguments = []

    # Simulation specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="If to only launch the fake hardware",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.18.10",
            description="ip of robot",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "vision",
            default_value="false",
            description="If to load vision topics",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "default_joint_pos",
            default_value="[0.0, 0.523599, 0.0, 1.5708, 0.0, 0.785398, 0.0]",
            description="Default joint positions of the robot",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_controller",
            default_value="joint_trajectory_controller",
            description="Name of robot controller to start",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_table_camera",
            default_value="false",
            description="If to launch table camera and RGB-D snapshot service",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch Kortex RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file", default_value="view_robot_camera.rviz", description="Name of RViz file in gen3_py/rviz/"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
