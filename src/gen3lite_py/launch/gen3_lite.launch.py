"""
gen3.launch.py

Main launch file for Kinova Gen3 Arm with Robotiq 2F 85 gripper

Written by Will Solow, 2025. IRAS Lab.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    # Packages to load
    pkg_gen3litepy = get_package_share_directory("gen3lite_py")

    # Variables
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file_name = LaunchConfiguration("rviz_config_file")
    robot_controller = LaunchConfiguration("robot_controller")


    moveit_controllers = Path(get_package_share_directory("gen3lite_py")) / "config" / "moveit_controllers.yaml"

    robot_path = Path(get_package_share_directory("gen3lite_py")) / "robot" / "gen3_lite_gen3_lite_2f.xacro"

    # Kinova Arm Launch Description
    kinova_arm_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([pkg_gen3litepy, "launch", "kortex_gen3_lite.launch.py"])),
        launch_arguments={
            "use_fake_hardware": use_fake_hardware,
            "robot_ip": robot_ip,
            "vision": "true",
            "robot_controller": robot_controller,
        }.items(),
    )

    moveit_config = (
        MoveItConfigsBuilder("gen3lite", package_name="kinova_gen3_lite_moveit_config")
        .robot_description(
            file_path=robot_path,
            mappings={
                "use_fake_hardware": use_fake_hardware,
                "robot_ip": robot_ip,
                "gripper": "gen3_lite_2f",
                "gripper_joint_name": "right_finger_bottom_joint",
                "dof": "6",
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

    rviz_config_file = PathJoinSubstitution([pkg_gen3litepy, "rviz", rviz_config_file_name])

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
            default_value="192.168.1.10",
            description="ip of robot",
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
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch Kortex RViz?")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file", default_value="view_robot.rviz", description="Name of RViz file in pkg_gen3litepy/rviz/"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
