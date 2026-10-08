from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="tabletop_camera",
                executable="side_grasp_action",
                name="side_grasp_action",
                output="screen",
                parameters=[
                    {
                        "base_frame": "base_link",
                        "ee_link": "end_effector_link",
                        "planning_group": "arm",
                        "move_action": "move_action",
                        "execute_action": "execute_trajectory",
                        "cartesian_path_service": "compute_cartesian_path",
                        "apply_planning_scene_service": "apply_planning_scene",
                        "gripper_action": "/gen3_lite_2f_gripper_controller/gripper_cmd",
                        "gripper_joint": "right_finger_bottom_joint",
                        # Must match the TF frames published by table_poses_tf.
                        "object_names": ["coffee cup", "coke can", "water bottle"],
                        "object_frames": [
                            "table_pose_0",
                            "table_pose_1",
                            "table_pose_2",
                        ],
                        "pregrasp_standoff_m": 0.12,
                        "grasp_inset_m": 0.0,
                        "grasp_z_offset_m": 0.0,
                        "retract_up_m": 0.08,
                        "retract_out_m": 0.12,
                        "publish_counter_collision": True,
                        # Counter footprint is relative to table (published as
                        # table_in_base by table_poses_tf), not Kinova base_link.
                        "counter_frame": "table_in_base",
                        "counter_x0": 1.15,
                        "counter_y0": -5.15,
                        "counter_dx": 0.90,
                        "counter_dy": 0.45,
                        "counter_thickness": 0.05,
                    }
                ],
            )
        ]
    )
