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
                        # Must match the TF frames published by tag11_poses_tf.
                        "object_names": ["coffee cup", "coke can", "water bottle"],
                        "object_frames": [
                            "tag11_pose_0",
                            "tag11_pose_1",
                            "tag11_pose_2",
                        ],
                        "pregrasp_standoff_m": 0.12,
                        "grasp_inset_m": 0.0,
                        "grasp_z_offset_m": 0.0,
                        "retract_up_m": 0.08,
                        "retract_out_m": 0.12,
                        "publish_counter_collision": True,
                        # Same footprint as table_scene_node defaults.
                        "counter_x0": -0.0889,
                        "counter_y0": -0.577,
                        "counter_dx": 0.762,
                        "counter_dy": 1.2446,
                        "counter_thickness": 0.03,
                        "counter_bulk_height": 0.35,
                        "counter_bulk_inset_from_edge": 0.08,
                    }
                ],
            )
        ]
    )
