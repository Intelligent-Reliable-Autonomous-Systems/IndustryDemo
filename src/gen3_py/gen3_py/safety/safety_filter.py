"""
safety_filter.py

The filter layer to check the safety of the command before publishing it to the controller.

The class extracts joint limits from the`urdf` file.

Author: Natalia Zaitseva
"""

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from urdf_parser_py.urdf import URDF


class SafetyFilter(Node):
    def __init__(self):
        super().__init__("safety_filter")
        self.declare_parameter("urdf_filename", "")
        self.declare_parameter("joint_state_topic", "joint_states")
        self.declare_parameter("joint_traj_topic", "/joint_trajectory_controller/joint_trajectory")
        self.declare_parameter("twist_topic", "/twist_controller/commands")
        self.declare_parameter("in_joint_traj_topic", "/safety/joint_trajectory_controller/joint_trajectory")
        self.declare_parameter("in_twist_topic", "/safety/twist_controller/commands")
        self.declare_parameter("num_arm_joints", 7)
        self.declare_parameter("pos_threshold", 0.01)
        self.declare_parameter("vel_threshold", 0.1)
        self.declare_parameter("eff_threshold_fac", 0.80)
        self.declare_parameter("use_fake_hardware", False)

        self.urdf_filename = self.get_parameter("urdf_filename").value
        self.joint_state_topic = self.get_parameter("joint_state_topic").value
        self.joint_traj_topic = self.get_parameter("joint_traj_topic").value
        self.twist_topic = self.get_parameter("twist_topic").value
        self.in_joint_traj_topic = self.get_parameter("in_joint_traj_topic").value
        self.in_twist_topic = self.get_parameter("in_twist_topic").value
        self.num_arm_joints = self.get_parameter("num_arm_joints").value
        self.fake_hardware = self.get_parameter("use_fake_hardware").value

        self.get_logger().warn(f"SAFETY FILTER: {self.fake_hardware}")

        self.state_topic = "/joint_states"
        self.current_pos = None
        self.current_vel = None
        self.current_effort = None
        self.state_sub = self.create_subscription(JointState, self.state_topic, self.state_cb, 10)
        self.joint_cmd_sub = self.create_subscription(JointTrajectory, self.in_joint_traj_topic, self.joint_cmd_cb, 10)
        self.twist_cmd_sub = self.create_subscription(Twist, self.in_twist_topic, self.twist_cmd_cb, 10)
        self.joint_cmd_pub = self.create_publisher(JointTrajectory, self.joint_traj_topic, 10)
        self.twist_cmd_pub = self.create_publisher(Twist, self.twist_topic, 10)

        self.joint_limits = self.read_joint_limits(self.urdf_filename)

        self.pos_threshold = self.get_parameter("pos_threshold").value
        self.vel_threshold = self.get_parameter("vel_threshold").value
        self.eff_threshold_fac = self.get_parameter("eff_threshold_fac").value

    def state_cb(self, msg: JointState):
        """Callback for states
        Runs automatically when receives the joint states"""
        self.current_pos = np.array(msg.position[: self.num_arm_joints], dtype=np.float32)
        self.current_vel = np.array(msg.velocity[: self.num_arm_joints], dtype=np.float32)
        self.current_torq = np.array(msg.effort[: self.num_arm_joints], dtype=np.float32)

    def joint_cmd_cb(self, msg: JointTrajectory):
        """Publishing callback
        Runs when the message is published to cmd_safety"""
        if self.current_pos is None or self.current_vel is None:
            return

        safe = self.safety_arm_check() if not self.fake_hardware else True
        if safe:
            self.joint_cmd_pub.publish(msg)
        else:
            traj_msg = JointTrajectory()

            traj_msg.joint_names = list(self.joint_limits.keys())
            point = JointTrajectoryPoint()
            point.positions = self.current_pos.tolist()
            point.time_from_start = Duration(sec=2)
            traj_msg.points.append(point)
            self.joint_cmd_pub.publish(traj_msg)
            self.get_logger().warn("BLOCKED: Unsafe Joint Trajectory Command")

    def twist_cmd_cb(self, msg: JointTrajectory):
        """Publishing callback
        Runs when the message is published to cmd_safety"""
        if self.current_pos is None or self.current_vel is None:
            return
        safe = self.safety_arm_check() if not self.fake_hardware else True
        if safe:
            self.twist_cmd_pub.publish(msg)
        else:
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.0
            self.twist_cmd_pub.publish(twist_msg)
            self.get_logger().warn("BLOCKED: Unsafe Twist Command")

    def read_joint_limits(self, filepath):
        """Read joint limits from the URDF file and normalizes the names
        from gen3_joint_1 to joint_1 that we use."""
        robot = URDF.from_xml_file(filepath)
        joint_limits = {}

        for i, joint in enumerate(robot.joints):
            if joint.type in ["revolute", "continuous", "prismatic"] and joint.limit:
                name = self.normalize_urdf_name(joint.name)
                if not name.startswith("joint_"):
                    continue
                joint_limits[name] = {
                    "lower": joint.limit.lower,
                    "upper": joint.limit.upper,
                    "effort": joint.limit.effort,
                    "velocity": joint.limit.velocity,
                }
        return joint_limits

    # to avoid joint names mismatch
    def normalize_urdf_name(self, urdf_name):
        if urdf_name.startswith("gen3_"):
            return urdf_name.replace("gen3_", "")
        elif urdf_name.startswith("gen3lite_"):
            return urdf_name.replace("gen3lite_", "")
        elif urdf_name.startswith("gen3_lite_"):
            return urdf_name.replace("gen3_lite_", "")
        else:
            return urdf_name

    def safety_arm_check(self) -> bool:
        """Checks if the joint implied velocity adn acceleration are within a safe range,
        which implies that the torque is in the safe range and prevents the hardware shoutdown.

        Args:
            target: target joint position"""

        for i, joint_name in enumerate(self.joint_limits.keys()):
            velocity_lim = self.joint_limits[joint_name]["velocity"]
            lower_lim = self.joint_limits[joint_name]["lower"]
            upper_lim = self.joint_limits[joint_name]["upper"]
            effort_lim = self.joint_limits[joint_name]["effort"]

            # check for continuous joints
            if lower_lim == upper_lim:
                continue

            if (
                self.current_pos[i] < lower_lim + self.pos_threshold
                or self.current_pos[i] > upper_lim - self.pos_threshold
            ):
                self.get_logger().warn(f"{joint_name} exceeds allowable joint limits.")
                return False
            if abs(self.current_vel[i]) > velocity_lim - self.vel_threshold:
                self.get_logger().warn(f"{joint_name} exceeds allowable velocity.")
                return False
            if abs(self.current_torq[i]) > (effort_lim * self.eff_threshold_fac):
                self.get_logger().warn(
                    f"{joint_name} exceeds allowable effort ({effort_lim}): {abs(self.current_torq[i])} > {effort_lim * self.eff_threshold_fac}."
                )
                return False
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SafetyFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
