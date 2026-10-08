"""
robot_info.py

Handles publishing the link and joint names of the robot and setting
the initial position of the robot.

Written by Will Solow, 2026. IRAS Lab.
"""

import time

import rclpy
from gen3_cpp.msg import RobotInfo
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from urdf_parser_py.urdf import URDF


class RobotInfoPublisher(Node):
    def __init__(self):
        super().__init__("robot_info_pub")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("traj_duration_sec", 0)
        self.declare_parameter("traj_duration_nsec", int(1e8))
        self.declare_parameter(
            "default_joint_pos",
            [0.0, 0.523599, 0.0, 1.5708, 0.0, 0.785398, 0.0],
        )
        self.declare_parameter(
            "joint_names",
            [
                "joint_1",
                "joint_2",
                "joint_3",
                "joint_4",
                "joint_5",
                "joint_6",
                "joint_7",
            ],
        )

        urdf_xml = self.get_parameter("robot_description").get_parameter_value().string_value
        self.default_joint_pos = self.get_parameter("default_joint_pos").get_parameter_value().double_array_value
        self.traj_duration_sec = self.get_parameter("traj_duration_sec").value
        self.traj_duration_nsec = self.get_parameter("traj_duration_nsec").value
        self.joint_names = self.get_parameter("joint_names").value

        self.robot = URDF.from_xml_string(urdf_xml)

        self.joint_traj_pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )
        waiting_count = 0
        while self.joint_traj_pub.get_subscription_count() == 0:
            time.sleep(0.1)
            waiting_count += 1
            if waiting_count > 50:
                self.get_logger().info("Waiting for /joint_trajectory_controller/joint_trajectory subscribers...")
                waiting_count = 0

        self.publish_initial_joint_state()

        self.pub = self.create_publisher(RobotInfo, "/robot_info", 10)

        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        upper_limit = []
        lower_limit = []
        joints = []
        links = []
        msg_pub = RobotInfo()
        for joint in self.robot.joints:
            if joint.joint_type == "fixed":
                continue
            if joint.limit:
                joints.append(joint.name)
                upper_limit.append(joint.limit.upper)
                lower_limit.append(joint.limit.lower)
        for link in self.robot.links:
            links.append(link.name)

        msg_pub.links = links
        msg_pub.joints = joints
        msg_pub.lower_limits = lower_limit
        msg_pub.upper_limits = upper_limit
        self.pub.publish(msg_pub)

    def publish_initial_joint_state(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.default_joint_pos
        point.time_from_start.sec = self.traj_duration_sec
        point.time_from_start.nanosec = self.traj_duration_nsec
        msg.points = [point]

        self.joint_traj_pub.publish(msg)
        time.sleep(self.traj_duration_sec * 1.5 + 0.1)


def main(args=None):
    rclpy.init(args=args)
    node = RobotInfoPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
