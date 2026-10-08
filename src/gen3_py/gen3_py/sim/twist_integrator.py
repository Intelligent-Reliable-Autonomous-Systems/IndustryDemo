#!/usr/bin/env python3
"""
velocity_integrator.py

Integrates joint velocity commands into position states for use with
topic_based_ros2_control in fake hardware mode.

Subscribes to:  /fake_joint_commands  (sensor_msgs/JointState)
Publishes to:   /fake_joint_states    (sensor_msgs/JointState)

Usage:
  ros2 run <your_pkg> velocity_integrator --ros-args
    -p joint_names:="['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7']"
    -p initial_positions:="[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
    -p publish_rate:=100.0
    -p command_topic:=/fake_joint_commands
    -p state_topic:=/fake_joint_states
"""

import math

import numpy as np
import PyKDL as kdl
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from kdl_parser_py.urdf import treeFromUrdfModel
from rclpy.node import Node
from rclpy.time import Time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from urdf_parser_py.urdf import URDF


class TwistIntegrator(Node):
    def __init__(self):
        super().__init__("twist_integrator")

        self.declare_parameter(
            "joint_names", ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]
        )
        self.declare_parameter("initial_positions", [0.2, -0.18, 2.16, -1.57, -0.6, -1.34])
        self.declare_parameter("publish_rate", 100.0)  # Hz
        self.declare_parameter("in_topic", "/twist_controller/commands")
        self.declare_parameter("out_topic", "/safety/joint_trajectory_controller/joint_trajectory")
        self.declare_parameter("position_limits_min", [-6.28, -2.41, -6.28, -2.66, -6.28, -2.23, -6.28])
        self.declare_parameter("position_limits_max", [6.28, 2.41, 6.28, 2.66, 6.28, 2.23, 6.28])

        self.joint_names = self.get_parameter("joint_names").value
        initial_positions = self.get_parameter("initial_positions").value
        publish_rate = self.get_parameter("publish_rate").value
        in_topic = self.get_parameter("in_topic").value
        out_topic = self.get_parameter("out_topic").value
        self.pos_min = np.array(self.get_parameter("position_limits_min").value)
        self.pos_max = np.array(self.get_parameter("position_limits_max").value)

        self.num_joints = len(self.joint_names)

        self.positions = np.array(initial_positions, dtype=np.float32)
        self.velocities = np.zeros(self.num_joints, dtype=np.float32)
        self.efforts = np.zeros(self.num_joints, dtype=np.float32)

        # Time tracking for integration
        self.last_twist_time: Time | None = None

        self.create_subscription(Twist, in_topic, self._twist_command_callback, 10)
        self.state_pub = self.create_publisher(JointTrajectory, out_topic, 10)

        self.joint_positions = np.zeros(self.num_joints, dtype=np.float32)

        self.get_logger().info(
            f"TwistIntegrator started\n"
            f"  Joints        : {self.joint_names}\n"
            f"  Initial pos   : {self.positions.tolist()}\n"
            f"  Command topic : {in_topic}\n"
            f"  State topic   : {out_topic}\n"
            f"  Publish rate  : {publish_rate} Hz"
        )

        # Twist Commands
        self.declare_parameter("robot_description", "")
        urdf_xml = self.get_parameter("robot_description").get_parameter_value().string_value
        self.robot = URDF.from_xml_string(urdf_xml)
        self.base_link = "base_link"
        self.ok, self.tree = treeFromUrdfModel(self.robot)

    def _twist_command_callback(self, msg: Twist):
        twist = np.array(
            [
                msg.linear.x,
                msg.linear.y,
                msg.linear.z,
                msg.angular.x,
                msg.angular.y,
                msg.angular.z,
            ]
        )

        chain = self.filter_fixed_joints_from_chain(self.tree.getChain("base_link", "end_effector_link"))
        if chain.getNrOfJoints() == 0:
            return

        jacobian = kdl.Jacobian(chain.getNrOfJoints())
        jac_solver = kdl.ChainJntToJacSolver(chain)
        joints_kdl = self.joints_to_kdl(chain, positions=dict(zip(self.joint_names, self.positions)))
        jac_solver.JntToJac(joints_kdl, jacobian)
        J = self.kdl_to_np(jacobian)

        # Damped pseudoinverse
        q_dot = self.damped_pinv(J, damping=0.05) @ twist

        # Hard clamp as safety net
        MAX_JOINT_VEL = 1.0
        if np.max(np.abs(q_dot)) > MAX_JOINT_VEL:
            q_dot = q_dot / np.max(np.abs(q_dot))

        now = self.get_clock().now()
        if self.last_twist_time is not None:
            dt = (now - self.last_twist_time).nanoseconds * 1e-9
            dt = min(dt, 0.1)
        else:
            dt = 0.0

        for i in range(self.num_joints):
            self.velocities[i] = q_dot[i]
            self.positions[i] += q_dot[i] * dt

        self.positions = np.clip(self.positions, self.pos_min, self.pos_max)
        self.last_twist_time = now

        # Publish
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.positions.tolist()
        point.time_from_start = Duration(sec=math.floor(dt), nanosec=math.floor(dt * 1e9))
        traj.points.append(point)
        self.state_pub.publish(traj)

    def damped_pinv(self, J: np.ndarray, damping: float = 0.05) -> np.ndarray:
        """Damped least squares pseudoinverse — stable near singularities."""
        JJT = J @ J.T
        return J.T @ np.linalg.inv(JJT + damping**2 * np.eye(JJT.shape[0]))

    def filter_fixed_joints_from_chain(self, chain: kdl.Chain) -> kdl.Chain:
        """Create a kinematic chain excluding fixed joints.

        Args:
            base_link: Name of the base link
            end_link: Name of the end effector link

        Returns:
            A PyKDL.Chain with only non-fixed joints
        """
        filtered_chain = kdl.Chain()

        for i in range(chain.getNrOfSegments()):
            segment = chain.getSegment(i)
            joint = segment.getJoint()

            # Only add segments with non-fixed joints
            if joint.getType() != kdl.Joint.Fixed:
                filtered_chain.addSegment(segment)

        return filtered_chain

    def joints_to_kdl(self, chain: kdl.Chain, positions: dict = None) -> kdl.JntArray | None:
        """Returns a KDL array of joint positions.

        Args:
            positions: a dictionary of joint names and joint positions

        Returns:
            A KDL Joint Array containing those positions"""
        kdl_array = kdl.JntArray(chain.getNrOfJoints())
        for i in range(chain.getNrOfJoints()):
            joint_name = chain.getSegment(i).getJoint().getName()
            if joint_name not in positions.keys():
                continue  # Handle unarticulated joints
            kdl_array[i] = positions[joint_name]
        return kdl_array

    def kdl_to_np(self, jac: kdl.Jacobian) -> np.ndarray:
        """Convert a PyKDL Jacobian object to a numpy ndarray.

        Input:
            jac: A PyKDL Jacobian data type of shape (num_links, num_joints)

        Returns:
            A numpy.ndarray of shape (num_links, num_joints)

        """
        arr = np.zeros(shape=(6, self.num_joints))
        for i in range(jac.rows()):
            for j in range(jac.columns()):
                arr[i, j] = jac[i, j]
        return arr


def main(args=None):
    rclpy.init(args=args)
    node = TwistIntegrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
