"""
jacobian_pub.py

Handles publishing of jacobian topic, mass matrix, and body velocity based on current joint positions and velocities

Written by Will Solow, 2026. IRAS Lab.
"""

import numpy as np
import PyKDL
import rclpy
from gen3_cpp.msg import BodyInfo, LinkMatrix
from kdl_parser_py.urdf import treeFromUrdfModel
from rclpy.node import Node
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF


class JacobianPublisher(Node):
    def __init__(self):
        super().__init__("jacobian_publisher")
        self.declare_parameter("state_topic", "/joint_states")
        self.declare_parameter("tip_link", "robotiq_85_left_knuckle_joint")
        self.state_topic = self.get_parameter("state_topic").value
        self.tip_link = self.get_parameter("tip_link").value

        self.joint_pos = None
        self.jac_pub = self.create_publisher(LinkMatrix, "jacobian", 10)
        self.mass_pub = self.create_publisher(LinkMatrix, "mass_matrix", 10)
        self.vel_pub = self.create_publisher(BodyInfo, "robot_body_vel_w", 10)
        self.grav_pub = self.create_publisher(LinkMatrix, "gravity_vector", 10)
        self.joint_sub = self.create_subscription(JointState, self.state_topic, self.joint_callback, 10)

        self.declare_parameter("robot_description", "")
        urdf_xml = self.get_parameter("robot_description").get_parameter_value().string_value

        self.robot = URDF.from_xml_string(urdf_xml)
        self.ok, self.tree = treeFromUrdfModel(self.robot)

        self.articulated_joints = None
        self.num_joints = None

        self._wait_count = 0

    def joint_callback(self, msg: JointState) -> None:
        """Recieve a JointState message and publish the Jacobian Transform between all joints.

        Args:
            msg: a JointState mesage

        """
        self.joint_pos = {}
        self.joint_vel = {}
        for i, n in enumerate(msg.name):
            self.joint_pos[n] = msg.position[i]
            self.joint_vel[n] = msg.velocity[i]
        self.num_joints = len(self.joint_pos)

        jacobians = []
        vels = []
        masses = []
        gravities = []
        for link in self.robot.links:
            if link.name == "world":
                continue
            # Exclude all non-articulated links from the chain
            # TODO if we ever want to know where finger links are, we will need to modify this
            if (
                "robotiq_85_left" in link.name
                or "robotiq_85_right" in link.name
                or "right_finger" in link.name
                or "left_finger" in link.name
            ):
                continue

            jac, mass, vel, gravity = self.jacobian_mass_vel(
                link_name=link.name, positions=self.joint_pos, velocities=self.joint_vel
            )

            jacobians.append(jac)
            vels.append(vel)
            masses.append(mass)
            gravities.append(gravity)
        jacobians = np.stack(jacobians, axis=0)
        vels = np.stack(vels, axis=0)
        masses = masses[-1]
        gravities = gravities[-1]

        jacobian_msg = LinkMatrix()
        jacobian_msg.num_links = jacobians.shape[0]
        jacobian_msg.matrix = jacobians.flatten()
        jacobian_msg.rows = jacobians.shape[1]
        jacobian_msg.cols = jacobians.shape[2]
        self.jac_pub.publish(jacobian_msg)

        mass_msg = LinkMatrix()
        mass_msg.num_links = 0
        mass_msg.matrix = masses.flatten()
        mass_msg.rows = masses.shape[0]
        mass_msg.cols = masses.shape[1]
        self.mass_pub.publish(mass_msg)

        gravity_msg = LinkMatrix()
        gravity_msg.num_links = 0
        gravity_msg.matrix = gravities.flatten()
        gravity_msg.rows = gravities.shape[0]
        gravity_msg.cols = 0
        self.grav_pub.publish(gravity_msg)

        vel_msg = BodyInfo()
        vel_msg.num_links = vels.shape[0]
        vel_msg.body_w = vels.flatten()
        self.vel_pub.publish(vel_msg)

    def kdl_to_np(self, jac: PyKDL.Jacobian) -> np.ndarray:
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

    def kdl_to_np_mass(self, mass: PyKDL.JntSpaceInertiaMatrix) -> np.ndarray:
        """Convert a PyKDL Mass object to a numpy ndarray.

        Input:
            jac: A PyKDL Mass data type of shape (num_joints, num_joints)

        Returns:
            A numpy.ndarray of shape (num_joints, num_joints)

        """
        arr = np.zeros(shape=(self.num_joints, self.num_joints))
        for i in range(mass.rows()):
            for j in range(mass.columns()):
                arr[i, j] = mass[i, j]
        return arr

    def jacobian_mass_vel(
        self, link_name: str = None, positions: dict = None, velocities: dict = None
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Computes the Jacobian matrices of all the joints. And the relative linear and angular velocity

        Args:
            link_name: a string of the link name to compute the Jacobian for
            positions: a dictionary of joint names and joint positions

        Returns:
            A numpy ndarray of shape (something, num_joints) for jacobian
            A numpy ndarray of shape (num_joints, num_joints) for mass matrix
            A numpy array of shape (6,) for linear and cartesian velocity
            A numpy array of (something,) for the gravity vector

        """
        # Create a chain from the base link to the current link
        chain = self.filter_fixed_joints_from_chain(self.tree.getChain("base_link", link_name))
        fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)
        jacobian = PyKDL.Jacobian(chain.getNrOfJoints())
        jac_solver = PyKDL.ChainJntToJacSolver(chain)

        link_frame = PyKDL.Frame()
        gravity = PyKDL.Vector(0, 0, -9.81)
        joints_kdl = self.joints_to_kdl(chain, positions=positions)
        joints_vel_kdl = self.joints_to_kdl(chain, positions=velocities)

        fk_solver.JntToCart(joints_kdl, link_frame)  # solve forward kinematics
        jac_solver.JntToJac(joints_kdl, jacobian)  # solve jacobian
        dyn_solver = PyKDL.ChainDynParam(chain, gravity)

        gravity = PyKDL.JntArray(chain.getNrOfJoints())
        dyn_solver.JntToGravity(joints_kdl, gravity)

        # Compute the linear and angular velocity
        twist = PyKDL.Twist()
        for i in range(chain.getNrOfJoints()):
            col = jacobian.getColumn(i)
            twist.vel += col.vel * joints_vel_kdl[i]
            twist.rot += col.rot * joints_vel_kdl[i]

        # Compute the mass matrix
        mass = PyKDL.JntSpaceInertiaMatrix(chain.getNrOfJoints())
        dyn_solver.JntToMass(joints_kdl, mass)

        return (
            self.kdl_to_np(jacobian),
            self.kdl_to_np_mass(mass),
            np.concatenate(
                (
                    [twist.vel.x(), twist.vel.y(), twist.vel.z()],
                    self.rpy_to_quaternion_xyzw(twist.rot.x(), twist.rot.y(), twist.rot.z()),
                ),
                axis=0,
            ),
            np.array([gravity[i] for i in range(chain.getNrOfJoints())]),
        )

    def filter_fixed_joints_from_chain(self, chain: PyKDL.Chain) -> PyKDL.Chain:
        """Create a kinematic chain excluding fixed joints.

        Args:
            base_link: Name of the base link
            end_link: Name of the end effector link

        Returns:
            A PyKDL.Chain with only non-fixed joints
        """
        filtered_chain = PyKDL.Chain()

        for i in range(chain.getNrOfSegments()):
            segment = chain.getSegment(i)
            joint = segment.getJoint()

            # Only add segments with non-fixed joints
            if joint.getType() != PyKDL.Joint.Fixed:
                filtered_chain.addSegment(segment)

        return filtered_chain

    def joints_to_kdl(self, chain: PyKDL.Chain, positions: dict = None) -> PyKDL.JntArray | None:
        """Returns a KDL array of joint positions.

        Args:
            positions: a dictionary of joint names and joint positions

        Returns:
            A KDL Joint Array containing those positions"""
        kdl_array = PyKDL.JntArray(chain.getNrOfJoints())
        for i in range(chain.getNrOfJoints()):
            joint_name = chain.getSegment(i).getJoint().getName()
            if joint_name not in positions.keys():
                continue  # Handle unarticulated joints
            kdl_array[i] = positions[joint_name]
        return kdl_array

    def rpy_to_quaternion_xyzw(self, roll, pitch, yaw):
        cr, sr = np.cos(roll / 2), np.sin(roll / 2)
        cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
        cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)

        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        w = cr * cp * cy + sr * sp * sy

        return np.array([x, y, z, w])


def main(args=None):
    rclpy.init(args=args)
    node = JacobianPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
