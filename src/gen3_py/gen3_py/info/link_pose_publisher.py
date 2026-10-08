"""
link_pose_publisher.py

Handles publishing the current positions of all the links in the robot

Written by Will Solow, 2026. IRAS Lab.
"""

import rclpy
import tf2_ros
from gen3_cpp.msg import BodyInfo
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from urdf_parser_py.urdf import URDF


class LinkPosePublisher(Node):
    def __init__(self):
        super().__init__("link_pose_publisher")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(BodyInfo, "/robot_body_pose_w", 10)

        self.timer = self.create_timer(0.02, self.timer_callback)

        self.declare_parameter("robot_description", "")
        urdf_xml = self.get_parameter("robot_description").get_parameter_value().string_value
        self.robot = URDF.from_xml_string(urdf_xml)

    def timer_callback(self) -> None:
        """Compute and publish the position and orientation of every link in the robot.

        Relative to the world frame.

        """
        pose_msg = BodyInfo()
        all_pose_w = []
        i = 0
        # Compute for the body_pose_w
        # Indices must align with robot_info links
        for link in self.robot.links:
            if link.name == "world":
                for _ in range(7):
                    all_pose_w.append(float("nan"))
            else:
                t = self.get_link_pose(link.name)
                if t is None:
                    for _ in range(7):
                        all_pose_w.append(float("nan"))
                else:
                    pos = t.transform.translation
                    quat = t.transform.rotation
                    for p in [pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w]:
                        all_pose_w.append(p)
            i += 1

        # Compute for the root_pose_w, ie the base link
        t = self.get_link_pose("base_link")
        if t is not None:
            pos = t.transform.translation
            quat = t.transform.rotation
            root_pose_w = [pos.x, pos.y, pos.z, quat.x, quat.y, quat.z, quat.w]

            pose_msg.num_links = i
            pose_msg.root_w = root_pose_w
            pose_msg.body_w = all_pose_w

            self.pub.publish(pose_msg)
        else:
            pass
            # self.get_logger().warn("Translation from world to `base_link` is None!")

    def get_link_pose(self, link_name: str, reference_frame: str = "world") -> TransformStamped:
        """Look up the latest transform between links.

        Args:
            link_name: string of the current link to look up
            reference_frame: string name of the reference frame, defaults to world

        Returns:
            A Transform between link and reference

        """
        try:
            return self.tf_buffer.lookup_transform(
                reference_frame,
                link_name,
                rclpy.time.Time(),
            )
        except Exception:
            return None


def main(args=None):
    rclpy.init(args=args)
    node = LinkPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
