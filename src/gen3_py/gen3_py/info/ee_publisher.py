"""
Docstring for gen3.gen3_py.gen3_py.ee_publisher

Publishes the end effector position relative to the base link

Written by Will Solow, 2025
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf2_ros import Buffer, TransformListener
from rcl_interfaces.msg import SetParametersResult


class EEPositionPublisher(Node):
    def __init__(self):
        super().__init__("ee_position_publisher")
        self.pub = self.create_publisher(PoseStamped, "ee_pose", 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_frame", "end_effector_link")

        self.base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self.ee_frame = self.get_parameter("ee_frame").get_parameter_value().string_value
        self.ready = False

    def timer_callback(self):
        if not self.ready:
            if self.tf_buffer.can_transform(
                self.base_frame, self.ee_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            ):
                self.ready = True
                self.get_logger().info("TF tree ready. Starting EE publisher.")
            else:
                self.get_logger().warn("Waiting for TF tree to connect...")
                return
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, rclpy.time.Time())

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.base_frame

            msg.pose.position = Point(
                x=t.transform.translation.x,
                y=t.transform.translation.y,
                z=t.transform.translation.z,
            )

            msg.pose.orientation = Quaternion(
                x=t.transform.rotation.x,
                y=t.transform.rotation.y,
                z=t.transform.rotation.z,
                w=t.transform.rotation.w,
            )

            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Could not get transform: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = EEPositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
