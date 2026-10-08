#!/usr/bin/env python3
"""Publish an RViz MarkerArray of the robot base, both AprilTags, and the three object poses.

Every marker is expressed in the robot base frame, so with RViz's fixed frame set to
that frame the base sits at the origin and the other poses are relative to it.
"""

import math

import numpy as np

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import transforms3d._gohlketransforms as t3
import tf2_ros
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


_AXIS_COLORS = {
    "x": ColorRGBA(r=0.9, g=0.1, b=0.1, a=1.0),
    "y": ColorRGBA(r=0.1, g=0.8, b=0.1, a=1.0),
    "z": ColorRGBA(r=0.2, g=0.4, b=1.0, a=1.0),
}


def _quat_along_axis(frame_wxyz, axis):
    """Orientation of an arrow that points along a local axis of a frame."""
    rotation = t3.quaternion_matrix(frame_wxyz)
    if axis == "y":
        rotation = t3.concatenate_matrices(rotation, t3.rotation_matrix(math.pi / 2.0, (0.0, 0.0, 1.0)))
    elif axis == "z":
        rotation = t3.concatenate_matrices(rotation, t3.rotation_matrix(-math.pi / 2.0, (0.0, 1.0, 0.0)))
    quat = t3.quaternion_from_matrix(rotation)  # (w, x, y, z)
    return float(quat[1]), float(quat[2]), float(quat[3]), float(quat[0])


class PoseMarkers(Node):
    def __init__(self):
        super().__init__("pose_markers")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.marker_pub = self.create_publisher(MarkerArray, "pose_markers", 10)

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("jackal_frame", "jackal_anchor")
        self.declare_parameter("table_frame", "table_in_base")
        self.declare_parameter("pose_0_frame", "table_pose_0")
        self.declare_parameter("pose_1_frame", "table_pose_1")
        self.declare_parameter("pose_2_frame", "table_pose_2")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("axis_length", 0.08)
        self.declare_parameter("base_axis_length", 0.15)

        self.base_frame = self.get_parameter("base_frame").value
        self.frames = [
            ("base", self.base_frame, float(self.get_parameter("base_axis_length").value), True),
            ("jackal", self.get_parameter("jackal_frame").value, float(self.get_parameter("axis_length").value), False),
            ("table", self.get_parameter("table_frame").value, float(self.get_parameter("axis_length").value), False),
            ("object_0", self.get_parameter("pose_0_frame").value, float(self.get_parameter("axis_length").value), False),
            ("object_1", self.get_parameter("pose_1_frame").value, float(self.get_parameter("axis_length").value), False),
            ("object_2", self.get_parameter("pose_2_frame").value, float(self.get_parameter("axis_length").value), False),
        ]
        self._missing = set()

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.create_timer(1.0 / max(rate, 1e-3), self._publish)
        self.get_logger().info(
            "Publishing pose_markers in "
            + self.base_frame
            + " for "
            + ", ".join(frame for _, frame, _, _ in self.frames)
        )

    def _lookup(self, child):
        try:
            return self.tf_buffer.lookup_transform(
                self.base_frame, child, rclpy.time.Time(), timeout=Duration(seconds=0.05)
            )
        except Exception as exc:
            if child not in self._missing:
                self.get_logger().warn(f"Could not look up {self.base_frame}->{child}: {exc}")
                self._missing.add(child)
            return None

    def _frame_pose(self, name, frame, identity):
        if identity:
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
        stamped = self._lookup(frame)
        if stamped is None:
            return None
        self._missing.discard(frame)
        translation = stamped.transform.translation
        rotation = stamped.transform.rotation
        return (translation.x, translation.y, translation.z), (rotation.x, rotation.y, rotation.z, rotation.w)

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        for name, frame, length, identity in self.frames:
            pose = self._frame_pose(name, frame, identity)
            if pose is None:
                markers.markers.extend(self._clear(name, stamp))
                self.get_logger().info(f"No available tranform to {name}")
                continue
            if name == "object_1":

                self.get_logger().info(f"{name}: {np.array2string(np.asarray(list(pose[0][:3])), precision=2)}")

            translation, xyzw = pose
            markers.markers.extend(self._axes(name, frame, stamp, translation, xyzw, length))
        self.marker_pub.publish(markers)

    def _header(self, marker, stamp):
        marker.header.frame_id = self.base_frame
        marker.header.stamp = stamp

    def _axes(self, name, label, stamp, translation, xyzw, length):
        qx, qy, qz, qw = xyzw
        frame_wxyz = (qw, qx, qy, qz)
        markers = []
        for index, axis in enumerate(("x", "y", "z")):
            arrow = Marker()
            self._header(arrow, stamp)
            arrow.ns = name
            arrow.id = index
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = float(translation[0])
            arrow.pose.position.y = float(translation[1])
            arrow.pose.position.z = float(translation[2])
            ax, ay, az, aw = _quat_along_axis(frame_wxyz, axis)
            arrow.pose.orientation.x = ax
            arrow.pose.orientation.y = ay
            arrow.pose.orientation.z = az
            arrow.pose.orientation.w = aw
            arrow.scale.x = length
            arrow.scale.y = length * 0.08
            arrow.scale.z = length * 0.16
            arrow.color = _AXIS_COLORS[axis]
            markers.append(arrow)

        text = Marker()
        self._header(text, stamp)
        text.ns = name
        text.id = 3
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(translation[0])
        text.pose.position.y = float(translation[1])
        text.pose.position.z = float(translation[2]) + length * 0.35
        text.pose.orientation.w = 1.0
        text.scale.z = length * 0.35
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        text.text = label
        markers.append(text)

        sphere = Marker()
        self._header(sphere, stamp)
        sphere.ns = name
        sphere.id = 4
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = float(translation[0])
        sphere.pose.position.y = float(translation[1])
        sphere.pose.position.z = float(translation[2])
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = length * 0.12
        sphere.scale.y = length * 0.12
        sphere.scale.z = length * 0.12
        sphere.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
        markers.append(sphere)
        return markers

    def _clear(self, name, stamp):
        cleared = []
        for marker_id in range(5):
            marker = Marker()
            self._header(marker, stamp)
            marker.ns = name
            marker.id = marker_id
            marker.action = Marker.DELETE
            cleared.append(marker)
        return cleared


def main(args=None):
    rclpy.init(args=args)
    node = PoseMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
