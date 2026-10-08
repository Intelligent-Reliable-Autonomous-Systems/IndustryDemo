#!/usr/bin/env python3
"""Publish three poses defined in table into the Kinova base frame.

Uses the known static base to jackal transform plus live AprilTag detections of
tags on jackal and table. If jackal or table tags are not currently detected, skips this
cycle without crashing.
"""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import transforms3d._gohlketransforms as t3
import tf2_ros
from geometry_msgs.msg import TransformStamped


def _pose_matrix(xyz, xyzw):
    """Build a 4x4 transform from translation xyz and quaternion xyzw."""
    x, y, z = xyz
    qx, qy, qz, qw = xyzw
    return t3.concatenate_matrices(
        t3.translation_matrix((x, y, z)),
        t3.quaternion_matrix((qw, qx, qy, qz)),
    )


def _stamped_from_matrix(mat, stamp, parent, child):
    trans = t3.translation_from_matrix(mat)
    quat = t3.quaternion_from_matrix(mat)  # (w, x, y, z)
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = float(trans[0])
    msg.transform.translation.y = float(trans[1])
    msg.transform.translation.z = float(trans[2])
    msg.transform.rotation.w = float(quat[0])
    msg.transform.rotation.x = float(quat[1])
    msg.transform.rotation.y = float(quat[2])
    msg.transform.rotation.z = float(quat[3])
    return msg


class TablePosesInKinova(Node):
    def __init__(self):
        super().__init__("table_poses_in_kinova")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.dyn_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.declare_parameter("kinova_base_link_frame", "base_link")
        self.declare_parameter("jackal_anchor_frame", "jackal_anchor")
        self.declare_parameter("camera_frame", "camera_color_frame")
        self.declare_parameter("jackal_frame", "tag16h5:1")
        self.declare_parameter("table_frame", "tag16h5:0")
        self.declare_parameter("publish_rate_hz", 10.0)

        # Three poses expressed in the table frame: xyz + xyzw quaternion each.
        # Replace these with the measured / calibrated poses for your setup.
        self.declare_parameter("obj_0_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("obj_0_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("obj_0_frame", "table_pose_0")
        self.declare_parameter("obj_1_xyz", [0.05, 0.0, 0.0])
        self.declare_parameter("obj_1_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("obj_1_frame", "table_pose_1")
        self.declare_parameter("obj_2_xyz", [0.0, 0.05, 0.0])
        self.declare_parameter("obj_2_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("obj_2_frame", "table_pose_2")

        self.base_frame = self.get_parameter("kinova_base_link_frame").value
        self.jackal_anchor_frame = self.get_parameter("jackal_anchor_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.jackal_frame = self.get_parameter("jackal_frame").value
        self.table_frame = self.get_parameter("table_frame").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.poses_in_table_frame = [
            (
                self.get_parameter("obj_0_frame").value,
                _pose_matrix(
                    list(self.get_parameter("obj_0_xyz").value),
                    list(self.get_parameter("obj_0_xyzw").value),
                ),
            ),
            (
                self.get_parameter("obj_1_frame").value,
                _pose_matrix(
                    list(self.get_parameter("obj_1_xyz").value),
                    list(self.get_parameter("obj_1_xyzw").value),
                ),
            ),
            (
                self.get_parameter("obj_2_frame").value,
                _pose_matrix(
                    list(self.get_parameter("obj_2_xyz").value),
                    list(self.get_parameter("obj_2_xyzw").value),
                ),
            ),
        ]

        self._warned_missing = set()
        timer_period = 1.0 / max(publish_rate_hz, 1e-3)
        self.create_timer(timer_period, self.on_timer)
        self.get_logger().info(
            f"Publishing {[name for name, _ in self.poses_in_table_frame]} "
            f"in {self.base_frame} via {self.jackal_frame} + {self.table_frame}"
        )

    def _lookup(self, parent, child):
        try:
            return self.tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time(), timeout=Duration(seconds=0.1)
            )
        except Exception as e:
            key = f"{parent}->{child}"
            if key not in self._warned_missing:
                self.get_logger().warn(f"Could not look up {key}: {e}")
                self._warned_missing.add(key)
            return None

    def on_timer(self):
        # Need live detections of both tags. Jackal frame is the bridge to the base;
        # if it is not visible we cannot place table poses in the base frame.
        cam_to_jackal = self._lookup(self.camera_frame, self.jackal_frame)
        if cam_to_jackal is None:
            return

        cam_to_table = self._lookup(self.camera_frame, self.table_frame)
        if cam_to_table is None:
            return

        base_to_jackal = self._lookup(self.base_frame, self.jackal_anchor_frame)
        if base_to_jackal is None:
            return

        # Once lookups succeed again, allow future missing-transform warnings.
        self._warned_missing.clear()

        t_jackal = cam_to_jackal.transform.translation
        r_jackal = cam_to_jackal.transform.rotation
        mat_cam_to_jackal = t3.concatenate_matrices(
            t3.translation_matrix((t_jackal.x, t_jackal.y, t_jackal.z)),
            t3.quaternion_matrix((r_jackal.w, r_jackal.x, r_jackal.y, r_jackal.z)),
        )
        mat_jackal_to_cam = t3.inverse_matrix(mat_cam_to_jackal)

        t11 = cam_to_table.transform.translation
        r11 = cam_to_table.transform.rotation
        mat_cam_to_table = t3.concatenate_matrices(
            t3.translation_matrix((t11.x, t11.y, t11.z)),
            t3.quaternion_matrix((r11.w, r11.x, r11.y, r11.z)),
        )

        bt = base_to_jackal.transform.translation
        br = base_to_jackal.transform.rotation
        mat_base_to_jackal = t3.concatenate_matrices(
            t3.translation_matrix((bt.x, bt.y, bt.z)),
            t3.quaternion_matrix((br.w, br.x, br.y, br.z)),
        )

        # base -> jackal frame -> cam -> table frame
        mat_base_to_table = t3.concatenate_matrices(
            mat_base_to_jackal, mat_jackal_to_cam, mat_cam_to_table
        )

        # base -> jackal frame -> cam, attaches the camera tree under the Kinova base
        mat_base_to_cam = t3.concatenate_matrices(mat_base_to_jackal, mat_jackal_to_cam)

        stamp = cam_to_table.header.stamp
        transforms = [
            _stamped_from_matrix(
                mat_base_to_cam, stamp, self.base_frame, self.camera_frame
            ),
            _stamped_from_matrix(
                mat_base_to_table, stamp, self.base_frame, "table_in_base"
            ),
        ]
        for frame_name, mat_table_to_obj in self.poses_in_table_frame:
            mat_base_to_pose = t3.concatenate_matrices(mat_base_to_table, mat_table_to_obj)
            transforms.append(
                _stamped_from_matrix(mat_base_to_pose, stamp, self.base_frame, frame_name)
            )

        self.dyn_broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = TablePosesInKinova()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
