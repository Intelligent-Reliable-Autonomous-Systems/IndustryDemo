#!/usr/bin/env python3
"""Publish three poses defined in tag-11 into the Kinova base frame.

Uses the known static base↔tag-3 transform plus live AprilTag detections of
tags 3 and 11. If tag 3 (or tag 11) is not currently detected, skips this
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


class Tag11PosesInKinova(Node):
    def __init__(self):
        super().__init__("tag11_poses_in_kinova")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.dyn_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.declare_parameter("kinova_base_link_frame", "base_link")
        self.declare_parameter("tag3_anchor_frame", "tag3_anchor")
        self.declare_parameter("camera_frame", "camera_color_frame")
        self.declare_parameter("tag3_frame", "tag36h11:3")
        self.declare_parameter("tag11_frame", "tag36h11:11")
        self.declare_parameter("publish_rate_hz", 10.0)

        # Three poses expressed in the tag-11 frame: xyz + xyzw quaternion each.
        # Replace these with the measured / calibrated poses for your setup.
        self.declare_parameter("pose_0_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("pose_0_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("pose_0_frame", "tag11_pose_0")
        self.declare_parameter("pose_1_xyz", [0.05, 0.0, 0.0])
        self.declare_parameter("pose_1_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("pose_1_frame", "tag11_pose_1")
        self.declare_parameter("pose_2_xyz", [0.0, 0.05, 0.0])
        self.declare_parameter("pose_2_xyzw", [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter("pose_2_frame", "tag11_pose_2")

        self.base_frame = self.get_parameter("kinova_base_link_frame").value
        self.tag3_anchor_frame = self.get_parameter("tag3_anchor_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.tag3_frame = self.get_parameter("tag3_frame").value
        self.tag11_frame = self.get_parameter("tag11_frame").value
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.poses_in_tag11 = [
            (
                self.get_parameter("pose_0_frame").value,
                _pose_matrix(
                    list(self.get_parameter("pose_0_xyz").value),
                    list(self.get_parameter("pose_0_xyzw").value),
                ),
            ),
            (
                self.get_parameter("pose_1_frame").value,
                _pose_matrix(
                    list(self.get_parameter("pose_1_xyz").value),
                    list(self.get_parameter("pose_1_xyzw").value),
                ),
            ),
            (
                self.get_parameter("pose_2_frame").value,
                _pose_matrix(
                    list(self.get_parameter("pose_2_xyz").value),
                    list(self.get_parameter("pose_2_xyzw").value),
                ),
            ),
        ]

        self._warned_missing = set()
        timer_period = 1.0 / max(publish_rate_hz, 1e-3)
        self.create_timer(timer_period, self.on_timer)
        self.get_logger().info(
            f"Publishing {[name for name, _ in self.poses_in_tag11]} "
            f"in {self.base_frame} via {self.tag3_frame} + {self.tag11_frame}"
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
        # Need live detections of both tags. Tag 3 is the bridge to the base;
        # if it is not visible we cannot place tag-11 poses in the base frame.
        cam_to_tag3 = self._lookup(self.camera_frame, self.tag3_frame)
        if cam_to_tag3 is None:
            return

        cam_to_tag11 = self._lookup(self.camera_frame, self.tag11_frame)
        if cam_to_tag11 is None:
            return

        base_to_tag3 = self._lookup(self.base_frame, self.tag3_anchor_frame)
        if base_to_tag3 is None:
            return

        # Once lookups succeed again, allow future missing-transform warnings.
        self._warned_missing.clear()

        t_tag3 = cam_to_tag3.transform.translation
        r_tag3 = cam_to_tag3.transform.rotation
        mat_cam_to_tag3 = t3.concatenate_matrices(
            t3.translation_matrix((t_tag3.x, t_tag3.y, t_tag3.z)),
            t3.quaternion_matrix((r_tag3.w, r_tag3.x, r_tag3.y, r_tag3.z)),
        )
        mat_tag3_to_cam = t3.inverse_matrix(mat_cam_to_tag3)

        t11 = cam_to_tag11.transform.translation
        r11 = cam_to_tag11.transform.rotation
        mat_cam_to_tag11 = t3.concatenate_matrices(
            t3.translation_matrix((t11.x, t11.y, t11.z)),
            t3.quaternion_matrix((r11.w, r11.x, r11.y, r11.z)),
        )

        bt = base_to_tag3.transform.translation
        br = base_to_tag3.transform.rotation
        mat_base_to_tag3 = t3.concatenate_matrices(
            t3.translation_matrix((bt.x, bt.y, bt.z)),
            t3.quaternion_matrix((br.w, br.x, br.y, br.z)),
        )

        # base -> tag3_anchor -> cam -> tag11
        mat_base_to_tag11 = t3.concatenate_matrices(
            mat_base_to_tag3, mat_tag3_to_cam, mat_cam_to_tag11
        )

        stamp = cam_to_tag11.header.stamp
        transforms = [
            _stamped_from_matrix(
                mat_base_to_tag11, stamp, self.base_frame, "tag11_in_base"
            )
        ]
        for frame_name, mat_tag11_to_pose in self.poses_in_tag11:
            mat_base_to_pose = t3.concatenate_matrices(mat_base_to_tag11, mat_tag11_to_pose)
            transforms.append(
                _stamped_from_matrix(mat_base_to_pose, stamp, self.base_frame, frame_name)
            )

        self.dyn_broadcaster.sendTransform(transforms)


def main(args=None):
    rclpy.init(args=args)
    node = Tag11PosesInKinova()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
