#!/usr/bin/env python3
"""Publishes RViz2 markers to visualize the table scene relative to the robot base frame."""

import os

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray, UVCoordinate
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import CompressedImage
from builtin_interfaces.msg import Duration
from ament_index_python.packages import get_package_share_directory


class TableSceneVisualizer(Node):

    # Table surface geometry (all in metres, relative to base_link)
    TABLE_X0 = -0.0889
    TABLE_Y0 = -0.577
    TABLE_DX = 0.762
    TABLE_DY = 1.2446
    TABLE_THICKNESS = 0.03
    LEG_INSET = 0.05
    LEG_RADIUS = 0.025

    def __init__(self):
        super().__init__("table_scene_visualizer")

        self.marker_pub = self.create_publisher(MarkerArray, "table_scene_markers", 10)

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tag0_frame", "tag0_anchor")
        self.declare_parameter("camera_frame", "camera_color_frame")
        self.declare_parameter("tag_size", 0.1)
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("table_leg_height", 0.73)

        self.base_frame = self.get_parameter("base_frame").value
        self.tag0_frame = self.get_parameter("tag0_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.tag_size = self.get_parameter("tag_size").value
        rate = self.get_parameter("publish_rate_hz").value
        self.leg_height = self.get_parameter("table_leg_height").value

        self.tag_texture = self._load_tag_texture()

        self.timer = self.create_timer(1.0 / max(rate, 0.1), self._publish)
        self.get_logger().info("Table scene visualizer started")

    # ------------------------------------------------------------------
    # Image loading
    # ------------------------------------------------------------------

    def _load_tag_texture(self):
        """Read the AprilTag PNG into a CompressedImage for embedding."""
        try:
            pkg_dir = get_package_share_directory("iras_viz")
            tag_path = os.path.join(pkg_dir, "data", "apriltag", "AprilTag-tag36h11-ID0.png")
            with open(tag_path, "rb") as f:
                png_bytes = f.read()

            img = CompressedImage()
            img.format = "png"
            img.data = list(png_bytes)
            self.get_logger().info(f"Loaded AprilTag texture ({len(png_bytes)} bytes) from {tag_path}")
            return img
        except Exception as e:
            self.get_logger().error(f"Failed to load AprilTag texture: {e}")
            return None

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _publish(self):
        now = self.get_clock().now().to_msg()
        lifetime = Duration(sec=3)

        markers = MarkerArray()
        markers.markers.extend(self._table_markers(now, lifetime))
        markers.markers.extend(self._camera_markers(now, lifetime))

        tag = self._apriltag_marker(now, lifetime)
        if tag:
            markers.markers.append(tag)

        self.marker_pub.publish(markers)

    # ------------------------------------------------------------------
    # Table
    # ------------------------------------------------------------------

    def _table_markers(self, stamp, lifetime):
        x0, y0 = self.TABLE_X0, self.TABLE_Y0
        dx, dy = self.TABLE_DX, self.TABLE_DY
        th = self.TABLE_THICKNESS

        cx = x0 + dx / 2.0
        cy = y0 + dy / 2.0

        out = []

        top = Marker()
        top.header.frame_id = self.base_frame
        top.header.stamp = stamp
        top.ns = "table"
        top.id = 0
        top.type = Marker.CUBE
        top.action = Marker.ADD
        top.pose.position.x = cx
        top.pose.position.y = cy
        top.pose.position.z = -th / 2.0
        top.pose.orientation.w = 1.0
        top.scale.x = dx
        top.scale.y = dy
        top.scale.z = th
        top.color = ColorRGBA(r=0.55, g=0.35, b=0.17, a=0.85)
        top.lifetime = lifetime
        out.append(top)

        corners = [
            (x0 + self.LEG_INSET, y0 + self.LEG_INSET),
            (x0 + dx - self.LEG_INSET, y0 + self.LEG_INSET),
            (x0 + self.LEG_INSET, y0 + dy - self.LEG_INSET),
            (x0 + dx - self.LEG_INSET, y0 + dy - self.LEG_INSET),
        ]
        for i, (lx, ly) in enumerate(corners):
            leg = Marker()
            leg.header.frame_id = self.base_frame
            leg.header.stamp = stamp
            leg.ns = "table"
            leg.id = i + 1
            leg.type = Marker.CYLINDER
            leg.action = Marker.ADD
            leg.pose.position.x = lx
            leg.pose.position.y = ly
            leg.pose.position.z = -th - self.leg_height / 2.0
            leg.pose.orientation.w = 1.0
            leg.scale.x = self.LEG_RADIUS * 2.0
            leg.scale.y = self.LEG_RADIUS * 2.0
            leg.scale.z = self.leg_height
            leg.color = ColorRGBA(r=0.45, g=0.28, b=0.12, a=0.85)
            leg.lifetime = lifetime
            out.append(leg)

        return out

    # ------------------------------------------------------------------
    # Camera (positioned in camera_color_frame)
    #
    # camera_color_frame convention: +x forward (optical axis),
    # +y left, +z up.
    # ------------------------------------------------------------------

    def _camera_markers(self, stamp, lifetime):
        # Approximate RealSense dimensions (metres)
        face_w, face_h, face_d = 0.09, 0.025, 0.005
        body_w, body_h, body_d = 0.07, 0.020, 0.035

        out = []

        # Lens face -- flat plate perpendicular to +x (forward)
        face = Marker()
        face.header.frame_id = self.camera_frame
        face.ns = "camera"
        face.id = 0
        face.type = Marker.CUBE
        face.action = Marker.ADD
        face.pose.position.x = -face_d / 2.0
        face.pose.orientation.w = 1.0
        face.scale.x = face_d
        face.scale.y = face_w
        face.scale.z = face_h
        face.color = ColorRGBA(r=0.15, g=0.15, b=0.15, a=0.95)
        face.lifetime = lifetime
        out.append(face)

        # Body -- extends behind the face (-x direction)
        body = Marker()
        body.header.frame_id = self.camera_frame
        body.ns = "camera"
        body.id = 1
        body.type = Marker.CUBE
        body.action = Marker.ADD
        body.pose.position.x = -face_d - body_d / 2.0
        body.pose.orientation.w = 1.0
        body.scale.x = body_d
        body.scale.y = body_w
        body.scale.z = body_h
        body.color = ColorRGBA(r=0.3, g=0.3, b=0.3, a=0.95)
        body.lifetime = lifetime
        out.append(body)

        # Top indicator -- small red nub on the +z side (physical "up")
        nub = Marker()
        nub.header.frame_id = self.camera_frame
        nub.ns = "camera"
        nub.id = 2
        nub.type = Marker.CUBE
        nub.action = Marker.ADD
        nub_h = 0.006
        nub.pose.position.x = -face_d - body_d / 2.0
        nub.pose.position.z = body_h / 2.0 + nub_h / 2.0
        nub.pose.orientation.w = 1.0
        nub.scale.x = body_d * 0.6
        nub.scale.y = 0.02
        nub.scale.z = nub_h
        nub.color = ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.95)
        nub.lifetime = lifetime
        out.append(nub)

        return out

    # ------------------------------------------------------------------
    # AprilTag (textured quad via TRIANGLE_LIST with embedded texture)
    # ------------------------------------------------------------------

    def _apriltag_marker(self, stamp, lifetime):
        if self.tag_texture is None:
            return None

        half = self.tag_size / 2.0

        m = Marker()
        m.header.frame_id = self.tag0_frame
        m.header.stamp = stamp
        m.ns = "apriltag0"
        m.id = 0
        m.type = Marker.TRIANGLE_LIST
        m.action = Marker.ADD

        m.pose.position.z = 0.001
        m.pose.orientation.w = 1.0

        m.scale.x = 1.0
        m.scale.y = 1.0
        m.scale.z = 1.0

        m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)

        # Two triangles forming a quad in the XY plane
        v0 = Point(x=half, y=-half, z=0.0)  # bottom-right
        v1 = Point(x=half, y=half, z=0.0)  # top-right
        v2 = Point(x=-half, y=half, z=0.0)  # top-left
        v3 = Point(x=-half, y=-half, z=0.0)  # bottom-left

        m.points = [v0, v1, v2, v0, v2, v3]

        # Embed the PNG data directly in the message
        m.texture_resource = "embedded://apriltag0"
        m.texture = self.tag_texture

        m.uv_coordinates = [
            UVCoordinate(u=1.0, v=1.0),  # v0 (+x, -y)
            UVCoordinate(u=1.0, v=0.0),  # v1 (+x, +y)
            UVCoordinate(u=0.0, v=0.0),  # v2 (-x, +y)
            UVCoordinate(u=1.0, v=1.0),  # v0
            UVCoordinate(u=0.0, v=0.0),  # v2
            UVCoordinate(u=0.0, v=1.0),  # v3 (-x, -y)
        ]

        m.lifetime = lifetime
        return m


def main(args=None):
    rclpy.init(args=args)
    node = TableSceneVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
