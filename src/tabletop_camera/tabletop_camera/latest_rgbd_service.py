"""On-demand RGB-D snapshot service for RealSense.

Listens to the camera pose and publishes the image and pose. For reference only."""

from __future__ import annotations

import copy

import cv2
from tabletop_camera_msgs.srv import GetLatestRgbd
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from tf2_ros import Buffer, TransformException, TransformListener


class LatestRgbdService(Node):
    """Cache latest RGB/depth/intrinsics and expose through a typed ROS service."""

    def __init__(self) -> None:
        super().__init__("latest_rgbd_service")

        self.declare_parameter("rgb_topic", "/table_camera/realsense/color/image_raw/compressed")
        self.declare_parameter("depth_topic", "/table_camera/realsense/aligned_depth_to_color/image_raw")
        self.declare_parameter(
            "camera_info_topic",
            "/table_camera/realsense/aligned_depth_to_color/camera_info",
        )
        self.declare_parameter("service_name", "/table_camera/realsense/get_latest_frame")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("camera_frame", "table_camera_link")

        rgb_topic = str(self.get_parameter("rgb_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        service_name = str(self.get_parameter("service_name").value)
        self._world_frame = str(self.get_parameter("world_frame").value)
        self._camera_frame_param = str(self.get_parameter("camera_frame").value)

        self._latest_rgb_jpeg: bytes | None = None
        self._latest_depth_mm: np.ndarray | None = None
        self._latest_camera_info: CameraInfo | None = None
        self._latest_stamp_sec: int | None = None
        self._latest_stamp_nanosec: int | None = None
        self._latest_frame_id: str = ""

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_subscription(CompressedImage, rgb_topic, self._rgb_cb, 10)
        self.create_subscription(Image, depth_topic, self._depth_cb, 10)
        self.create_subscription(CameraInfo, camera_info_topic, self._camera_info_cb, 10)

        self.create_service(GetLatestRgbd, service_name, self._handle_get_latest)
        self.get_logger().info(f"Serving latest RGB-D frame at '{service_name}'")

    def _rgb_cb(self, msg: CompressedImage) -> None:
        self._latest_rgb_jpeg = bytes(msg.data)
        self._latest_stamp_sec = int(msg.header.stamp.sec)
        self._latest_stamp_nanosec = int(msg.header.stamp.nanosec)
        self._latest_frame_id = msg.header.frame_id

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        self._latest_camera_info = msg

    def _depth_cb(self, msg: Image) -> None:
        depth_img = self._image_msg_to_numpy_uint16(msg)
        self._latest_depth_mm = depth_img
        self._latest_stamp_sec = int(msg.header.stamp.sec)
        self._latest_stamp_nanosec = int(msg.header.stamp.nanosec)
        self._latest_frame_id = msg.header.frame_id

    def _image_msg_to_numpy_uint16(self, msg: Image) -> np.ndarray:
        if msg.encoding not in ("16UC1", "mono16"):
            raise ValueError(f"Unsupported depth encoding: {msg.encoding}")

        dtype = np.dtype(np.uint16).newbyteorder(">" if msg.is_bigendian else "<")
        img = np.frombuffer(msg.data, dtype=dtype)
        img = img.reshape((msg.height, msg.width))
        return img.astype(np.uint16, copy=True)

    def _handle_get_latest(self, _: GetLatestRgbd.Request, response: GetLatestRgbd.Response) -> GetLatestRgbd.Response:
        missing: list[str] = []
        if self._latest_rgb_jpeg is None:
            missing.append("rgb_jpeg")
        if self._latest_depth_mm is None:
            missing.append("depth_mm")
        if self._latest_camera_info is None:
            missing.append("camera_info")
        if self._latest_stamp_sec is None or self._latest_stamp_nanosec is None:
            missing.append("timestamp")

        if missing:
            response.success = False
            response.message = f"Missing fields: {', '.join(missing)}"
            return response

        ok, depth_encoded = cv2.imencode(".png", self._latest_depth_mm)
        if not ok:
            response.success = False
            response.message = "Failed to encode depth image to PNG"
            return response

        t_world_cam = self._lookup_t_world_cam()
        response.success = True
        response.message = ""
        response.rgb_jpeg = list(self._latest_rgb_jpeg)
        response.depth_png = depth_encoded.tobytes()
        response.camera_info = copy.deepcopy(self._latest_camera_info)
        response.t_world_cam = t_world_cam
        response.stamp.sec = self._latest_stamp_sec
        response.stamp.nanosec = self._latest_stamp_nanosec
        response.frame_id = self._latest_frame_id
        return response

    def _lookup_t_world_cam(self) -> TransformStamped:
        source_frame = self._latest_frame_id if self._latest_frame_id else self._camera_frame_param
        try:
            return self._tf_buffer.lookup_transform(self._world_frame, source_frame, Time())
        except TransformException as exc:
            self.get_logger().warning(
                f"TF lookup failed for {self._world_frame} -> {source_frame}: {exc}. Using identity transform."
            )
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = self._world_frame
            transform.child_frame_id = source_frame
            transform.transform.rotation.w = 1.0
            return transform


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = LatestRgbdService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
