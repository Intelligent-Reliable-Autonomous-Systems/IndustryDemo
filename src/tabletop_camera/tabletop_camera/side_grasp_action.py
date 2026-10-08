#!/usr/bin/env python3
"""Side-grasp action server for Kinova Gen3 Lite via MoveIt.

Exposes GraspObject.action (object name string). Internally uses the MoveIt
MoveGroup / ExecuteTrajectory action clients plus the gripper ParallelGripper
command to:

1. Resolve the named object to a TF frame published by tag11_poses_tf
2. Seed the planning scene with a counter collision box (arm base is below
   the objects on the counter edge)
3. Open the gripper, move to a side-grasp pre-approach pose, Cartesian-slide
   into the grasp, close, then retract up and toward the robot while keeping
   the end-effector orientation (object upright)
"""

from __future__ import annotations

import math
from typing import Optional, Sequence, Tuple

import numpy as np
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros
from control_msgs.action import ParallelGripperCommand
from geometry_msgs.msg import Point, Pose, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    CollisionObject,
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningOptions,
    PositionConstraint,
)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from tabletop_camera_msgs.action import GraspObject


def _normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < 1e-9:
        raise ValueError("Cannot normalize near-zero vector")
    return v / n


def _quat_xyzw_from_rotmat(R: np.ndarray) -> Tuple[float, float, float, float]:
    """Convert 3x3 rotation matrix to quaternion (x, y, z, w)."""
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]
    tr = m00 + m11 + m22
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=float)
    q /= np.linalg.norm(q)
    return float(q[0]), float(q[1]), float(q[2]), float(q[3])


def side_grasp_orientation(approach_xy: Sequence[float]) -> Quaternion:
    """Build an EE orientation for a horizontal side grasp that keeps objects upright.

    Kinova tool convention: +Z is the gripper approach axis. World +Z stays
    roughly aligned with the gripper's "up" so a grasped cup/can remains upright.
    """
    ax, ay = float(approach_xy[0]), float(approach_xy[1])
    z_axis = _normalize(np.array([ax, ay, 0.0], dtype=float))
    world_up = np.array([0.0, 0.0, 1.0], dtype=float)
    x_axis = np.cross(world_up, z_axis)
    if np.linalg.norm(x_axis) < 1e-6:
        x_axis = np.array([1.0, 0.0, 0.0], dtype=float)
    x_axis = _normalize(x_axis)
    y_axis = _normalize(np.cross(z_axis, x_axis))
    R = np.column_stack((x_axis, y_axis, z_axis))
    qx, qy, qz, qw = _quat_xyzw_from_rotmat(R)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def make_pose(xyz: Sequence[float], quat: Quaternion) -> Pose:
    p = Pose()
    p.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
    p.orientation = quat
    return p


class SideGraspAction(Node):
    def __init__(self) -> None:
        super().__init__("side_grasp_action")

        self._reentrant = ReentrantCallbackGroup()
        self._server_cb = MutuallyExclusiveCallbackGroup()

        # Frames / MoveIt groups
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("ee_link", "end_effector_link")
        self.declare_parameter("planning_group", "arm")
        self.declare_parameter("move_action", "move_action")
        self.declare_parameter("execute_action", "execute_trajectory")
        self.declare_parameter("cartesian_path_service", "compute_cartesian_path")
        self.declare_parameter("apply_planning_scene_service", "apply_planning_scene")
        self.declare_parameter(
            "gripper_action", "/gen3_lite_2f_gripper_controller/gripper_cmd"
        )
        self.declare_parameter("gripper_joint", "right_finger_bottom_joint")

        # Object name → TF frame map (must match tag11_poses_tf published frames)
        self.declare_parameter(
            "object_names", ["coffee cup", "coke can", "water bottle"]
        )
        self.declare_parameter(
            "object_frames", ["tag11_pose_0", "tag11_pose_1", "tag11_pose_2"]
        )

        # Grasp geometry (metres)
        self.declare_parameter("pregrasp_standoff_m", 0.12)
        self.declare_parameter("grasp_inset_m", 0.0)
        self.declare_parameter("grasp_z_offset_m", 0.0)
        self.declare_parameter("retract_up_m", 0.08)
        self.declare_parameter("retract_out_m", 0.12)
        self.declare_parameter("cartesian_step_m", 0.01)
        self.declare_parameter("cartesian_jump_threshold", 0.0)
        self.declare_parameter("min_cartesian_fraction", 0.85)
        self.declare_parameter("position_tolerance_m", 0.01)
        self.declare_parameter("orientation_tolerance_rad", 0.15)
        self.declare_parameter("planning_time_s", 5.0)
        self.declare_parameter("num_planning_attempts", 5)
        self.declare_parameter("max_velocity_scaling", 0.3)
        self.declare_parameter("max_acceleration_scaling", 0.3)
        self.declare_parameter("gripper_open_position", 0.0)
        self.declare_parameter("gripper_close_position", 0.75)
        self.declare_parameter("gripper_max_effort", 50.0)
        self.declare_parameter("tf_lookup_timeout_s", 2.0)

        # Counter collision box in base_frame (matches table_scene_node defaults)
        self.declare_parameter("publish_counter_collision", True)
        self.declare_parameter("counter_x0", -0.0889)
        self.declare_parameter("counter_y0", -0.577)
        self.declare_parameter("counter_dx", 0.762)
        self.declare_parameter("counter_dy", 1.2446)
        self.declare_parameter("counter_thickness", 0.03)
        # Extra bulk behind the edge so the planner treats the counter volume
        # as occupied above the surface near the edge (arm approaches from below).
        self.declare_parameter("counter_bulk_height", 0.35)
        self.declare_parameter("counter_bulk_inset_from_edge", 0.08)

        self.base_frame = self.get_parameter("base_frame").value
        self.ee_link = self.get_parameter("ee_link").value
        self.planning_group = self.get_parameter("planning_group").value
        self.gripper_joint = self.get_parameter("gripper_joint").value

        names = list(self.get_parameter("object_names").value)
        frames = list(self.get_parameter("object_frames").value)
        if len(names) != len(frames):
            raise RuntimeError("object_names and object_frames must be the same length")
        self.object_frames = {
            n.strip().lower(): f for n, f in zip(names, frames)
        }

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._move_client = ActionClient(
            self, MoveGroup, self.get_parameter("move_action").value,
            callback_group=self._reentrant,
        )
        self._exec_client = ActionClient(
            self, ExecuteTrajectory, self.get_parameter("execute_action").value,
            callback_group=self._reentrant,
        )
        self._gripper_client = ActionClient(
            self, ParallelGripperCommand, self.get_parameter("gripper_action").value,
            callback_group=self._reentrant,
        )
        self._cartesian_cli = self.create_client(
            GetCartesianPath,
            self.get_parameter("cartesian_path_service").value,
            callback_group=self._reentrant,
        )
        self._scene_cli = self.create_client(
            ApplyPlanningScene,
            self.get_parameter("apply_planning_scene_service").value,
            callback_group=self._reentrant,
        )

        self._action_server = ActionServer(
            self,
            GraspObject,
            "grasp_object",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self._server_cb,
        )

        self._busy = False
        self.get_logger().info(
            f"Sidegrasp ready. Objects: {self.object_frames}. "
            f"Waiting for MoveIt on '{self.get_parameter('move_action').value}'."
        )

    def goal_callback(self, goal_request: GraspObject.Goal) -> GoalResponse:
        name = goal_request.object_name.strip().lower()
        if name not in self.object_frames:
            self.get_logger().warn(
                f"Rejecting unknown object '{goal_request.object_name}'. "
                f"Known: {sorted(self.object_frames)}"
            )
            return GoalResponse.REJECT
        if self._busy:
            self.get_logger().warn("Rejecting grasp: another grasp is in progress")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        self._busy = True
        result = GraspObject.Result()
        feedback = GraspObject.Feedback()
        object_name = goal_handle.request.object_name.strip()
        key = object_name.lower()
        frame = self.object_frames[key]

        def publish_status(text: str) -> None:
            feedback.status = text
            goal_handle.publish_feedback(feedback)
            self.get_logger().info(text)

        try:
            publish_status(f"Looking up '{object_name}' frame '{frame}'")
            obj_pose = self._lookup_object_pose(frame)
            if obj_pose is None:
                return self._abort(goal_handle, result, f"TF lookup failed for {frame}")

            if bool(self.get_parameter("publish_counter_collision").value):
                publish_status("Applying counter collision to planning scene")
                ok = await self._apply_counter_collision()
                if not ok:
                    return self._abort(
                        goal_handle, result, "Failed to apply counter planning scene"
                    )

            ox = obj_pose.position.x
            oy = obj_pose.position.y
            oz = obj_pose.position.z + float(
                self.get_parameter("grasp_z_offset_m").value
            )

            # Horizontal approach: from robot toward object in the XY plane.
            radial = np.array([ox, oy], dtype=float)
            if np.linalg.norm(radial) < 1e-3:
                approach_xy = np.array([1.0, 0.0])
            else:
                approach_xy = _normalize(radial)

            quat = side_grasp_orientation(approach_xy)
            standoff = float(self.get_parameter("pregrasp_standoff_m").value)
            inset = float(self.get_parameter("grasp_inset_m").value)
            retract_up = float(self.get_parameter("retract_up_m").value)
            retract_out = float(self.get_parameter("retract_out_m").value)

            grasp_xyz = [
                ox - approach_xy[0] * inset,
                oy - approach_xy[1] * inset,
                oz,
            ]
            pregrasp_xyz = [
                grasp_xyz[0] - approach_xy[0] * standoff,
                grasp_xyz[1] - approach_xy[1] * standoff,
                oz,
            ]
            # Retract: up (+Z) and back toward the robot (opposite approach)
            retract_xyz = [
                grasp_xyz[0] - approach_xy[0] * retract_out,
                grasp_xyz[1] - approach_xy[1] * retract_out,
                oz + retract_up,
            ]

            pregrasp = make_pose(pregrasp_xyz, quat)
            grasp = make_pose(grasp_xyz, quat)
            retract = make_pose(retract_xyz, quat)

            publish_status("Opening gripper")
            if not await self._set_gripper(
                float(self.get_parameter("gripper_open_position").value)
            ):
                return self._abort(goal_handle, result, "Failed to open gripper")

            if goal_handle.is_cancel_requested:
                return self._cancel(goal_handle, result, "Cancelled before pregrasp")

            publish_status(
                f"Moving to pregrasp "
                f"({pregrasp_xyz[0]:.3f}, {pregrasp_xyz[1]:.3f}, {pregrasp_xyz[2]:.3f})"
            )
            if not await self._move_to_pose(pregrasp):
                return self._abort(goal_handle, result, "Failed to reach pregrasp")

            if goal_handle.is_cancel_requested:
                return self._cancel(goal_handle, result, "Cancelled before approach")

            publish_status("Cartesian approach to grasp")
            if not await self._cartesian_to([grasp]):
                return self._abort(goal_handle, result, "Cartesian approach failed")

            publish_status("Closing gripper")
            if not await self._set_gripper(
                float(self.get_parameter("gripper_close_position").value)
            ):
                return self._abort(goal_handle, result, "Failed to close gripper")

            if goal_handle.is_cancel_requested:
                return self._cancel(goal_handle, result, "Cancelled before retract")

            publish_status(
                "Retracting up and out while holding upright "
                f"({retract_xyz[0]:.3f}, {retract_xyz[1]:.3f}, {retract_xyz[2]:.3f})"
            )
            # Intermediate lift keeps the object upright (same orientation) before
            # sliding away from the counter edge.
            lift = make_pose(
                [grasp_xyz[0], grasp_xyz[1], grasp_xyz[2] + retract_up * 0.5], quat
            )
            if not await self._cartesian_to([lift, retract]):
                return self._abort(goal_handle, result, "Retract failed")

            result.success = True
            result.message = f"Grasped '{object_name}' and retracted"
            publish_status(result.message)
            goal_handle.succeed()
            return result

        except Exception as exc:  # noqa: BLE001 — surface any planning failure
            self.get_logger().error(f"Grasp failed with exception: {exc}")
            return self._abort(goal_handle, result, str(exc))
        finally:
            self._busy = False

    def _abort(self, goal_handle, result: GraspObject.Result, message: str):
        result.success = False
        result.message = message
        self.get_logger().error(message)
        goal_handle.abort()
        return result

    def _cancel(self, goal_handle, result: GraspObject.Result, message: str):
        result.success = False
        result.message = message
        self.get_logger().warn(message)
        goal_handle.canceled()
        return result

    def _lookup_object_pose(self, frame: str) -> Optional[Pose]:
        timeout = Duration(
            seconds=float(self.get_parameter("tf_lookup_timeout_s").value)
        )
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, frame, Time(), timeout=timeout
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"TF lookup {self.base_frame}->{frame}: {exc}")
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        pose = Pose()
        pose.position = Point(x=t.x, y=t.y, z=t.z)
        pose.orientation = Quaternion(x=r.x, y=r.y, z=r.z, w=r.w)
        return pose

    async def _apply_counter_collision(self) -> bool:
        if not self._scene_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("apply_planning_scene service unavailable")
            return False

        x0 = float(self.get_parameter("counter_x0").value)
        y0 = float(self.get_parameter("counter_y0").value)
        dx = float(self.get_parameter("counter_dx").value)
        dy = float(self.get_parameter("counter_dy").value)
        th = float(self.get_parameter("counter_thickness").value)
        bulk_h = float(self.get_parameter("counter_bulk_height").value)
        inset = float(self.get_parameter("counter_bulk_inset_from_edge").value)

        stamp = self.get_clock().now().to_msg()
        objects = []

        # Tabletop slab (top face near z=0, matching table_scene_node)
        top = CollisionObject()
        top.header = Header(frame_id=self.base_frame, stamp=stamp)
        top.id = "counter_top"
        top.operation = CollisionObject.ADD
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [dx, dy, th]
        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.x = x0 + dx / 2.0
        pose.position.y = y0 + dy / 2.0
        pose.position.z = -th / 2.0
        top.primitives.append(prim)
        top.primitive_poses.append(pose)
        objects.append(top)

        # Bulk volume set back from the near edge so the arm may approach the
        # edge-mounted objects from below/side without planning through the
        # counter interior.
        bulk = CollisionObject()
        bulk.header = Header(frame_id=self.base_frame, stamp=stamp)
        bulk.id = "counter_bulk"
        bulk.operation = CollisionObject.ADD
        bprim = SolidPrimitive()
        bprim.type = SolidPrimitive.BOX
        bulk_dx = max(dx - inset, 0.05)
        bulk_dy = max(dy - inset, 0.05)
        bprim.dimensions = [bulk_dx, bulk_dy, bulk_h]
        bpose = Pose()
        bpose.orientation.w = 1.0
        # Shift bulk away from the robot-side edge (near y0 / x0 corner).
        bpose.position.x = x0 + inset + bulk_dx / 2.0
        bpose.position.y = y0 + inset + bulk_dy / 2.0
        bpose.position.z = bulk_h / 2.0
        bulk.primitives.append(bprim)
        bulk.primitive_poses.append(bpose)
        objects.append(bulk)

        req = ApplyPlanningScene.Request()
        req.scene.is_diff = True
        req.scene.world.collision_objects = objects
        future = self._scene_cli.call_async(req)
        await future
        resp = future.result()
        return bool(resp is not None and resp.success)

    def _pose_constraints(self, pose: Pose) -> Constraints:
        pos_tol = float(self.get_parameter("position_tolerance_m").value)
        orn_tol = float(self.get_parameter("orientation_tolerance_rad").value)
        stamp = self.get_clock().now().to_msg()

        constraints = Constraints()

        pc = PositionConstraint()
        pc.header = Header(frame_id=self.base_frame, stamp=stamp)
        pc.link_name = self.ee_link
        pc.weight = 1.0
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [pos_tol]
        bv = BoundingVolume()
        bv.primitives.append(sphere)
        center = Pose()
        center.position = pose.position
        center.orientation.w = 1.0
        bv.primitive_poses.append(center)
        pc.constraint_region = bv
        constraints.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header = Header(frame_id=self.base_frame, stamp=stamp)
        oc.link_name = self.ee_link
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = orn_tol
        oc.absolute_y_axis_tolerance = orn_tol
        oc.absolute_z_axis_tolerance = orn_tol
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)
        return constraints

    async def _move_to_pose(self, pose: Pose) -> bool:
        if not self._move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup action server unavailable")
            return False

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = self.planning_group
        req.num_planning_attempts = int(
            self.get_parameter("num_planning_attempts").value
        )
        req.allowed_planning_time = float(self.get_parameter("planning_time_s").value)
        req.max_velocity_scaling_factor = float(
            self.get_parameter("max_velocity_scaling").value
        )
        req.max_acceleration_scaling_factor = float(
            self.get_parameter("max_acceleration_scaling").value
        )
        req.goal_constraints.append(self._pose_constraints(pose))

        options = PlanningOptions()
        options.plan_only = False
        options.replan = True
        options.replan_attempts = 3
        goal.planning_options = options

        send_future = self._move_client.send_goal_async(goal)
        await send_future
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("MoveGroup goal rejected")
            return False
        result_future = goal_handle.get_result_async()
        await result_future
        result = result_future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"MoveGroup failed with error_code={result.error_code.val}"
            )
            return False
        return True

    async def _cartesian_to(self, waypoints: Sequence[Pose]) -> bool:
        if not self._cartesian_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("compute_cartesian_path unavailable")
            return False

        req = GetCartesianPath.Request()
        req.header = Header(
            frame_id=self.base_frame, stamp=self.get_clock().now().to_msg()
        )
        req.group_name = self.planning_group
        req.link_name = self.ee_link
        req.waypoints = list(waypoints)
        req.max_step = float(self.get_parameter("cartesian_step_m").value)
        req.jump_threshold = float(self.get_parameter("cartesian_jump_threshold").value)
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = float(
            self.get_parameter("max_velocity_scaling").value
        )
        req.max_acceleration_scaling_factor = float(
            self.get_parameter("max_acceleration_scaling").value
        )

        future = self._cartesian_cli.call_async(req)
        await future
        resp = future.result()
        if resp is None:
            return False
        min_frac = float(self.get_parameter("min_cartesian_fraction").value)
        if resp.fraction < min_frac:
            self.get_logger().error(
                f"Cartesian path only achieved fraction={resp.fraction:.2f} "
                f"(need >= {min_frac:.2f})"
            )
            return False
        if resp.error_code.val not in (MoveItErrorCodes.SUCCESS, 1):
            # Some MoveIt builds still return SUCCESS=1 even when fraction < 1
            self.get_logger().warn(
                f"Cartesian path error_code={resp.error_code.val}, "
                f"fraction={resp.fraction:.2f}"
            )

        return await self._execute_trajectory(resp.solution)

    async def _execute_trajectory(self, trajectory) -> bool:
        if not self._exec_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("ExecuteTrajectory action unavailable")
            return False
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self._exec_client.send_goal_async(goal)
        await send_future
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("ExecuteTrajectory goal rejected")
            return False
        result_future = goal_handle.get_result_async()
        await result_future
        result = result_future.result().result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"ExecuteTrajectory failed error_code={result.error_code.val}"
            )
            return False
        return True

    async def _set_gripper(self, position: float) -> bool:
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Gripper action server unavailable")
            return False
        goal = ParallelGripperCommand.Goal()
        js = JointState()
        js.name = [self.gripper_joint]
        js.position = [float(position)]
        effort = float(self.get_parameter("gripper_max_effort").value)
        js.effort = [effort]
        goal.command = js
        send_future = self._gripper_client.send_goal_async(goal)
        await send_future
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            return False
        result_future = goal_handle.get_result_async()
        await result_future
        # Stall while grasping is expected; treat acceptance + completion as OK.
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SideGraspAction()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
