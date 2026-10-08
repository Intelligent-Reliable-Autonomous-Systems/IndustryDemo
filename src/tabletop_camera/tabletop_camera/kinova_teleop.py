import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand, ParallelGripperCommand
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.action import ActionClient
from rclpy.node import Node

from tabletop_camera.teleop.devices.se3_keyboard import Se3Keyboard, Se3KeyboardCfg
from tabletop_camera.teleop.devices.xbox_controller import XboxControllerCfg, XboxController

class TeleOp(Node):
    def __init__(self):
        super().__init__("kinova_teleop")

        self.declare_parameter("ee_topic_name", "kinova_ee_pose")
        self.declare_parameter("twist_topic", "/twist_controller/commands")
        self.declare_parameter("gripper_name", "robotiq_85_left_knuckle_joint")
        self.declare_parameter("gripper_cmd_type", "GripperCommand")
        self.declare_parameter("gripper_action", "/robotiq_gripper_controller/gripper_cmd")
        self.declare_parameter("teleop_interface", "xbox")
        self.ee_topic = self.get_parameter("ee_topic_name").value
        self.twist_topic = self.get_parameter("twist_topic").value
        self.gripper_name = self.get_parameter("gripper_name").value
        self.gripper_cmd_type = self.get_parameter("gripper_cmd_type").value
        self.gripper_action_name = self.get_parameter("gripper_action").value
        self.teleop_interface = self.get_parameter("teleop_interface").value

        self.ee_sub = self.create_subscription(PoseStamped, self.ee_topic, self.ee_cb, 10)

        if self.teleop_interface == "xbox":
            self.teleop = XboxController(XboxControllerCfg(start_thread=True))
        else:

            self.teleop = Se3Keyboard(Se3KeyboardCfg())

        self.get_logger().info(f"{str(self.teleop)}")

        self.timer = self.create_timer(0.05, self.teleop_callback)

        self.twist_cmd_pub = self.create_publisher(Twist, self.twist_topic, 10)

        self.gripper_client = ActionClient(self, GripperCommand, self.gripper_action_name)
        self.ee_pose = None
        self.gripper_pos = 0.0

        # Controller stuff
        self.declare_parameter("controller_manager", "/controller_manager")
        self.declare_parameter("trajectory_controller", "joint_trajectory_controller")
        self.declare_parameter("twist_controller", "twist_controller")
        self.declare_parameter("service_timeout", 5.0)
 
        cm = self.get_parameter("controller_manager").value
        self._jtc = self.get_parameter("trajectory_controller").value
        self._twist = self.get_parameter("twist_controller").value
        self._timeout = self.get_parameter("service_timeout").value
 
        self._list_cli = self.create_client(ListControllers, f"{cm}/list_controllers")
        self._load_cli = self.create_client(LoadController, f"{cm}/load_controller")
        self._config_cli = self.create_client(ConfigureController, f"{cm}/configure_controller")
        self._switch_cli = self.create_client(SwitchController, f"{cm}/switch_controller")

        self._activate_twist_controller()



    def teleop_callback(self) -> None:
        if self.ee_pose is not None:
            # Twist command
            twist = self.teleop.advance(self.ee_pose)
            twist_msg = Twist()
            twist_msg.linear.x = twist[0]
            twist_msg.linear.y = twist[1]
            twist_msg.linear.z = twist[2]
            twist_msg.angular.x = twist[3]
            twist_msg.angular.y = twist[4]
            twist_msg.angular.z = twist[5]

            self.twist_cmd_pub.publish(twist_msg)

            old_gripper_pos = self.gripper_pos
            if twist[-1] == -1:
                self.gripper_pos = 0.0
            else:
                self.gripper_pos = 0.8
            if self.gripper_cmd_type == "ParallelGripperCommand":
                gripper_goal = ParallelGripperCommand.Goal()
                gripper_goal.command.name = [self.gripper_name]
                gripper_goal.command.position = [self.gripper_pos]
                gripper_goal.command.effort = [100.0]
                self.gripper_action_client.send_goal(gripper_goal)
            else:
                gripper_goal = GripperCommand.Goal()
                gripper_goal.command.position = self.gripper_pos
                gripper_goal.command.max_effort = 100.0
            if old_gripper_pos != self.gripper_pos:
                self.get_logger().info("Sending girpper goal...")
                future = self.gripper_client.send_goal_async(
                gripper_goal, feedback_callback=self._gripper_feedback_cb
                    )
                future.add_done_callback(self._gripper_goal_response_cb)
        
            #self.get_logger().info(f"{twist}")
        
    def ee_cb(self, msg: PoseStamped) -> None:
        self.ee_pose = np.asarray([msg.pose.position.x, 
                   msg.pose.position.y, 
                   msg.pose.position.z, 
                    msg.pose.orientation.w,
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,])

    def _gripper_goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Gripper goal rejected")
            return
        goal_handle.get_result_async().add_done_callback(self._gripper_result_cb)

    def _gripper_feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().debug(f"Gripper at {fb.position:.4f}, effort {fb.effort:.2f}")

    def _gripper_result_cb(self, future):
        result = future.result().result
        self.get_logger().info(
            f"Gripper done: position={result.position:.4f} "
            f"reached_goal={result.reached_goal} stalled={result.stalled}"
        )
    # ------------------------------------------------------------------ #
    # Controller switching
    # ------------------------------------------------------------------ #
    def _call(self, client, request):
        """Blocking service call. Only safe before the node starts spinning."""
        if not client.wait_for_service(timeout_sec=self._timeout):
            raise RuntimeError(f"Service {client.srv_name} not available")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self._timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"Service {client.srv_name} call failed or timed out")
        return future.result()
 
    def _controller_states(self):
        resp = self._call(self._list_cli, ListControllers.Request())
        return {c.name: c.state for c in resp.controller}


    def _activate_twist_controller(self):
        states = self._controller_states()
 
        # Load the twist controller if the controller manager doesn't know it yet
        if self._twist not in states:
            self.get_logger().info(f"Loading '{self._twist}'")
            req = LoadController.Request()
            req.name = self._twist
            if not self._call(self._load_cli, req).ok:
                raise RuntimeError(f"Failed to load '{self._twist}'")
            states = self._controller_states()
 
        # Configure it if it's still unconfigured
        if states.get(self._twist) == "unconfigured":
            self.get_logger().info(f"Configuring '{self._twist}'")
            req = ConfigureController.Request()
            req.name = self._twist
            if not self._call(self._config_cli, req).ok:
                raise RuntimeError(f"Failed to configure '{self._twist}'")
 
        if states.get(self._twist) == "active":
            self.get_logger().info(f"'{self._twist}' already active")
            if states.get(self._jtc) != "active":
                return
 
        # Deactivate the trajectory controller and activate the twist controller atomically
        req = SwitchController.Request()
        req.activate_controllers = [] if states.get(self._twist) == "active" else [self._twist]
        req.deactivate_controllers = [self._jtc] if states.get(self._jtc) == "active" else []
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout = Duration(sec=int(self._timeout))
 
        if not self._call(self._switch_cli, req).ok:
            raise RuntimeError(
                f"Failed to switch from '{self._jtc}' to '{self._twist}'"
            )
        self.get_logger().info(f"Switched from '{self._jtc}' to '{self._twist}'")



def main(args=None):
    rclpy.init(args=args)
    node = TeleOp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()