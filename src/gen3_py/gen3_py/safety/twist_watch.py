#!/usr/bin/env python3
"""
Twist Watchdog Node

Subscribes to twist_controller/commands. If no command has been received
within `timeout_sec` seconds AND the last command was non-zero, publishes
a zero Twist to the same topic (or a configurable output topic).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def twist_is_nonzero(msg: Twist) -> bool:
    """Return True if any field of the Twist message is non-zero."""
    linear = msg.linear
    angular = msg.angular
    return any(
        [
            linear.x,
            linear.y,
            linear.z,
            angular.x,
            angular.y,
            angular.z,
        ]
    )


class TwistWatchdog(Node):

    def __init__(self):
        super().__init__("twist_watchdog")

        # ---------- parameters ----------
        self.declare_parameter("timeout_sec", 1.4)
        self.declare_parameter("input_topic", "twist_controller/commands")
        self.declare_parameter("output_topic", "twist_controller/commands")
        self.declare_parameter("timer_period_sec", 0.1)  # watchdog check rate

        timeout_sec = self.get_parameter("timeout_sec").value
        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        timer_period_sec = self.get_parameter("timer_period_sec").value

        # ---------- state ----------
        self._last_msg_time: rclpy.time.Time | None = None
        self._last_cmd_was_nonzero: bool = False
        self._zero_published: bool = False  # avoid spamming zeros

        # ---------- pub / sub ----------
        self._sub = self.create_subscription(
            Twist,
            input_topic,
            self._cmd_callback,
            10,
        )

        self._pub = self.create_publisher(Twist, output_topic, 10)

        # ---------- watchdog timer ----------
        self._timeout_sec = timeout_sec
        self._timer = self.create_timer(timer_period_sec, self._watchdog_callback)

        self.get_logger().info(
            f"TwistWatchdog started — "
            f"input: {input_topic!r}, output: {output_topic!r}, "
            f"timeout: {timeout_sec} s"
        )

    # ------------------------------------------------------------------
    def _cmd_callback(self, msg: Twist) -> None:
        self._last_msg_time = self.get_clock().now()
        self._last_cmd_was_nonzero = twist_is_nonzero(msg)
        self._zero_published = False  # reset so we can send a zero next time

    # ------------------------------------------------------------------
    def _watchdog_callback(self) -> None:
        # No message ever received — nothing to do.
        if self._last_msg_time is None:
            return

        elapsed = (self.get_clock().now() - self._last_msg_time).nanoseconds * 1e-9

        if elapsed >= self._timeout_sec and self._last_cmd_was_nonzero and not self._zero_published:
            self.get_logger().warn(
                f"No Twist received for {elapsed:.2f} s " f"(timeout={self._timeout_sec} s) — publishing zero Twist."
            )
            self._pub.publish(Twist())  # all fields default to 0.0
            self._zero_published = True  # only publish once per timeout event
            self._last_cmd_was_nonzero = False  # nothing more to do until next cmd


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = TwistWatchdog()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
