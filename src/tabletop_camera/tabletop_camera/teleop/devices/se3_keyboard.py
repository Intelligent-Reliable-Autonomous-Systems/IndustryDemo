"""Keyboard controller for SE(3) control (standalone, no Omni/Carb dependency)."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from pynput import keyboard as pynput_keyboard

from .device_base import DeviceBase, DeviceCfg
from .utils import base_to_tcp_twist, euler_xyz_to_rotvec


class Se3Keyboard(DeviceBase):
    """A keyboard controller for sending SE(3) commands as delta poses and binary command (open/close).

    Uses pynput instead of Omniverse's carb/omni keyboard interface, so it can run
    outside of an Isaac Sim / Omniverse context.

    Key bindings:
        ============================== ================= =================
        Description                    Key (+ve axis)    Key (-ve axis)
        ============================== ================= =================
        Toggle gripper (open/close)    K
        Move along x-axis              W                 S
        Move along y-axis              A                 D
        Move along z-axis              Q                 E
        Rotate along x-axis            Z                 X
        Rotate along y-axis            T                 G
        Rotate along z-axis            C                 V
        ============================== ================= =================
    """

    def __init__(self, cfg: Se3KeyboardCfg):
        self.pos_sensitivity = cfg.pos_sensitivity
        self.rot_sensitivity = cfg.rot_sensitivity
        self.gripper_term = cfg.gripper_term
        self._device = cfg.sim_device
        self.frame = cfg.reference_frame

        # Command buffers
        self._close_gripper = False
        self._delta_pos = np.zeros((3,), )
        self._delta_rot = np.zeros((3,), )

        self._additional_callbacks: dict[str, Callable] = {}

        self._create_key_bindings()

        self._pressed_keys: set[str] = set()

        # Start the pynput listener in a background thread (non-blocking)
        self._listener = pynput_keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release,
        )
        self._listener.start()

    def __del__(self):
        """Stop the keyboard listener."""
        if hasattr(self, "_listener") and self._listener.is_alive():
            self._listener.stop()

    def __str__(self) -> str:
        msg = f"Keyboard Controller for SE(3): {self.__class__.__name__}\n"
        msg += "\t----------------------------------------------\n"
        msg += "\tToggle gripper (open/close): K\n"
        msg += "\tMove arm along x-axis: W/S\n"
        msg += "\tMove arm along y-axis: A/D\n"
        msg += "\tMove arm along z-axis: Q/E\n"
        msg += "\tRotate arm along x-axis: Z/X\n"
        msg += "\tRotate arm along y-axis: T/G\n"
        msg += "\tRotate arm along z-axis: C/V"
        return msg

    # ------------------------------------------------------------------
    # Operations
    # ------------------------------------------------------------------

    def reset(self):
        self._close_gripper = False
        self._delta_pos = np.zeros((3,), )
        self._delta_rot = np.zeros((3,), )
        self._pressed_keys.clear()

    def add_callback(self, key: str, func: Callable):
        """Register a function to call when a key is pressed.

        Args:
            key: Single character string, e.g. "P".
            func: Zero-argument callable.

        """
        self._additional_callbacks[key.upper()] = func

    def advance(self, tcp_pose_b: np.ndnarray) -> np.ndnarray:
        """Return the current command as a npdarray.

        Args:
            tcp_pose_b: TCP pose of the robot in the robot base frame.

        Returns:
            np.ndnarray: [x, y, z, rx, ry, rz] or [x, y, z, rx, ry, rz, gripper]

        """
        rot_vec = euler_xyz_to_rotvec(self._delta_rot)

        if self.frame == "tcp":
            command = np.concatenate((self._delta_pos, rot_vec), axis=0)
        elif self.frame == "base":
            tcp_lin_vel, tcp_ang_vel = base_to_tcp_twist(
                self._delta_pos, self._delta_rot, tcp_pose_b[ 3:7]
            )
            command = np.concatenate((tcp_lin_vel, tcp_ang_vel), axis=0)

        if self.gripper_term:
            gripper_value = 1.0 if self._close_gripper else -1.0
            command = np.concatenate((command, np.asarray([gripper_value], )), axis=0)
        return command

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _key_to_char(self, key) -> str | None:
        """Convert a pynput key object to an uppercase character string, or None."""
        try:
            # Regular character keys
            return key.char.upper()
        except AttributeError:
            # Special keys (shift, ctrl, etc.) — not used here
            return None

    def _on_press(self, key):
        char = self._key_to_char(key)
        if char is None:
            return

        if char == "L":
            self.reset()
            return

        if char == "K":
            self._close_gripper = not self._close_gripper
            return

        # Guard against key-repeat events firing duplicate additions
        if char in self._pressed_keys:
            return
        self._pressed_keys.add(char)

        if char in self._POS_KEYS:
            self._delta_pos += self._INPUT_KEY_MAPPING[char]
        elif char in self._ROT_KEYS:
            self._delta_rot += self._INPUT_KEY_MAPPING[char]

        # User-registered callbacks
        if char in self._additional_callbacks:
            self._additional_callbacks[char]()

    def _on_release(self, key):
        char = self._key_to_char(key)
        if char is None:
            return

        self._pressed_keys.discard(char)

        if char in self._POS_KEYS:
            self._delta_pos -= self._INPUT_KEY_MAPPING[char]
        elif char in self._ROT_KEYS:
            self._delta_rot -= self._INPUT_KEY_MAPPING[char]

    def _create_key_bindings(self):
        self._POS_KEYS = {"W", "S", "A", "D", "Q", "E"}
        self._ROT_KEYS = {"Z", "X", "T", "G", "C", "V"}

        self._INPUT_KEY_MAPPING = {
            # x-axis (forward/back)
            "W": np.asarray([1.0, 0.0, 0.0], ) * self.pos_sensitivity,
            "S": np.asarray([-1.0, 0.0, 0.0], ) * self.pos_sensitivity,
            # y-axis (left/right)
            "A": np.asarray([0.0, 1.0, 0.0], ) * self.pos_sensitivity,
            "D": np.asarray([0.0, -1.0, 0.0], ) * self.pos_sensitivity,
            # z-axis (up/down)
            "Q": np.asarray([0.0, 0.0, 1.0], ) * self.pos_sensitivity,
            "E": np.asarray([0.0, 0.0, -1.0], ) * self.pos_sensitivity,
            # roll (around x)
            "Z": np.asarray([1.0, 0.0, 0.0], ) * self.rot_sensitivity,
            "X": np.asarray([-1.0, 0.0, 0.0], ) * self.rot_sensitivity,
            # pitch (around y)
            "T": np.asarray([0.0, 1.0, 0.0], ) * self.rot_sensitivity,
            "G": np.asarray([0.0, -1.0, 0.0], ) * self.rot_sensitivity,
            # yaw (around z)
            "C": np.asarray([0.0, 0.0, 1.0], ) * self.rot_sensitivity,
            "V": np.asarray([0.0, 0.0, -1.0], ) * self.rot_sensitivity,
        }


@dataclass
class Se3KeyboardCfg(DeviceCfg):
    """Configuration for SE3 keyboard devices."""

    gripper_term: bool = True
    pos_sensitivity: float = 0.4
    rot_sensitivity: float = 0.8
    retargeters: None = None
    class_type: type[DeviceBase] = Se3Keyboard
    reference_frame: str = "base"  # or base
