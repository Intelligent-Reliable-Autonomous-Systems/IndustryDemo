
import numpy as np

def euler_xyz_to_rotvec(euler: np.ndarray) -> np.ndarray:
    """Convert XYZ extrinsic Euler angles to a rotation vector.
 
    Args:
        euler: array of shape (3,) in radians [rx, ry, rz].
 
    Returns:
        Rotation vector of shape (3,).
 
    """
    rx, ry, rz = euler
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
 
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]])
    Ry = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]])
    Rz = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]])
 
    # Combined: XYZ extrinsic = Z @ Y @ X
    R = Rz @ Ry @ Rx
 
    angle = np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
 
    # Near-zero rotation
    if abs(angle) < 1e-6:
        return np.zeros(3)
 
    # Axis from skew-symmetric part
    axis = np.array(
        [
            R[2, 1] - R[1, 2],
            R[0, 2] - R[2, 0],
            R[1, 0] - R[0, 1],
        ]
    )
    axis = axis / (2.0 * np.sin(angle) + 1e-8)
 
    return axis * angle
 
 
def quat_inv(q: np.ndarray) -> np.ndarray:
    """Inverse of a quaternion [w, x, y, z]."""
    return np.array([q[0], -q[1], -q[2], -q[3]]) / np.dot(q, q)
 
 
def quat_apply(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate vector v (3,) by unit quaternion q [w, x, y, z]."""
    w, xyz = q[0], q[1:]
    t = 2.0 * np.cross(xyz, v)
    return v + w * t + np.cross(xyz, t)
 
 
def base_to_tcp_twist(
    linear_vel: np.ndarray,
    angular_vel: np.ndarray,
    tcp_quat: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Transform a twist from base frame to TCP frame.
 
    Skillet quaternions are [w, x, y, z] following IsaacLab convention.
 
    Args:
        linear_vel: linear velocity in XYZ (m/s), shape (3,)
        angular_vel: angular velocity in RPY (rad/s), shape (3,)
        tcp_quat: current orientation of the TCP frame, shape (4,)
 
    Returns:
        Linear and angular velocity in TCP frame, each shape (3,).
 
    """
    tcp_quat_inv = quat_inv(tcp_quat)
 
    # Rotate linear and angular velocities into TCP frame
    linear_vel_tcp = quat_apply(tcp_quat_inv, linear_vel)
    angular_vel_tcp = quat_apply(tcp_quat_inv, angular_vel)
 
    return linear_vel_tcp, angular_vel_tcp
 


