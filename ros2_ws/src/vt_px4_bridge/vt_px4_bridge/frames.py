"""ROS (FLU body, z-up world) <-> PX4 (FRD body, z-down world) conversions.

The VSLAM world frame is whatever heading the drone started at, so we publish
to PX4 with POSE_FRAME_FRD (arbitrary-heading, z-down) rather than NED; EKF2
estimates the yaw offset. Converting both world and body frames is a rotation
of pi about x on each side: R' = F R F with F = diag(1, -1, -1).

Pure Python (no ROS imports) so it can be unit tested anywhere.
"""


def flu_to_frd_vector(v):
    """Position / linear or angular velocity: (x, y, z) -> (x, -y, -z)."""
    return (v[0], -v[1], -v[2])


def flu_to_frd_quaternion(q_wxyz):
    """Orientation (w, x, y, z) of FLU body in z-up world -> FRD body in z-down world.

    F R F for F = rot_x(pi) keeps w and x and negates y and z.
    """
    w, x, y, z = q_wxyz
    return (w, x, -y, -z)


def covariance_diagonals(cov36, min_variance):
    """Split a ROS 6x6 row-major covariance into (position, orientation) diagonals.

    Axis sign flips do not change variances. Zero/negative entries (commonly
    published as "unknown") are clamped to min_variance so EKF2 never gets 0.
    """
    diag = [max(cov36[i * 7], min_variance) for i in range(6)]
    return diag[:3], diag[3:]
