"""Forward a nav_msgs/Odometry (VSLAM or ground truth) to PX4 EKF2 as external vision.

Subscribes:  odom_in (nav_msgs/Odometry)   world z-up, child frame = FLU base_link
Publishes:   /fmu/in/vehicle_visual_odometry (px4_msgs/VehicleOdometry)
"""
import math

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from vt_px4_bridge.frames import covariance_diagonals, flu_to_frd_quaternion, flu_to_frd_vector

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class OdomToPx4(Node):
    def __init__(self):
        super().__init__("odom_to_px4")
        self.min_pos_var = self.declare_parameter("min_position_variance", 1e-4).value
        self.min_rot_var = self.declare_parameter("min_orientation_variance", 1e-4).value
        self.min_vel_var = self.declare_parameter("min_velocity_variance", 1e-3).value

        self.pub = self.create_publisher(VehicleOdometry, "/fmu/in/vehicle_visual_odometry", PX4_QOS)
        self.create_subscription(Odometry, "odom_in", self.on_odom, 10)

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular

        out = VehicleOdometry()
        out.timestamp = self.get_clock().now().nanoseconds // 1000
        out.timestamp_sample = (msg.header.stamp.sec * 1_000_000) + (msg.header.stamp.nanosec // 1000)

        out.pose_frame = VehicleOdometry.POSE_FRAME_FRD
        out.position = [float(c) for c in flu_to_frd_vector((p.x, p.y, p.z))]
        out.q = [float(c) for c in flu_to_frd_quaternion((q.w, q.x, q.y, q.z))]

        # nav_msgs/Odometry twist is expressed in the child (body) frame.
        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_BODY_FRD
        out.velocity = [float(c) for c in flu_to_frd_vector((v.x, v.y, v.z))]
        out.angular_velocity = [float(c) for c in flu_to_frd_vector((w.x, w.y, w.z))]

        pos_var, rot_var = covariance_diagonals(msg.pose.covariance, self.min_pos_var)
        rot_var = [max(r, self.min_rot_var) for r in rot_var]
        vel_var, _ = covariance_diagonals(msg.twist.covariance, self.min_vel_var)
        out.position_variance = [float(c) for c in pos_var]
        out.orientation_variance = [float(c) for c in rot_var]
        out.velocity_variance = [float(c) for c in vel_var]

        # TODO: increment on VSLAM resets/relocalization jumps so EKF2 can handle them.
        out.reset_counter = 0
        out.quality = 0

        if any(math.isnan(c) for c in out.position):
            return
        self.pub.publish(out)


def main():
    rclpy.init()
    node = OdomToPx4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
