"""Record estimated and ground-truth odometry to TUM files for evo.

Writes <out_dir>/estimate.tum and <out_dir>/ground_truth.tum on shutdown
(Ctrl-C). Score with: scripts/eval_ate.sh <out_dir>
"""
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


def to_tum(msg):
    t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return f"{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n"


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__("trajectory_recorder")
        self.out_dir = Path(self.declare_parameter("out_dir", "results/latest").value)
        est_topic = self.declare_parameter("estimate_topic", "/visual_slam/tracking/odometry").value
        gt_topic = self.declare_parameter("ground_truth_topic", "/vt/ground_truth/odom").value
        self.rows = {"estimate": [], "ground_truth": []}
        self.create_subscription(Odometry, est_topic, lambda m: self.rows["estimate"].append(to_tum(m)), 50)
        self.create_subscription(Odometry, gt_topic, lambda m: self.rows["ground_truth"].append(to_tum(m)), 50)
        self.get_logger().info(f"recording {est_topic} and {gt_topic} -> {self.out_dir}")

    def save(self):
        self.out_dir.mkdir(parents=True, exist_ok=True)
        for name, rows in self.rows.items():
            (self.out_dir / f"{name}.tum").write_text("".join(rows))
        print(f"saved {len(self.rows['estimate'])} estimate / {len(self.rows['ground_truth'])} ground-truth poses "
              f"to {self.out_dir}")


def main():
    rclpy.init()
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
