"""Scripted PX4 offboard mission: arm -> takeoff -> waypoints -> land.

Waypoints are relative to the takeoff point, in PX4 local frame (NED, or FRD
when EKF2 runs on vision only). Good enough for hover-drift and square/figure
tests that exercise VSLAM; real course flight will come from the planner.
"""
import math
from enum import Enum

import rclpy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

NAN = float("nan")

# name -> list of (x, y, z_up, hold_s) relative to takeoff point
MISSIONS = {
    "hover": [(0.0, 0.0, 1.5, 60.0)],
    "square": [(0.0, 0.0, 1.5, 3.0), (2.0, 0.0, 1.5, 2.0), (2.0, 2.0, 1.5, 2.0),
               (0.0, 2.0, 1.5, 2.0), (0.0, 0.0, 1.5, 3.0)],
    "yaw_spin": [(0.0, 0.0, 1.5, 3.0)],  # TODO: add yaw-rate segment to stress VSLAM
}


class State(Enum):
    WARMUP = 0
    ARMING = 1
    FLYING = 2
    LANDING = 3
    DONE = 4


class Mission(Node):
    def __init__(self):
        super().__init__("vt_mission")
        name = self.declare_parameter("mission", "square").value
        self.tolerance = self.declare_parameter("tolerance_m", 0.15).value
        self.waypoints = MISSIONS[name]

        self.offboard_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", PX4_QOS)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", PX4_QOS)
        self.command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", PX4_QOS)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.on_position, PX4_QOS)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.on_status, PX4_QOS)

        self.position = None
        self.origin = None
        self.status = None
        self.state = State.WARMUP
        self.ticks = 0
        self.wp_index = 0
        self.hold_start = None
        self.create_timer(0.05, self.tick)  # 20 Hz; PX4 needs >2 Hz to stay in offboard
        self.get_logger().info(f"mission '{name}' with {len(self.waypoints)} waypoints")

    def on_position(self, msg):
        if msg.xy_valid and msg.z_valid:
            self.position = (msg.x, msg.y, msg.z)

    def on_status(self, msg):
        self.status = msg

    def now_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def send_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.now_us()
        self.command_pub.publish(msg)

    def target_ned(self):
        x, y, z_up, _ = self.waypoints[self.wp_index]
        ox, oy, oz = self.origin
        return (ox + x, oy + y, oz - z_up)

    def publish_setpoint(self, target):
        mode = OffboardControlMode()
        mode.position = True
        mode.timestamp = self.now_us()
        self.offboard_pub.publish(mode)

        sp = TrajectorySetpoint()
        sp.position = [float(c) for c in target]
        sp.velocity = [NAN, NAN, NAN]
        sp.acceleration = [NAN, NAN, NAN]
        sp.yaw = NAN  # hold current heading
        sp.timestamp = self.now_us()
        self.setpoint_pub.publish(sp)

    def tick(self):
        if self.position is None or self.state == State.DONE:
            return
        if self.origin is None:
            self.origin = self.position

        if self.state == State.LANDING:
            if self.status and self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                self.get_logger().info("landed and disarmed")
                self.state = State.DONE
            return

        target = self.target_ned()
        self.publish_setpoint(target)

        if self.state == State.WARMUP:
            self.ticks += 1
            if self.ticks >= 20:  # stream setpoints for 1 s before switching mode
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # custom mode: offboard
                self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.state = State.ARMING
        elif self.state == State.ARMING:
            if (self.status and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
                    and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD):
                self.get_logger().info("armed in offboard")
                self.state = State.FLYING
            elif self.ticks % 20 == 0:  # retry once per second
                self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
                self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.ticks += 1
        elif self.state == State.FLYING:
            self.advance_waypoint(target)

    def advance_waypoint(self, target):
        if math.dist(self.position, target) > self.tolerance:
            self.hold_start = None
            return
        now = self.get_clock().now()
        if self.hold_start is None:
            self.hold_start = now
            return
        hold_s = self.waypoints[self.wp_index][3]
        if (now - self.hold_start).nanoseconds * 1e-9 < hold_s:
            return
        self.hold_start = None
        if self.wp_index + 1 < len(self.waypoints):
            self.wp_index += 1
            self.get_logger().info(f"waypoint {self.wp_index}/{len(self.waypoints) - 1}")
        else:
            self.get_logger().info("mission complete, landing")
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.state = State.LANDING


def main():
    rclpy.init()
    node = Mission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
