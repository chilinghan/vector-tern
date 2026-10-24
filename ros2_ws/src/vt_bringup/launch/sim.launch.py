"""Gazebo Harmonic + PX4 SITL (standalone) + XRCE-DDS agent + ros_gz bridge + sensor TFs.

    ros2 launch vt_bringup sim.launch.py world:=/workspace/sim/worlds/generated/course_0.sdf
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

VT_ROOT = Path(os.environ.get("VT_ROOT", "/workspace"))
PX4_DIR = Path(os.environ.get("PX4_DIR", "/opt/PX4-Autopilot"))

# (parent, child, x, y, z, roll, pitch, yaw). Must match sim/models/vt_x500_oakd/model.sdf.
HALF_PI = 1.5707963267948966
STATIC_TFS = [
    ("base_link", "oakd_link", 0.12, 0.0, 0.03, 0.0, 0.0, 0.0),
    ("oakd_link", "oakd_imu", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    ("oakd_link", "oakd_left", 0.0, 0.0375, 0.0, 0.0, 0.0, 0.0),
    ("oakd_link", "oakd_right", 0.0, -0.0375, 0.0, 0.0, 0.0, 0.0),
    ("oakd_left", "oakd_left_optical", 0.0, 0.0, 0.0, -HALF_PI, 0.0, -HALF_PI),
    ("oakd_right", "oakd_right_optical", 0.0, 0.0, 0.0, -HALF_PI, 0.0, -HALF_PI),
]


def load_px4_params(path):
    env = {}
    for line in Path(path).read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if line:
            name, value = (s.strip() for s in line.split("=", 1))
            env[f"PX4_PARAM_{name}"] = value
    return env


def launch_setup(context):
    world = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    share = Path(get_package_share_directory("vt_bringup"))

    resource_path = os.pathsep.join(filter(None, [
        str(VT_ROOT / "sim" / "models"),
        str(PX4_DIR / "Tools" / "simulation" / "gz" / "models"),
        os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    ]))

    gz_cmd = ["gz", "sim", "-r", world]
    if headless:
        gz_cmd[2:2] = ["-s", "--headless-rendering"]
    gazebo = ExecuteProcess(cmd=gz_cmd, additional_env={"GZ_SIM_RESOURCE_PATH": resource_path}, output="screen")

    px4_env = {
        "PX4_GZ_STANDALONE": "1",
        "PX4_SYS_AUTOSTART": "4001",  # gz_x500 airframe
        "PX4_GZ_WORLD": "course",
        "PX4_GZ_MODEL_NAME": "vt_drone",
        "GZ_SIM_RESOURCE_PATH": resource_path,
        **load_px4_params(share / "config" / "px4_sitl.params"),
    }
    px4 = ExecuteProcess(
        cmd=[str(PX4_DIR / "build" / "px4_sitl_default" / "bin" / "px4"), "-d"],
        cwd=str(PX4_DIR),
        additional_env=px4_env,
        output="screen",
    )

    agent = ExecuteProcess(cmd=["MicroXRCEAgent", "udp4", "-p", "8888"], output="screen")

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"config_file": str(share / "config" / "gz_bridge.yaml"), "use_sim_time": True}],
        output="screen",
    )

    tfs = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"tf_{child}",
            arguments=["--x", str(x), "--y", str(y), "--z", str(z),
                       "--roll", str(r), "--pitch", str(p), "--yaw", str(yw),
                       "--frame-id", parent, "--child-frame-id", child],
            parameters=[{"use_sim_time": True}],
        )
        for parent, child, x, y, z, r, p, yw in STATIC_TFS
    ]

    # Give Gazebo a head start so PX4 finds the world.
    return [gazebo, agent, bridge, *tfs, TimerAction(period=5.0, actions=[px4])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=str(VT_ROOT / "sim" / "worlds" / "generated" / "course_0.sdf")),
        DeclareLaunchArgument("headless", default_value="false"),
        OpaqueFunction(function=launch_setup),
    ])
