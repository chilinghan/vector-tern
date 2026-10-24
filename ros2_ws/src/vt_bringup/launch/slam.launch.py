"""cuVSLAM (stereo + IMU) and the odometry -> PX4 bridge.

    # Stage A (open loop): PX4 flies on ground truth, VSLAM only observes
    ros2 launch vt_bringup slam.launch.py px4_odom_source:=ground_truth
    # Stage B (closed loop): PX4 flies on VSLAM
    ros2 launch vt_bringup slam.launch.py px4_odom_source:=slam
    # No NVIDIA GPU: fly on ground truth without VSLAM
    ros2 launch vt_bringup slam.launch.py run_slam:=false px4_odom_source:=ground_truth
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

ODOM_TOPICS = {
    "slam": "/visual_slam/tracking/odometry",
    "ground_truth": "/vt/ground_truth/odom",
}


def launch_setup(context):
    run_slam = LaunchConfiguration("run_slam").perform(context).lower() == "true"
    source = LaunchConfiguration("px4_odom_source").perform(context)
    if source not in ODOM_TOPICS:
        raise RuntimeError(f"px4_odom_source must be one of {list(ODOM_TOPICS)}, got '{source}'")
    if source == "slam" and not run_slam:
        raise RuntimeError("px4_odom_source:=slam requires run_slam:=true")
    share = Path(get_package_share_directory("vt_bringup"))

    actions = [
        Node(
            package="vt_px4_bridge",
            executable="odom_to_px4",
            parameters=[{"use_sim_time": True}],
            remappings=[("odom_in", ODOM_TOPICS[source])],
            output="screen",
        )
    ]

    if run_slam:
        vslam = ComposableNode(
            name="visual_slam_node",
            package="isaac_ros_visual_slam",
            plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode",
            parameters=[str(share / "config" / "cuvslam.yaml"), {"use_sim_time": True}],
            remappings=[
                ("visual_slam/image_0", "/vt/left/image_raw"),
                ("visual_slam/camera_info_0", "/vt/left/camera_info"),
                ("visual_slam/image_1", "/vt/right/image_raw"),
                ("visual_slam/camera_info_1", "/vt/right/camera_info"),
                ("visual_slam/imu", "/vt/imu"),
            ],
        )
        actions.append(ComposableNodeContainer(
            name="visual_slam_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[vslam],
            output="screen",
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("run_slam", default_value="true"),
        DeclareLaunchArgument("px4_odom_source", default_value="slam", choices=list(ODOM_TOPICS)),
        OpaqueFunction(function=launch_setup),
    ])
