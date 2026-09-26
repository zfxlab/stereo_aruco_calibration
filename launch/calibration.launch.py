import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("stereo_aruco_calibration")
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("calibration_file", default_value=os.path.join(share, "config", "calibration.yaml")),
        ComposableNodeContainer(
            package="rclcpp_components",
            executable="component_container_mt",
            name="stereo_calibration_pipeline",
            namespace="/",
            output="screen",
            emulate_tty=True,
            composable_node_descriptions=[ComposableNode(
                package="stereo_aruco_calibration",
                plugin="stereo_calibration::StereoArucoCalibratorNode",
                namespace="/stereo_calibration",
                name="calibrator",
                parameters=[
                    LaunchConfiguration("calibration_file"),
                    {"use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool)},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )],
        ),
        Node(
            package="stereo_aruco_calibration",
            executable="stereo_calibration_gui",
            name="stereo_calibration_gui",
            output="screen",
            condition=IfCondition(LaunchConfiguration("gui")),
        ),
    ])
