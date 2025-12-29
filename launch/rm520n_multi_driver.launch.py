from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def _make_nodes(context, *args, **kwargs):
    pkg = "rm520n_multi_driver"

    # evaluate LaunchConfiguration
    namespace = LaunchConfiguration("namespace")
    node_name = LaunchConfiguration("node_name")
    log_level = LaunchConfiguration("log_level")
    use_component = LaunchConfiguration("use_component")
    use_intra_process = LaunchConfiguration("use_intra_process")

    params = [LaunchConfiguration("params_file")]
    log_args = ["--ros-args", "--log-level", log_level]

    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace=namespace,
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),  # component_container or component_container_mt
        output="screen",
        emulate_tty=True,
        composable_node_descriptions=[
            ComposableNode(
                package=pkg,
                plugin="rm520n_multi_driver::RM520NMultiDriverNode",
                name=node_name,
                parameters=params,
                extra_arguments=[{"use_intra_process_comms": use_intra_process}],
            )
        ],
        arguments=log_args,
        condition=IfCondition(use_component),
    )

    node = Node(
        package=pkg,
        executable="rm520n_multi_driver_node",
        name=node_name,
        namespace=namespace,
        output="screen",
        emulate_tty=True,
        parameters=params,
        arguments=log_args,
        condition=UnlessCondition(use_component),
    )

    return [container, node]


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare("rm520n_multi_driver"), "config", "rm520n_params.yaml"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="rm520n",
                              description="ROS namespace"),
        DeclareLaunchArgument("node_name", default_value="rm520n_multi_driver",
                              description="Node name"),
        DeclareLaunchArgument("use_component", default_value="false",
                              description="Use rclcpp_components container if true"),
        DeclareLaunchArgument("use_intra_process", default_value="true",
                              description="Enable intra-process comms in component mode"),
        DeclareLaunchArgument("container_name", default_value="rm520n_multi_driver_container",
                              description="Component container name"),
        DeclareLaunchArgument("container_executable", default_value="component_container_mt",
                              description="Container executable (component_container or component_container_mt)"),
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="YAML parameter file"),
        DeclareLaunchArgument("log_level", default_value="info",
                              description="rclcpp log level (debug, info, warn, error, fatal)"),
        OpaqueFunction(function=_make_nodes),
    ])
