from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import subprocess


def generate_launch_description() -> LaunchDescription:
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    backend_mode = LaunchConfiguration("backend_mode")
    joint_mapping_path = LaunchConfiguration("joint_mapping_path")
    auto_enable_on_activate = LaunchConfiguration("auto_enable_on_activate")
    auto_enable_delay_sec = LaunchConfiguration("auto_enable_delay_sec")
    start_hardware_services = LaunchConfiguration("start_hardware_services")
    start_bridge_nodes = LaunchConfiguration("start_bridge_nodes")
    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("robot_arm_moveit_config_4338"), "config", "robot_arm.urdf.xacro"]
            ),
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "backend_mode:=",
            backend_mode,
            " ",
            "joint_mapping_path:=",
            joint_mapping_path,
            " ",
            "auto_enable_on_activate:=",
            auto_enable_on_activate,
            " ",
            "auto_enable_delay_sec:=",
            auto_enable_delay_sec,
        ]
    )

    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_arm_moveit_config_4338"), "launch", "demo.launch.py"]
            )
        ),
        launch_arguments={
            "use_fake_hardware": use_fake_hardware,
            "backend_mode": backend_mode,
            "auto_enable_on_activate": auto_enable_on_activate,
            "auto_enable_delay_sec": auto_enable_delay_sec,
            "robot_description": robot_description,
        }.items(),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            PathJoinSubstitution(
                [FindPackageShare("robot_arm_moveit_config_4338"), "config", "ros2_controllers.yaml"]
            ),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    start_moveit_after_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[
                LogInfo(msg="[bringup] arm_controller spawned, starting MoveIt demo launch."),
                moveit_demo,
            ],
        )
    )

    hardware_services_node = Node(
        package="robot_arm_core",
        executable="hardware_control_services",
        name="robot_arm_hardware_control_services",
        output="screen",
        parameters=[{"joint_mapping_path": joint_mapping_path}],
        condition=IfCondition(start_hardware_services),
    )

    bridge_control_entry_node = Node(
        package="robot_arm_web_bridge",
        executable="ros_control_entry_server",
        name="robot_arm_ros_control_entry_server",
        output="screen",
        condition=IfCondition(start_bridge_nodes),
    )

    bridge_api_node = Node(
        package="robot_arm_web_bridge",
        executable="mock_api_server",
        name="robot_arm_mock_api_server",
        output="screen",
        condition=IfCondition(start_bridge_nodes),
    )

    def _emit_chain_log(context, *args, **kwargs):
        use_fake = LaunchConfiguration("use_fake_hardware").perform(context)
        backend = LaunchConfiguration("backend_mode").perform(context)
        mapping = LaunchConfiguration("joint_mapping_path").perform(context)
        auto_enable = LaunchConfiguration("auto_enable_on_activate").perform(context)
        auto_enable_delay = LaunchConfiguration("auto_enable_delay_sec").perform(context)
        source_xacro = (
            get_package_share_directory("robot_arm_moveit_config_4338")
            + "/config/robot_arm.urdf.xacro"
        )
        rendered = subprocess.run(
            [
                "xacro",
                source_xacro,
                f"use_fake_hardware:={use_fake}",
                f"backend_mode:={backend}",
                f"joint_mapping_path:={mapping}",
                f"auto_enable_on_activate:={auto_enable}",
                f"auto_enable_delay_sec:={auto_enable_delay}",
            ],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
        with open("/tmp/robot_arm_runtime_description.urdf", "w", encoding="utf-8") as f:
            f.write(rendered)
        final_plugin = (
            "mock_components/GenericSystem"
            if "mock_components/GenericSystem" in rendered
            else "robot_arm_hardware_system/RobotArmHardwareSystem"
        )
        expected = (
            "mock_components/GenericSystem"
            if use_fake.lower() == "true"
            else "robot_arm_hardware_system/RobotArmHardwareSystem"
        )
        if use_fake.lower() != "true" and "mock_components/GenericSystem" in rendered:
            raise RuntimeError(
                "[bringup] ERROR: real mode detected mock_components/GenericSystem in runtime description."
            )
        return [
            LogInfo(
                msg=(
                    f"[bringup] use_fake_hardware={use_fake}, backend_mode={backend}, "
                    f"joint_mapping_path={mapping}, auto_enable_on_activate={auto_enable}, "
                    f"auto_enable_delay_sec={auto_enable_delay}, source_xacro={source_xacro}, "
                    f"expected_plugin={expected}, final_plugin={final_plugin}"
                )
            )
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_fake_hardware", default_value="true"),
            DeclareLaunchArgument("backend_mode", default_value="fake"),
            DeclareLaunchArgument("auto_enable_on_activate", default_value="false"),
            DeclareLaunchArgument("auto_enable_delay_sec", default_value="1.0"),
            DeclareLaunchArgument("start_hardware_services", default_value="true"),
            DeclareLaunchArgument("start_bridge_nodes", default_value="true"),
            DeclareLaunchArgument(
                "joint_mapping_path",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("robot_arm_model_4338"), "config", "joint_mapping.example.yaml"]
                ),
            ),
            OpaqueFunction(function=_emit_chain_log),
            robot_state_publisher_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            start_moveit_after_arm_controller,
            hardware_services_node,
            bridge_control_entry_node,
            bridge_api_node,
        ]
    )
