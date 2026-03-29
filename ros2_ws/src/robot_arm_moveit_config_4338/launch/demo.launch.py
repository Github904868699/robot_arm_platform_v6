from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch, generate_moveit_rviz_launch
from ament_index_python.packages import get_package_share_directory
import os


def _launch_setup(context, *args, **kwargs):
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    backend_mode = LaunchConfiguration("backend_mode").perform(context)
    precomputed_robot_description = LaunchConfiguration("robot_description").perform(context).strip()
    source_xacro_path = os.path.join(
        get_package_share_directory("robot_arm_moveit_config_4338"),
        "config",
        "robot_arm.urdf.xacro",
    )

    if not precomputed_robot_description:
        raise RuntimeError(
            "[robot_arm demo.launch] ERROR: robot_description must be provided by top-level bringup."
        )

    moveit_config = (
        MoveItConfigsBuilder("robot_arm", package_name="robot_arm_moveit_config_4338")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    moveit_config.robot_description = {"robot_description": precomputed_robot_description}
    description_source = f"bringup_precomputed({source_xacro_path})"
    resolved_robot_description = precomputed_robot_description

    mode_label = "fake(mock_components/GenericSystem)" if use_fake_hardware.lower() == "true" else "real(robot_arm_hardware_system/RobotArmHardwareSystem)"

    if use_fake_hardware.lower() != "true":
        forbidden_tokens = [
            "libgazebo_ros_control.so",
            "Joint_1",
            "mock_components/GenericSystem",
        ]
        found = [token for token in forbidden_tokens if token in resolved_robot_description]
        if found:
            raise RuntimeError(
                "[robot_arm demo.launch] ERROR: real-mode robot_description still contains "
                f"forbidden legacy tokens: {found}"
            )

    move_group_launch = generate_move_group_launch(moveit_config)
    rviz_launch = generate_moveit_rviz_launch(moveit_config)
    return [
        LogInfo(
            msg=(
                "[robot_arm demo.launch] robot_description_source="
                f"{description_source}, use_fake_hardware={use_fake_hardware}, "
                f"backend_mode={backend_mode}, expected_plugin={mode_label}"
            )
        ),
        *move_group_launch.entities,
        *rviz_launch.entities,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_fake_hardware", default_value="true"),
            DeclareLaunchArgument("backend_mode", default_value="fake"),
            DeclareLaunchArgument("robot_description", default_value=""),
            OpaqueFunction(function=_launch_setup),
        ]
    )
