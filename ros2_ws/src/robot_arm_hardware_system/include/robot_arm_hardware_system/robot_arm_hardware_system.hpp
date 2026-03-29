#pragma once

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "robot_arm_hardware_system/backend_interface.hpp"
#include "robot_arm_hardware_system/joint_router.hpp"
#include "robot_arm_hardware_system/types.hpp"

namespace robot_arm_hardware_system
{

enum class RuntimeState
{
  DISCOVERING,
  SYNCING_POSITION,
  READY_UNARMED,
  ARMING,
  ARMED_SERVO_HOLD,
  EXECUTING,
  FAULT,
};

class RobotArmHardwareSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  // Control entries (future ROS service callbacks can call these methods).
  bool request_enable();
  bool request_disable();
  bool request_stop();
  bool request_clear_fault();
  bool request_recover();

private:
  void setup_backend_services();
  void teardown_backend_services();
  void spin_backend_services();
  void set_hold_targets_from_current();
  bool check_enable_safety_samples(size_t consecutive_required, int sleep_ms);
  bool load_routing();

  // Upper layers only see logical joints: joint_1 ~ joint_6.
  // This class is a ros2_control skeleton before full real protocol write loop.
  JointRouter router_;
  std::unique_ptr<IRobotHardwareBackend> backend_;

  std::vector<std::string> joint_names_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  std::vector<double> hold_targets_;

  RuntimeState runtime_state_{RuntimeState::DISCOVERING};
  bool enabled_{false};
  std::string backend_mode_{"fake"};
  std::string joint_mapping_path_{};
  bool auto_enable_on_activate_{false};
  double auto_enable_delay_sec_{1.0};

  rclcpp::Node::SharedPtr backend_service_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> backend_service_executor_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr backend_enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr backend_disable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr backend_stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr backend_clear_fault_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr backend_recover_srv_;
};

}  // namespace robot_arm_hardware_system
