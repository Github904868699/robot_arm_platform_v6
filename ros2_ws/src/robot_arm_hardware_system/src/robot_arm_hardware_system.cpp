#include "robot_arm_hardware_system/robot_arm_hardware_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_arm_hardware_system/backend_factory.hpp"

namespace robot_arm_hardware_system
{
namespace
{
constexpr double kTwoPi = 6.28318530717958647692;
constexpr double kMotionRejectEpsilonRad = 1e-4;

double turns_to_rad(double turns)
{
  return turns * kTwoPi;
}

double rad_to_turns(double rad)
{
  return rad / kTwoPi;
}

double backend_position_to_ros(const JointRoute * route, double backend_position)
{
  if (route == nullptr) {
    return backend_position;
  }
  const double signed_with_offset = route->direction_sign * (backend_position + route->position_offset);
  // Current adapter-backed backend reports position in turns for both drivers.
  if (route->driver == "hightorque_canfd" || route->driver == "yiyou_can20a") {
    return turns_to_rad(signed_with_offset);
  }
  return signed_with_offset;
}

double backend_velocity_to_ros(const JointRoute * route, double backend_velocity)
{
  if (route == nullptr) {
    return backend_velocity;
  }
  const double signed_velocity = route->direction_sign * backend_velocity;
  // HighTorque velocity: rps -> rad/s
  if (route->driver == "hightorque_canfd") {
    return signed_velocity * kTwoPi;
  }
  // Yiyou velocity: rpm -> rad/s
  if (route->driver == "yiyou_can20a") {
    return signed_velocity * kTwoPi / 60.0;
  }
  return signed_velocity;
}

double ros_position_to_backend(const JointRoute * route, double ros_position_rad)
{
  if (route == nullptr) {
    return ros_position_rad;
  }
  const double ros_turns = rad_to_turns(ros_position_rad);
  const double backend_turns = (ros_turns / route->direction_sign) - route->position_offset;
  // Current adapter-backed backend expects position in turns for both drivers.
  if (route->driver == "hightorque_canfd" || route->driver == "yiyou_can20a") {
    return backend_turns;
  }
  return backend_turns;
}

const char * runtime_state_name(RuntimeState state)
{
  switch (state) {
    case RuntimeState::DISCOVERING:
      return "DISCOVERING";
    case RuntimeState::SYNCING_POSITION:
      return "SYNCING_POSITION";
    case RuntimeState::READY_UNARMED:
      return "READY_UNARMED";
    case RuntimeState::ARMING:
      return "ARMING";
    case RuntimeState::ARMED_SERVO_HOLD:
      return "ARMED_SERVO_HOLD";
    case RuntimeState::EXECUTING:
      return "EXECUTING";
    case RuntimeState::FAULT:
      return "FAULT";
    default:
      return "UNKNOWN";
  }
}

}  // namespace

hardware_interface::CallbackReturn RobotArmHardwareSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotArmHardwareSystem"), "on_init: begin");
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotArmHardwareSystem"), "on_init: base init failed");
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.clear();
  for (const auto & joint : info_.joints) {
    joint_names_.push_back(joint.name);
  }

  if (joint_names_.size() != 6) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotArmHardwareSystem"), "Expected 6 logical joints, got %zu", joint_names_.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!info_.hardware_parameters.count("joint_mapping_path")) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotArmHardwareSystem"), "Missing required hardware parameter: joint_mapping_path");
    return hardware_interface::CallbackReturn::ERROR;
  }

  backend_mode_ = info_.hardware_parameters.count("backend_mode") ?
    info_.hardware_parameters.at("backend_mode") : "fake";
  joint_mapping_path_ = info_.hardware_parameters.at("joint_mapping_path");
  if (info_.hardware_parameters.count("auto_enable_on_activate")) {
    const std::string v = info_.hardware_parameters.at("auto_enable_on_activate");
    auto_enable_on_activate_ = (v == "1" || v == "true" || v == "True" || v == "TRUE");
  } else {
    auto_enable_on_activate_ = false;
  }
  if (info_.hardware_parameters.count("auto_enable_delay_sec")) {
    auto_enable_delay_sec_ = std::max(0.0, std::stod(info_.hardware_parameters.at("auto_enable_delay_sec")));
  } else {
    auto_enable_delay_sec_ = 1.0;
  }
  if (info_.hardware_parameters.count("enable_min_stable_cycles")) {
    enable_min_stable_cycles_ = std::max(1, std::stoi(info_.hardware_parameters.at("enable_min_stable_cycles")));
  }
  if (info_.hardware_parameters.count("enable_wait_timeout_ms")) {
    enable_wait_timeout_ms_ = std::max(100, std::stoi(info_.hardware_parameters.at("enable_wait_timeout_ms")));
  }

  hw_positions_.assign(joint_names_.size(), 0.0);
  hw_velocities_.assign(joint_names_.size(), 0.0);
  hw_position_commands_.assign(joint_names_.size(), 0.0);
  hw_velocity_commands_.assign(joint_names_.size(), 0.0);
  hold_targets_.assign(joint_names_.size(), 0.0);

  runtime_state_ = RuntimeState::DISCOVERING;
  enabled_ = false;
  RCLCPP_INFO(
    rclcpp::get_logger("RobotArmHardwareSystem"),
    "on_init: success backend_mode=%s mapping=%s joints=%zu",
    backend_mode_.c_str(),
    joint_mapping_path_.c_str(),
    joint_names_.size());
  RCLCPP_INFO(
    rclcpp::get_logger("RobotArmHardwareSystem"),
    "on_init: auto_enable_on_activate=%s auto_enable_delay_sec=%.3f enable_min_stable_cycles=%d enable_wait_timeout_ms=%d",
    auto_enable_on_activate_ ? "true" : "false",
    auto_enable_delay_sec_,
    enable_min_stable_cycles_,
    enable_wait_timeout_ms_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotArmHardwareSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotArmHardwareSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]);
    command_interfaces.emplace_back(
      joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_velocity_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn RobotArmHardwareSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotArmHardwareSystem"), "on_configure: begin");
  if (!load_routing()) {
    runtime_state_ = RuntimeState::FAULT;
    RCLCPP_ERROR(rclcpp::get_logger("RobotArmHardwareSystem"), "on_configure: load_routing failed");
    return hardware_interface::CallbackReturn::ERROR;
  }

  backend_ = create_backend(backend_mode_);
  if (!backend_ || !backend_->configure(router_.routes())) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotArmHardwareSystem"), "Failed to configure backend: %s", backend_mode_.c_str());
    runtime_state_ = RuntimeState::FAULT;
    return hardware_interface::CallbackReturn::ERROR;
  }

  runtime_state_ = RuntimeState::DISCOVERING;
  enabled_ = false;
  setup_backend_services();
  RCLCPP_INFO(rclcpp::get_logger("RobotArmHardwareSystem"), "on_configure: success");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotArmHardwareSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobotArmHardwareSystem"), "on_activate: begin");
  runtime_state_ = RuntimeState::DISCOVERING;
  if (!backend_->discover_joints()) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotArmHardwareSystem"), "on_activate: discover_joints failed");
    runtime_state_ = RuntimeState::FAULT;
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("RobotArmHardwareSystem"), "on_activate: discover_joints success");

  runtime_state_ = RuntimeState::SYNCING_POSITION;
  std::vector<JointState> synced;
  if (!backend_->sync_current_positions(synced) || synced.size() != joint_names_.size()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobotArmHardwareSystem"),
      "on_activate: sync_current_positions failed (size=%zu expected=%zu)",
      synced.size(),
      joint_names_.size());
    runtime_state_ = RuntimeState::FAULT;
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("RobotArmHardwareSystem"), "on_activate: sync_current_positions success");

  for (size_t i = 0; i < synced.size(); ++i) {
    const auto * route = router_.route_for(joint_names_[i]);
    hw_positions_[i] = backend_position_to_ros(route, synced[i].position);
    hw_velocities_[i] = backend_velocity_to_ros(route, synced[i].velocity);
    hw_position_commands_[i] = hw_positions_[i];
    hw_velocity_commands_[i] = 0.0;
    RCLCPP_INFO(
      rclcpp::get_logger("RobotArmHardwareSystem"),
      "on_activate: joint=%s online=%s available=%s pos_ros_rad=%.6f vel_ros_rad_s=%.6f",
      joint_names_[i].c_str(),
      synced[i].online ? "true" : "false",
      synced[i].available ? "true" : "false",
      hw_positions_[i],
      hw_velocities_[i]);
  }
  set_hold_targets_from_current();

  // Startup safety: discover + sync only -> READY_UNARMED.
  enabled_ = false;
  runtime_state_ = RuntimeState::READY_UNARMED;
  RCLCPP_INFO(
    rclcpp::get_logger("RobotArmHardwareSystem"),
    "on_activate: success -> READY_UNARMED (enabled=false)");

  if (auto_enable_on_activate_) {
    RCLCPP_INFO(
      rclcpp::get_logger("RobotArmHardwareSystem"),
      "AUTO_ENABLE begin enabled_=%s runtime_state_=%s",
      enabled_ ? "true" : "false",
      runtime_state_name(runtime_state_));
    if (auto_enable_delay_sec_ > 0.0) {
      RCLCPP_INFO(
        rclcpp::get_logger("RobotArmHardwareSystem"),
        "AUTO_ENABLE delayed %.3f sec",
        auto_enable_delay_sec_);
      std::this_thread::sleep_for(std::chrono::duration<double>(auto_enable_delay_sec_));
    }
    if (!request_enable()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("RobotArmHardwareSystem"),
        "AUTO_ENABLE failed: request_enable rejected enabled_=%s runtime_state_=%s",
        enabled_ ? "true" : "false",
        runtime_state_name(runtime_state_));
      return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(
      rclcpp::get_logger("RobotArmHardwareSystem"),
      "AUTO_ENABLE success enabled_=%s runtime_state_=%s",
      enabled_ ? "true" : "false",
      runtime_state_name(runtime_state_));
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotArmHardwareSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  request_disable();
  teardown_backend_services();
  runtime_state_ = RuntimeState::READY_UNARMED;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotArmHardwareSystem::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  spin_backend_services();
  static uint64_t read_cycles = 0;
  ++read_cycles;
  if (!backend_) {
    runtime_state_ = RuntimeState::FAULT;
    return hardware_interface::return_type::ERROR;
  }

  std::vector<JointState> states;
  if (!backend_->read_all_joint_states(states) || states.size() != joint_names_.size()) {
    runtime_state_ = RuntimeState::FAULT;
    return hardware_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < states.size(); ++i) {
    if (states[i].available) {
      const auto * route = router_.route_for(joint_names_[i]);
      hw_positions_[i] = backend_position_to_ros(route, states[i].position);
      hw_velocities_[i] = backend_velocity_to_ros(route, states[i].velocity);
    } else {
      RCLCPP_DEBUG(
        rclcpp::get_logger("RobotArmHardwareSystem"),
        "joint=%s unavailable this cycle (online=%s), keep previous pos=%.6f vel=%.6f",
        joint_names_[i].c_str(),
        states[i].online ? "true" : "false",
        hw_positions_[i],
        hw_velocities_[i]);
    }
  }

  if (read_cycles % 200 == 0) {
    RCLCPP_INFO(
      rclcpp::get_logger("RobotArmHardwareSystem"),
      "read: cycle=%lu updated joint state buffer for %zu joints",
      static_cast<unsigned long>(read_cycles),
      states.size());
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotArmHardwareSystem::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  spin_backend_services();
  if (!backend_) {
    runtime_state_ = RuntimeState::FAULT;
    return hardware_interface::return_type::ERROR;
  }

  if (!enabled_) {
    // READY_UNARMED and earlier states must reject motion writes.
    bool motion_requested = false;
    for (size_t i = 0; i < hw_position_commands_.size(); ++i) {
      if (
        std::abs(hw_position_commands_[i] - hw_positions_[i]) > kMotionRejectEpsilonRad ||
        std::abs(hw_velocity_commands_[i]) > kMotionRejectEpsilonRad)
      {
        motion_requested = true;
        break;
      }
    }
    if (motion_requested) {
      static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("RobotArmHardwareSystem"),
        steady_clock,
        1000,
        "write ignored: system is not enabled (state=READY_UNARMED/earlier), keep current position.");
      // Keep command buffer aligned to current state so controllers won't accumulate stale deltas.
      for (size_t i = 0; i < hw_position_commands_.size(); ++i) {
        hw_position_commands_[i] = hw_positions_[i];
        hw_velocity_commands_[i] = 0.0;
      }
    }
    return hardware_interface::return_type::OK;
  }

  std::vector<JointCommand> commands(joint_names_.size());
  for (size_t i = 0; i < commands.size(); ++i) {
    const auto * route = router_.route_for(joint_names_[i]);
    commands[i].position = ros_position_to_backend(route, hw_position_commands_[i]);

    if (std::isfinite(hw_velocity_commands_[i])) {
      if (route == nullptr) {
        commands[i].velocity = hw_velocity_commands_[i];
      } else if (route->driver == "hightorque_canfd") {
        commands[i].velocity = (hw_velocity_commands_[i] / kTwoPi) / route->direction_sign;
      } else if (route->driver == "yiyou_can20a") {
        commands[i].velocity = (hw_velocity_commands_[i] * 60.0 / kTwoPi) / route->direction_sign;
      } else {
        commands[i].velocity = hw_velocity_commands_[i] / route->direction_sign;
      }
    } else {
      commands[i].velocity = 0.0;
      RCLCPP_DEBUG(
        rclcpp::get_logger("RobotArmHardwareSystem"),
        "write: velocity command nan fallback to 0 joint=%s",
        joint_names_[i].c_str());
    }
  }

  if (!backend_->write_all_joint_commands(commands)) {
    runtime_state_ = RuntimeState::FAULT;
    return hardware_interface::return_type::ERROR;
  }

  runtime_state_ = RuntimeState::EXECUTING;
  return hardware_interface::return_type::OK;
}

bool RobotArmHardwareSystem::request_enable()
{
  RCLCPP_INFO(
    rclcpp::get_logger("RobotArmHardwareSystem"),
    "request_enable: enter enabled_=%s runtime_state_=%s",
    enabled_ ? "true" : "false",
    runtime_state_name(runtime_state_));
  if (!backend_ || runtime_state_ != RuntimeState::READY_UNARMED) {
    RCLCPP_WARN(
      rclcpp::get_logger("RobotArmHardwareSystem"),
      "request_enable: rejected backend_ready=%s runtime_state_=%s",
      backend_ ? "true" : "false",
      runtime_state_name(runtime_state_));
    return false;
  }
  if (!wait_for_stable_samples_before_enable()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobotArmHardwareSystem"),
      "request_enable: rejected because stable valid joint samples are not ready");
    return false;
  }

  runtime_state_ = RuntimeState::ARMING;
  set_hold_targets_from_current();  // seed backend command buffer from current readings for MIT2 hold.

  if (!backend_->enable()) {
    runtime_state_ = RuntimeState::FAULT;
    RCLCPP_ERROR(
      rclcpp::get_logger("RobotArmHardwareSystem"),
      "request_enable: backend enable failed -> runtime_state_=FAULT enabled_=%s",
      enabled_ ? "true" : "false");
    return false;
  }

  enabled_ = true;
  runtime_state_ = RuntimeState::ARMED_SERVO_HOLD;
  RCLCPP_INFO(
    rclcpp::get_logger("RobotArmHardwareSystem"),
    "request_enable: success enabled_=%s runtime_state_=%s",
    enabled_ ? "true" : "false",
    runtime_state_name(runtime_state_));
  return true;
}

bool RobotArmHardwareSystem::wait_for_stable_samples_before_enable()
{
  if (!backend_) {
    return false;
  }
  std::vector<int> stable_cycles(joint_names_.size(), 0);
  const auto t_begin = std::chrono::steady_clock::now();
  while (true) {
    std::vector<JointState> states;
    if (!backend_->read_all_joint_states(states) || states.size() != joint_names_.size()) {
      return false;
    }

    for (size_t i = 0; i < states.size(); ++i) {
      const bool valid =
        states[i].available && states[i].online && !states[i].stale &&
        std::isfinite(states[i].position) && std::isfinite(states[i].velocity);
      stable_cycles[i] = valid ? (stable_cycles[i] + 1) : 0;
    }

    bool all_ready = true;
    for (const int c : stable_cycles) {
      if (c < enable_min_stable_cycles_) {
        all_ready = false;
        break;
      }
    }
    if (all_ready) {
      RCLCPP_INFO(
        rclcpp::get_logger("RobotArmHardwareSystem"),
        "enable gate passed: each joint has >=%d consecutive valid samples",
        enable_min_stable_cycles_);
      return true;
    }

    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - t_begin).count();
    if (elapsed_ms >= enable_wait_timeout_ms_) {
      RCLCPP_WARN(
        rclcpp::get_logger("RobotArmHardwareSystem"),
        "enable gate timeout: waited %ldms but stable samples not ready",
        static_cast<long>(elapsed_ms));
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

bool RobotArmHardwareSystem::request_disable()
{
  RCLCPP_INFO(
    rclcpp::get_logger("RobotArmHardwareSystem"),
    "request_disable: enter enabled_=%s runtime_state_=%s",
    enabled_ ? "true" : "false",
    runtime_state_name(runtime_state_));
  if (!backend_) {
    RCLCPP_WARN(rclcpp::get_logger("RobotArmHardwareSystem"), "request_disable: rejected backend missing");
    return false;
  }

  backend_->disable();
  enabled_ = false;
  runtime_state_ = RuntimeState::READY_UNARMED;
  RCLCPP_INFO(
    rclcpp::get_logger("RobotArmHardwareSystem"),
    "request_disable: success enabled_=%s runtime_state_=%s",
    enabled_ ? "true" : "false",
    runtime_state_name(runtime_state_));
  return true;
}

bool RobotArmHardwareSystem::request_stop()
{
  RCLCPP_INFO(
    rclcpp::get_logger("RobotArmHardwareSystem"),
    "request_stop: enter enabled_=%s runtime_state_=%s",
    enabled_ ? "true" : "false",
    runtime_state_name(runtime_state_));
  if (!backend_) {
    RCLCPP_WARN(rclcpp::get_logger("RobotArmHardwareSystem"), "request_stop: rejected backend missing");
    return false;
  }

  backend_->stop();
  enabled_ = false;
  runtime_state_ = RuntimeState::FAULT;
  RCLCPP_INFO(
    rclcpp::get_logger("RobotArmHardwareSystem"),
    "request_stop: success enabled_=%s runtime_state_=%s",
    enabled_ ? "true" : "false",
    runtime_state_name(runtime_state_));
  return true;
}

bool RobotArmHardwareSystem::request_clear_fault()
{
  RCLCPP_INFO(
    rclcpp::get_logger("RobotArmHardwareSystem"),
    "request_clear_fault: enter enabled_=%s runtime_state_=%s",
    enabled_ ? "true" : "false",
    runtime_state_name(runtime_state_));
  if (!backend_) {
    return false;
  }

  if (!backend_->clear_fault()) {
    return false;
  }

  if (runtime_state_ == RuntimeState::FAULT) {
    runtime_state_ = RuntimeState::READY_UNARMED;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("RobotArmHardwareSystem"),
    "request_clear_fault: success enabled_=%s runtime_state_=%s",
    enabled_ ? "true" : "false",
    runtime_state_name(runtime_state_));
  return true;
}

bool RobotArmHardwareSystem::request_recover()
{
  RCLCPP_INFO(
    rclcpp::get_logger("RobotArmHardwareSystem"),
    "request_recover: enter enabled_=%s runtime_state_=%s",
    enabled_ ? "true" : "false",
    runtime_state_name(runtime_state_));
  if (runtime_state_ != RuntimeState::FAULT) {
    RCLCPP_WARN(
      rclcpp::get_logger("RobotArmHardwareSystem"),
      "request_recover: rejected runtime_state_=%s (expect FAULT)",
      runtime_state_name(runtime_state_));
    return false;
  }

  if (!request_clear_fault()) {
    return false;
  }

  enabled_ = false;
  runtime_state_ = RuntimeState::READY_UNARMED;
  RCLCPP_INFO(
    rclcpp::get_logger("RobotArmHardwareSystem"),
    "request_recover: success enabled_=%s runtime_state_=%s",
    enabled_ ? "true" : "false",
    runtime_state_name(runtime_state_));
  return true;
}

bool RobotArmHardwareSystem::load_routing()
{
  const bool loaded = router_.load_from_model_config(joint_mapping_path_);
  if (!loaded) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobotArmHardwareSystem"),
      "Failed to load/validate joint mapping from '%s': %s",
      joint_mapping_path_.c_str(), router_.last_error().c_str());
    return false;
  }

  for (const auto & joint_name : joint_names_) {
    if (!router_.has_route(joint_name)) {
      RCLCPP_ERROR(rclcpp::get_logger("RobotArmHardwareSystem"), "Route missing for joint: %s", joint_name.c_str());
      return false;
    }
  }
  return true;
}

void RobotArmHardwareSystem::set_hold_targets_from_current()
{
  std::copy(hw_positions_.begin(), hw_positions_.end(), hold_targets_.begin());
  hw_position_commands_ = hold_targets_;
  std::fill(hw_velocity_commands_.begin(), hw_velocity_commands_.end(), 0.0);
}

void RobotArmHardwareSystem::setup_backend_services()
{
  if (backend_service_node_) {
    return;
  }

  backend_service_node_ = std::make_shared<rclcpp::Node>("robot_arm_hardware_backend_services");
  backend_service_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  backend_service_executor_->add_node(backend_service_node_);

  backend_enable_srv_ = backend_service_node_->create_service<std_srvs::srv::Trigger>(
    "/robot_arm/hardware_backend/enable",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      response->success = request_enable();
      response->message = response->success ? "enabled" : "enable rejected";
    });

  backend_disable_srv_ = backend_service_node_->create_service<std_srvs::srv::Trigger>(
    "/robot_arm/hardware_backend/disable",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      response->success = request_disable();
      response->message = response->success ? "disabled" : "disable failed";
    });

  backend_stop_srv_ = backend_service_node_->create_service<std_srvs::srv::Trigger>(
    "/robot_arm/hardware_backend/stop",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      response->success = request_stop();
      response->message = response->success ? "stopped" : "stop failed";
    });

  backend_clear_fault_srv_ = backend_service_node_->create_service<std_srvs::srv::Trigger>(
    "/robot_arm/hardware_backend/clear_fault",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      response->success = request_clear_fault();
      response->message = response->success ? "fault cleared" : "clear fault failed";
    });

  backend_recover_srv_ = backend_service_node_->create_service<std_srvs::srv::Trigger>(
    "/robot_arm/hardware_backend/recover",
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      response->success = request_recover();
      response->message = response->success ? "recovered" : "recover rejected";
    });
}

void RobotArmHardwareSystem::teardown_backend_services()
{
  backend_enable_srv_.reset();
  backend_disable_srv_.reset();
  backend_stop_srv_.reset();
  backend_clear_fault_srv_.reset();
  backend_recover_srv_.reset();

  if (backend_service_executor_ && backend_service_node_) {
    backend_service_executor_->remove_node(backend_service_node_);
  }
  backend_service_executor_.reset();
  backend_service_node_.reset();
}

void RobotArmHardwareSystem::spin_backend_services()
{
  if (backend_service_executor_) {
    backend_service_executor_->spin_some();
  }
}

}  // namespace robot_arm_hardware_system

PLUGINLIB_EXPORT_CLASS(
  robot_arm_hardware_system::RobotArmHardwareSystem,
  hardware_interface::SystemInterface)
