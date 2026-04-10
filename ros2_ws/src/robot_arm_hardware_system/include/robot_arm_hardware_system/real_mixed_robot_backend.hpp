#pragma once

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "robot_arm_hardware_system/backend_interface.hpp"

namespace robot_arm_hardware_system
{

struct HightorqueMit2Config
{
  std::string model{"M4438_30"};
  double kp{25.0};
  double kd{1.5};
  double tqe_nm{0.0};
  double max_velocity_rps{2.0};
  double vel_lpf_alpha{0.2};
  int write_period_ms{20};
};

struct HightorqueDesiredCommand
{
  double position_turns{0.0};
  double velocity_rps{0.0};
  bool has_velocity{false};
  double stamp_sec{0.0};
  bool valid{false};
};

inline void maybe_override_joint_mit2_model(const std::string & joint_name, std::string & model)
{
  const std::string specific = "ROBOT_ARM_HT_" + joint_name + "_MIT2_MODEL";
  if (const char * v = std::getenv(specific.c_str())) {
    if (std::strlen(v) > 0) {
      model = std::string(v);
      return;
    }
  }
  const std::string generic = "ROBOT_ARM_HT_MIT2_MODEL_" + joint_name;
  if (const char * v = std::getenv(generic.c_str())) {
    if (std::strlen(v) > 0) {
      model = std::string(v);
    }
  }
}

inline void maybe_override_joint_mit2_param(const std::string & joint_name, const char * suffix, double & value)
{
  const std::string specific = "ROBOT_ARM_HT_" + joint_name + "_MIT2_" + suffix;
  if (const char * v = std::getenv(specific.c_str())) {
    value = std::atof(v);
    return;
  }
  const std::string generic = "ROBOT_ARM_HT_MIT2_" + std::string(suffix) + "_" + joint_name;
  if (const char * v = std::getenv(generic.c_str())) {
    value = std::atof(v);
  }
}

inline void maybe_override_joint_mit2_param(const std::string & joint_name, const char * suffix, int & value)
{
  const std::string specific = "ROBOT_ARM_HT_" + joint_name + "_MIT2_" + suffix;
  if (const char * v = std::getenv(specific.c_str())) {
    value = std::max(1, std::atoi(v));
    return;
  }
  const std::string generic = "ROBOT_ARM_HT_MIT2_" + std::string(suffix) + "_" + joint_name;
  if (const char * v = std::getenv(generic.c_str())) {
    value = std::max(1, std::atoi(v));
  }
}

struct HightorqueReadChannel
{
  std::string bridge_device{"/dev/ttyACM0"};
  bool debug_single_joint{false};
  std::string debug_joint_name{"joint_1"};
  int fd{-1};
  bool initialized{false};
  std::unordered_map<std::string, JointState> latest_states;
  uint64_t read_attempts{0};
  uint64_t frames_seen{0};
  uint64_t valid_samples{0};
  uint64_t send_failures{0};
  uint64_t recv_timeouts{0};
  uint64_t parse_failures{0};
  bool strict_replay_done{false};
  std::mutex io_mutex;

  bool discover(const JointRouteTable & routes, std::unordered_map<std::string, bool> & seen);
  bool sync_current_positions(const JointRouteTable & routes, std::unordered_map<std::string, JointState> & out_states);
  bool read_joint(const JointRoute & route, JointState & out_state);
};

struct YiyouReadChannel
{
  std::string bridge_device{"/dev/ttyACM1"};
  int fd{-1};
  bool initialized{false};
  std::optional<JointState> latest_joint2_state;
  uint64_t read_attempts{0};
  uint64_t frames_seen{0};
  uint64_t valid_samples{0};
  uint64_t send_failures{0};
  uint64_t recv_timeouts{0};
  uint64_t parse_failures{0};
  bool strict_replay_done{false};
  std::mutex io_mutex;

  bool discover(const JointRouteTable & routes, std::unordered_map<std::string, bool> & seen);
  bool sync_current_positions(const JointRouteTable & routes, std::unordered_map<std::string, JointState> & out_states);
  bool read_joint(const JointRoute & route, JointState & out_state);
};

class RealMixedRobotBackend : public IRobotHardwareBackend
{
public:
  ~RealMixedRobotBackend() override;

  bool configure(const JointRouteTable & routes) override;
  bool discover_joints() override;
  bool sync_current_positions(std::vector<JointState> & states) override;
  bool read_all_joint_states(std::vector<JointState> & states) override;
  bool write_all_joint_commands(const std::vector<JointCommand> & commands) override;

  bool enable() override;
  bool disable() override;
  bool stop() override;
  bool clear_fault() override;
  void set_hold_seed_snapshot(
    const std::vector<JointCommand> & commands, const std::string & source) override;
  void set_step_transition_enabled(bool enabled, const std::string & reason) override;

  std::string backend_name() const override { return "real_mixed_readonly_placeholder"; }

private:
  enum class HightorqueControlMode
  {
    DISABLED,
    HOLD_ACTIVE,
    STEP_MOVE_ACTIVE,
  };

  void start_polling_worker();
  void stop_polling_worker();
  void polling_loop_hightorque();
  void polling_loop_yiyou();
  void hightorque_tx_loop();
  void write_cache_locked(const std::string & joint_name, const JointState & state);
  bool resolve_bridge_devices();
  bool probe_hightorque_on_device(const std::string & device, const std::set<int> & node_ids);
  bool probe_yiyou_on_device(const std::string & device, int node_id);
  bool get_recent_last_good_state(const std::string & joint_name, JointState & out_state, double max_age_sec);

  bool discover_hightorque_joints();
  bool discover_yiyou_joints();
  bool send_hightorque_action(
    const JointRoute & route, const std::string & semantic, const std::string & frame, bool wait_reply);
  bool send_yiyou_write_u32(const JointRoute & route, uint8_t reg_addr, int32_t value, const std::string & semantic);
  bool read_latest_position_turns(const std::string & joint_name, double & position_turns);

  JointRouteTable routes_;
  JointRouteTable hightorque_routes_;
  JointRouteTable yiyou_routes_;
  std::unordered_map<std::string, bool> discovered_;
  HightorqueReadChannel hightorque_channel_;
  YiyouReadChannel yiyou_channel_;
  std::unordered_map<std::string, JointState> latest_cache_;
  std::unordered_map<std::string, JointState> latest_poll_result_;
  std::unordered_map<std::string, JointState> last_good_cache_;
  std::unordered_map<std::string, double> last_good_time_sec_;
  std::mutex cache_mutex_;
  std::thread hightorque_polling_thread_;
  std::thread yiyou_polling_thread_;
  std::thread hightorque_tx_thread_;
  std::atomic<bool> polling_running_{false};
  std::atomic<bool> hightorque_tx_running_{false};
  std::mutex yiyou_tx_mutex_;
  std::unordered_map<std::string, double> yiyou_desired_position_;
  std::unordered_map<std::string, double> yiyou_last_sent_position_;
  std::unordered_map<std::string, double> yiyou_last_send_time_sec_;

  std::unordered_map<std::string, double> last_command_position_;
  std::unordered_map<std::string, double> hightorque_hold_targets_;
  std::unordered_map<std::string, bool> hightorque_hold_ready_;

  std::mutex hightorque_tx_mutex_;
  std::condition_variable hightorque_tx_cv_;
  bool hightorque_tx_kick_{false};
  uint64_t hightorque_tx_wakeup_command_{0};
  uint64_t hightorque_tx_wakeup_periodic_{0};
  std::unordered_map<std::string, HightorqueDesiredCommand> hightorque_desired_commands_;
  HightorqueControlMode hightorque_control_mode_{HightorqueControlMode::DISABLED};
  bool hightorque_allow_step_transition_{false};
  double hightorque_last_step_command_sec_{0.0};
  std::unordered_map<std::string, double> hightorque_last_sent_position_;
  std::unordered_map<std::string, double> hightorque_last_send_time_sec_;
  std::unordered_map<std::string, double> hightorque_filtered_velocity_;
  std::unordered_map<std::string, double> hightorque_last_velocity_target_;
  std::unordered_map<std::string, double> hightorque_last_velocity_target_time_sec_;
  HightorqueMit2Config hightorque_mit2_config_{};
  std::unordered_map<std::string, HightorqueMit2Config> hightorque_joint_mit2_config_;

  bool write_path_warned_{false};
  std::string hightorque_command_family_{"mode2_velocity_unlimited"};
  bool hightorque_position_hold_supported_{false};
  bool hightorque_mode_log_once_{false};
  bool hightorque_position_hold_active_{false};
  int hightorque_poll_period_ms_armed_{10};
  int hightorque_poll_period_ms_unarmed_{10};
  double sample_recency_window_sec_{0.5};
  int sync_wait_timeout_ms_{2000};
  std::atomic<bool> enabled_{false};
  std::atomic<bool> faulted_{false};
};

}  // namespace robot_arm_hardware_system
