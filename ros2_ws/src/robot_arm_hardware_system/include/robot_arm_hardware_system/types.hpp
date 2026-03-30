#pragma once

#include <string>
#include <limits>
#include <vector>

namespace robot_arm_hardware_system
{

struct JointState
{
  double position{0.0};
  double velocity{0.0};
  bool available{false};  // true: this cycle has a valid read; false: no fresh valid read this cycle.
  bool online{false};     // true: transport/frame observed; false: channel unavailable or no frame.
  double last_update_time_sec{0.0};
  bool stale{true};
  std::string source{"none"};
  std::string last_error;
};

struct JointCommand
{
  double position{0.0};
  double velocity{std::numeric_limits<double>::quiet_NaN()};
};

struct JointRoute
{
  std::string joint_name;
  std::string driver;
  std::string bus;
  int node_id{0};
  double direction_sign{1.0};   // model-level sign: +1 or -1
  double position_offset{0.0};  // model-level offset in turns
};

using JointRouteTable = std::vector<JointRoute>;

}  // namespace robot_arm_hardware_system
