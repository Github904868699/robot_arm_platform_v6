#pragma once

#include <string>
#include <vector>

#include "robot_arm_hardware_system/types.hpp"

namespace robot_arm_hardware_system
{

class IRobotHardwareBackend
{
public:
  virtual ~IRobotHardwareBackend() = default;

  virtual bool configure(const JointRouteTable & routes) = 0;
  virtual bool discover_joints() = 0;
  virtual bool sync_current_positions(std::vector<JointState> & states) = 0;
  virtual bool read_all_joint_states(std::vector<JointState> & states) = 0;
  virtual bool write_all_joint_commands(const std::vector<JointCommand> & commands) = 0;

  virtual bool enable() = 0;
  virtual bool disable() = 0;
  virtual bool stop() = 0;
  virtual bool clear_fault() = 0;

  virtual std::string backend_name() const = 0;
};

}  // namespace robot_arm_hardware_system
