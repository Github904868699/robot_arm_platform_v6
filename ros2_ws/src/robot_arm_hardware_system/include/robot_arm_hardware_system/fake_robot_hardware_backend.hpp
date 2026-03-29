#pragma once

#include "robot_arm_hardware_system/backend_interface.hpp"

namespace robot_arm_hardware_system
{

class FakeRobotHardwareBackend : public IRobotHardwareBackend
{
public:
  bool configure(const JointRouteTable & routes) override;
  bool discover_joints() override;
  bool sync_current_positions(std::vector<JointState> & states) override;
  bool read_all_joint_states(std::vector<JointState> & states) override;
  bool write_all_joint_commands(const std::vector<JointCommand> & commands) override;

  bool enable() override;
  bool disable() override;
  bool stop() override;
  bool clear_fault() override;

  std::string backend_name() const override { return "fake"; }

private:
  JointRouteTable routes_;
  std::vector<JointState> fake_states_;
  bool enabled_{false};
};

}  // namespace robot_arm_hardware_system
