#include "robot_arm_hardware_system/fake_robot_hardware_backend.hpp"

#include <cstddef>

namespace robot_arm_hardware_system
{

bool FakeRobotHardwareBackend::configure(const JointRouteTable & routes)
{
  routes_ = routes;
  fake_states_.assign(routes_.size(), JointState{});
  for (auto & state : fake_states_) {
    state.available = true;
    state.online = true;
  }
  enabled_ = false;
  return routes_.size() == 6;
}

bool FakeRobotHardwareBackend::discover_joints()
{
  return routes_.size() == 6;
}

bool FakeRobotHardwareBackend::sync_current_positions(std::vector<JointState> & states)
{
  states = fake_states_;
  return true;
}

bool FakeRobotHardwareBackend::read_all_joint_states(std::vector<JointState> & states)
{
  states = fake_states_;
  return true;
}

bool FakeRobotHardwareBackend::write_all_joint_commands(const std::vector<JointCommand> & commands)
{
  if (!enabled_ || commands.size() != fake_states_.size()) {
    return false;
  }

  for (size_t i = 0; i < commands.size(); ++i) {
    const double delta = commands[i].position - fake_states_[i].position;
    fake_states_[i].velocity = delta;
    fake_states_[i].position = commands[i].position;
  }
  return true;
}

bool FakeRobotHardwareBackend::enable()
{
  enabled_ = true;
  return true;
}

bool FakeRobotHardwareBackend::disable()
{
  enabled_ = false;
  return true;
}

bool FakeRobotHardwareBackend::stop()
{
  for (auto & state : fake_states_) {
    state.velocity = 0.0;
  }
  enabled_ = false;
  return true;
}

bool FakeRobotHardwareBackend::clear_fault()
{
  return true;
}

}  // namespace robot_arm_hardware_system
