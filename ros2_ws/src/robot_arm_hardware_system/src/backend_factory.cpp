#include "robot_arm_hardware_system/backend_factory.hpp"

#include <memory>

#include "robot_arm_hardware_system/fake_robot_hardware_backend.hpp"
#include "robot_arm_hardware_system/real_mixed_robot_backend.hpp"

namespace robot_arm_hardware_system
{

std::unique_ptr<IRobotHardwareBackend> create_backend(const std::string & backend_mode)
{
  if (backend_mode == "real") {
    return std::make_unique<RealMixedRobotBackend>();
  }
  return std::make_unique<FakeRobotHardwareBackend>();
}

}  // namespace robot_arm_hardware_system
