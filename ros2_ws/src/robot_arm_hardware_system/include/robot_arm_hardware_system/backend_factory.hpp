#pragma once

#include <memory>
#include <string>

#include "robot_arm_hardware_system/backend_interface.hpp"

namespace robot_arm_hardware_system
{

std::unique_ptr<IRobotHardwareBackend> create_backend(const std::string & backend_mode);

}  // namespace robot_arm_hardware_system
