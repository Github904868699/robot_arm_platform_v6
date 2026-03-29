#pragma once

#include <string>
#include <unordered_map>

#include "robot_arm_hardware_system/types.hpp"

namespace robot_arm_hardware_system
{

class JointRouter
{
public:
  bool load_from_model_config(const std::string & path);

  bool has_route(const std::string & joint_name) const;
  const JointRoute * route_for(const std::string & joint_name) const;
  const JointRouteTable & routes() const;
  std::string last_error() const;

private:
  bool validate_routes();

  std::unordered_map<std::string, JointRoute> route_map_;
  JointRouteTable route_table_;
  std::string last_error_;
};

}  // namespace robot_arm_hardware_system
