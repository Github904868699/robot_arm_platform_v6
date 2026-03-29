#include "robot_arm_hardware_system/joint_router.hpp"

#include <array>
#include <fstream>
#include <set>
#include <sstream>
#include <string>

namespace robot_arm_hardware_system
{

namespace
{
std::string trim(const std::string & input)
{
  const auto first = input.find_first_not_of(" \t\n\r");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = input.find_last_not_of(" \t\n\r");
  return input.substr(first, last - first + 1);
}
}  // namespace

bool JointRouter::load_from_model_config(const std::string & path)
{
  route_map_.clear();
  route_table_.clear();
  last_error_.clear();

  std::ifstream fin(path);
  if (!fin.is_open()) {
    last_error_ = "failed_to_open_mapping_file";
    return false;
  }

  JointRoute current;
  bool in_joint = false;

  std::string line;
  while (std::getline(fin, line)) {
    const auto t = trim(line);
    if (t.empty() || t[0] == '#') {
      continue;
    }

    if (t.rfind("joint_", 0) == 0 && t.back() == ':') {
      if (in_joint && !current.joint_name.empty()) {
        route_map_[current.joint_name] = current;
        route_table_.push_back(current);
      }
      current = {};
      current.joint_name = t.substr(0, t.size() - 1);
      in_joint = true;
      continue;
    }

    if (!in_joint) {
      continue;
    }

    if (t.rfind("driver:", 0) == 0) {
      current.driver = trim(t.substr(7));
    } else if (t.rfind("bus:", 0) == 0) {
      current.bus = trim(t.substr(4));
    } else if (t.rfind("node_id:", 0) == 0) {
      current.node_id = std::stoi(trim(t.substr(8)));
    } else if (t.rfind("direction_sign:", 0) == 0) {
      current.direction_sign = std::stod(trim(t.substr(15)));
    } else if (t.rfind("position_offset:", 0) == 0) {
      current.position_offset = std::stod(trim(t.substr(16)));
    }
  }

  if (in_joint && !current.joint_name.empty()) {
    route_map_[current.joint_name] = current;
    route_table_.push_back(current);
  }

  return validate_routes();
}

bool JointRouter::validate_routes()
{
  if (route_table_.size() != 6) {
    last_error_ = "mapping_must_contain_exactly_6_joints";
    return false;
  }

  const std::array<std::string, 6> expected = {
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

  for (const auto & name : expected) {
    if (route_map_.find(name) == route_map_.end()) {
      last_error_ = "missing_expected_joint:" + name;
      return false;
    }
  }

  std::set<std::string> driver_bus_node_guard;
  for (const auto & route : route_table_) {
    if (route.node_id <= 0) {
      last_error_ = "invalid_node_id:" + route.joint_name;
      return false;
    }
    if (route.direction_sign != 1.0 && route.direction_sign != -1.0) {
      last_error_ = "direction_sign_must_be_plus_or_minus_one:" + route.joint_name;
      return false;
    }

    const std::string key = route.driver + "|" + route.bus + "|" + std::to_string(route.node_id);
    if (!driver_bus_node_guard.insert(key).second) {
      last_error_ = "duplicate_node_id_in_same_driver_bus:" + key;
      return false;
    }
  }

  const auto * j2 = route_for("joint_2");
  if (!j2 || j2->driver != "yiyou_can20a") {
    last_error_ = "joint_2_must_route_to_yiyou_can20a";
    return false;
  }

  for (const auto & name : {"joint_1", "joint_3", "joint_4", "joint_5", "joint_6"}) {
    const auto * route = route_for(name);
    if (!route || route->driver != "hightorque_canfd") {
      last_error_ = std::string(name) + "_must_route_to_hightorque_canfd";
      return false;
    }
  }

  return true;
}

bool JointRouter::has_route(const std::string & joint_name) const
{
  return route_map_.find(joint_name) != route_map_.end();
}

const JointRoute * JointRouter::route_for(const std::string & joint_name) const
{
  const auto it = route_map_.find(joint_name);
  if (it == route_map_.end()) {
    return nullptr;
  }
  return &it->second;
}

const JointRouteTable & JointRouter::routes() const
{
  return route_table_;
}

std::string JointRouter::last_error() const
{
  return last_error_;
}

}  // namespace robot_arm_hardware_system
