#include "robot_arm_hardware_system/real_mixed_robot_backend.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <iomanip>
#include <fcntl.h>
#include <filesystem>
#include <optional>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>
#include <unordered_map>
#include <vector>
#include <thread>

#include "rclcpp/rclcpp.hpp"

namespace robot_arm_hardware_system
{
namespace
{
constexpr size_t kReadBufSize = 512;
constexpr int kSerialTimeoutMs = 0;
constexpr int kWriteTimeoutMs = 1000;
constexpr int kReadWindowMs = 80;
constexpr int kReadIdleGapMs = 10;

double now_monotonic_sec()
{
  using clock = std::chrono::steady_clock;
  const auto now = clock::now().time_since_epoch();
  return std::chrono::duration<double>(now).count();
}

std::optional<double> extract_numeric(const std::string & text, const std::vector<std::string> & keys)
{
  for (const auto & key : keys) {
    const std::regex pattern(key + R"((?:\s*[:=]\s*|\s+)(-?\d+(?:\.\d+)?))", std::regex::icase);
    std::smatch match;
    if (std::regex_search(text, match, pattern)) {
      try {
        return std::stod(match[1].str());
      } catch (...) {
      }
    }
  }
  return std::nullopt;
}

std::string hex_digest(const std::string & data)
{
  static const char * kHex = "0123456789ABCDEF";
  constexpr size_t kMaxBytes = 24;
  std::string out;
  const size_t n = std::min(kMaxBytes, data.size());
  out.reserve(n * 3);
  for (size_t i = 0; i < n; ++i) {
    const unsigned char c = static_cast<unsigned char>(data[i]);
    out.push_back(kHex[(c >> 4) & 0x0F]);
    out.push_back(kHex[c & 0x0F]);
    if (i + 1 < n) {
      out.push_back(' ');
    }
  }
  return out;
}

std::string ascii_escaped(const std::string & data)
{
  std::string out;
  out.reserve(data.size() * 2);
  for (const char c : data) {
    switch (c) {
      case '\r':
        out += "\\r";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        out.push_back(c);
        break;
    }
  }
  return out;
}

std::string trim_crlf(std::string s)
{
  while (!s.empty() && (s.back() == '\r' || s.back() == '\n')) {
    s.pop_back();
  }
  return s;
}

bool is_hightorque_bridge_ack(const std::string & raw)
{
  const std::string token = trim_crlf(raw);
  if (token.size() != 5) {
    return false;
  }
  if (token[0] != 'd' && token[0] != 'D') {
    return false;
  }
  const auto is_hex = [](char c) {
      return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f');
    };
  return is_hex(token[1]) && token[2] == '0' && token[3] == '0' && token[4] == '0';
}

uint8_t hightorque_node_id_from_can_id(uint32_t can_id)
{
  const uint8_t node_hi = static_cast<uint8_t>((can_id >> 8) & 0xFFu);
  if (node_hi != 0) {
    return node_hi;
  }
  return static_cast<uint8_t>(can_id & 0xFFu);
}

std::string to_hex_upper(uint32_t value, int width)
{
  std::ostringstream oss;
  oss << std::uppercase << std::hex << std::setfill('0') << std::setw(width) << value;
  return oss.str();
}

std::string build_slcan_t_frame(uint16_t can_id, const std::vector<uint8_t> & data)
{
  std::string frame;
  frame.reserve(1 + 3 + 1 + data.size() * 2 + 1);
  frame += 't';
  frame += to_hex_upper(can_id & 0x7FF, 3);
  frame += to_hex_upper(static_cast<uint32_t>(data.size()) & 0x0F, 1);
  for (const auto byte : data) {
    frame += to_hex_upper(byte, 2);
  }
  frame += '\r';
  return frame;
}

struct SlcanStdFrame
{
  uint16_t can_id{0};
  std::vector<uint8_t> payload;
};

bool parse_slcan_t_frame(const std::string & raw, SlcanStdFrame & out, std::string & error_reason)
{
  std::string frame = trim_crlf(raw);
  auto pos = frame.find('t');
  if (pos == std::string::npos) {
    pos = frame.find('T');
  }
  if (pos == std::string::npos) {
    error_reason = "no_t_prefix";
    return false;
  }
  frame = frame.substr(pos);
  if (frame.size() < 5) {
    error_reason = "short_reply";
    return false;
  }
  try {
    out.can_id = static_cast<uint16_t>(std::stoul(frame.substr(1, 3), nullptr, 16) & 0x7FFu);
    const size_t dlc = static_cast<size_t>(std::stoul(frame.substr(4, 1), nullptr, 16) & 0x0Fu);
    const size_t need_len = 5 + dlc * 2;
    if (frame.size() < need_len) {
      error_reason = "short_reply";
      return false;
    }
    out.payload.clear();
    out.payload.reserve(dlc);
    for (size_t i = 0; i < dlc; ++i) {
      const size_t byte_pos = 5 + i * 2;
      out.payload.push_back(static_cast<uint8_t>(std::stoul(frame.substr(byte_pos, 2), nullptr, 16) & 0xFFu));
    }
    return true;
  } catch (...) {
    error_reason = "malformed_reply";
    return false;
  }
}

bool parse_slcan_D_payload(const std::string & raw, std::vector<uint8_t> & payload, uint32_t & can_id, std::string & reason)
{
  const auto decode_canfd_len = [](unsigned int dlc_code) -> size_t {
      switch (dlc_code & 0x0Fu) {
        case 0x0: return 0;
        case 0x1: return 1;
        case 0x2: return 2;
        case 0x3: return 3;
        case 0x4: return 4;
        case 0x5: return 5;
        case 0x6: return 6;
        case 0x7: return 7;
        case 0x8: return 8;
        case 0x9: return 12;
        case 0xA: return 16;
        case 0xB: return 20;
        case 0xC: return 24;
        case 0xD: return 32;
        case 0xE: return 48;
        case 0xF: return 64;
        default: return 0;
      }
    };

  std::string frame = trim_crlf(raw);
  std::vector<size_t> candidates;
  for (size_t i = 0; i < frame.size(); ++i) {
    if (frame[i] == 'D' || frame[i] == 'd') {
      candidates.push_back(i);
    }
  }
  if (candidates.empty()) {
    reason = "no_D_or_d_prefix";
    return false;
  }

  for (const auto pos : candidates) {
    const std::string sub = frame.substr(pos);
    const bool is_extended = sub[0] == 'D';
    const bool is_standard = sub[0] == 'd';
    const size_t can_id_hex_len = is_extended ? 8 : 3;
    if (!(is_extended || is_standard)) {
      continue;
    }
    if (sub.size() < 1 + can_id_hex_len + 1) {
      continue;
    }
    try {
      const uint32_t parsed_can_id = static_cast<uint32_t>(
        std::stoul(sub.substr(1, can_id_hex_len), nullptr, 16));
      const unsigned int dlc_code = static_cast<unsigned int>(
        std::stoul(sub.substr(1 + can_id_hex_len, 1), nullptr, 16) & 0x0Fu);
      const size_t payload_len = decode_canfd_len(dlc_code);
      const size_t need_len = 1 + can_id_hex_len + 1 + payload_len * 2;
      if (sub.size() < need_len) {
        continue;
      }
      payload.clear();
      payload.reserve(payload_len);
      const size_t payload_hex_pos = 1 + can_id_hex_len + 1;
      for (size_t i = 0; i < payload_len; ++i) {
        payload.push_back(static_cast<uint8_t>(
          std::stoul(sub.substr(payload_hex_pos + i * 2, 2), nullptr, 16) & 0xFFu));
      }
      can_id = parsed_can_id;
      reason = is_standard ? "parsed_ok_standard_canfd" : "parsed_ok_extended_canfd";
      return true;
    } catch (...) {
      continue;
    }
  }

  reason = "short_reply";
  return false;
}

uint8_t encode_canfd_dlc(size_t payload_len)
{
  switch (payload_len) {
    case 0: return 0x0;
    case 1: return 0x1;
    case 2: return 0x2;
    case 3: return 0x3;
    case 4: return 0x4;
    case 5: return 0x5;
    case 6: return 0x6;
    case 7: return 0x7;
    case 8: return 0x8;
    case 12: return 0x9;
    case 16: return 0xA;
    case 20: return 0xB;
    case 24: return 0xC;
    case 32: return 0xD;
    case 48: return 0xE;
    case 64: return 0xF;
    default: throw std::runtime_error("unsupported CAN-FD payload length");
  }
}

std::string build_slcan_D_frame(uint32_t can_id, const std::vector<uint8_t> & data)
{
  std::string frame;
  frame.reserve(1 + 8 + 1 + data.size() * 2 + 1);
  frame += 'D';
  frame += to_hex_upper(can_id, 8);
  frame += to_hex_upper(static_cast<uint32_t>(encode_canfd_dlc(data.size())) & 0x0F, 1);
  for (const auto byte : data) {
    frame += to_hex_upper(byte, 2);
  }
  frame += '\r';
  return frame;
}

std::string build_hightorque_read_query(const JointRoute & route)
{
  // Official H730 read_motor_state_int32: payload={0x18,0x04,0x00,0x11,0x0F}.
  const uint32_t can_id = 0x8000u | static_cast<uint32_t>(route.node_id & 0xFF);
  return build_slcan_D_frame(can_id, {0x18, 0x04, 0x00, 0x11, 0x0F});
}

std::string build_hightorque_full_status_query(const JointRoute & route)
{
  // Follow-up query after bridge ACK, used to fetch full status response.
  const uint32_t can_id = 0x8000u | static_cast<uint32_t>(route.node_id & 0xFF);
  return build_slcan_D_frame(can_id, {0x1C, 0x04, 0x00, 0x11, 0x0F});
}

std::string build_yiyou_read_query(const JointRoute & route, uint8_t reg_addr)
{
  return build_slcan_t_frame(static_cast<uint16_t>(route.node_id & 0x7FF), {0x03, reg_addr});
}

std::string build_yiyou_write_u32_query(const JointRoute & route, uint8_t reg_addr, int32_t value)
{
  const uint32_t be = static_cast<uint32_t>(value);
  return build_slcan_t_frame(
    static_cast<uint16_t>(route.node_id & 0x7FF),
    {0x01, reg_addr, static_cast<uint8_t>((be >> 24) & 0xFF), static_cast<uint8_t>((be >> 16) & 0xFF),
      static_cast<uint8_t>((be >> 8) & 0xFF), static_cast<uint8_t>(be & 0xFF)});
}

bool parse_yiyou_read_u32_reply(
  const std::string & raw,
  uint8_t expect_addr,
  int32_t & value_out,
  std::string & reason)
{
  SlcanStdFrame frame{};
  if (!parse_slcan_t_frame(raw, frame, reason)) {
    return false;
  }
  if (frame.payload.size() < 6) {
    reason = "short_reply";
    return false;
  }
  if (frame.payload[0] != 0x04) {
    reason = "unexpected_cmd";
    return false;
  }
  if (frame.payload[1] != expect_addr) {
    reason = "unexpected_addr";
    return false;
  }
  const uint32_t raw_u32 = (static_cast<uint32_t>(frame.payload[2]) << 24) |
    (static_cast<uint32_t>(frame.payload[3]) << 16) |
    (static_cast<uint32_t>(frame.payload[4]) << 8) |
    static_cast<uint32_t>(frame.payload[5]);
  value_out = static_cast<int32_t>(raw_u32);
  reason = "parsed_ok";
  return true;
}

bool parse_hightorque_full_status(
  const std::string & raw,
  double & pos_out,
  double & vel_out,
  std::string & reason,
  uint32_t & can_id_out)
{
  std::vector<std::pair<uint32_t, std::vector<uint8_t>>> frames;
  {
    size_t cursor = 0;
    while (cursor < raw.size()) {
      const size_t d_pos = raw.find_first_of("DdBb", cursor);
      if (d_pos == std::string::npos) {
        break;
      }
      const bool is_extended = (raw[d_pos] == 'D' || raw[d_pos] == 'B');
      const bool is_canfd = (raw[d_pos] == 'D' || raw[d_pos] == 'd');
      const size_t can_id_hex_len = is_extended ? 8 : 3;
      if (!is_canfd || d_pos + 1 + can_id_hex_len + 1 > raw.size()) {
        cursor = d_pos + 1;
        continue;
      }
      try {
        const uint32_t parsed_can_id = static_cast<uint32_t>(
          std::stoul(raw.substr(d_pos + 1, can_id_hex_len), nullptr, 16));
        const unsigned int dlc_code = static_cast<unsigned int>(
          std::stoul(raw.substr(d_pos + 1 + can_id_hex_len, 1), nullptr, 16) & 0x0Fu);
        const size_t payload_len = [] (unsigned int dlc) -> size_t {
            switch (dlc & 0x0Fu) {
              case 0x0: return 0;
              case 0x1: return 1;
              case 0x2: return 2;
              case 0x3: return 3;
              case 0x4: return 4;
              case 0x5: return 5;
              case 0x6: return 6;
              case 0x7: return 7;
              case 0x8: return 8;
              case 0x9: return 12;
              case 0xA: return 16;
              case 0xB: return 20;
              case 0xC: return 24;
              case 0xD: return 32;
              case 0xE: return 48;
              case 0xF: return 64;
              default: return 0;
            }
          }(dlc_code);
        const size_t payload_hex_pos = d_pos + 1 + can_id_hex_len + 1;
        const size_t need_len = payload_hex_pos + payload_len * 2;
        if (need_len > raw.size()) {
          cursor = d_pos + 1;
          continue;
        }
        std::vector<uint8_t> payload;
        payload.reserve(payload_len);
        for (size_t i = 0; i < payload_len; ++i) {
          payload.push_back(static_cast<uint8_t>(
            std::stoul(raw.substr(payload_hex_pos + i * 2, 2), nullptr, 16) & 0xFFu));
        }
        frames.emplace_back(parsed_can_id, std::move(payload));
        cursor = need_len;
      } catch (...) {
        cursor = d_pos + 1;
      }
    }
  }
  if (frames.empty()) {
    reason = "short_reply";
    return false;
  }

  const auto le_i16 = [&](const std::vector<uint8_t> & payload, size_t off) -> int16_t {
      return static_cast<int16_t>(static_cast<uint16_t>(payload[off]) | (static_cast<uint16_t>(payload[off + 1]) << 8));
    };
  const auto le_i32 = [&](const std::vector<uint8_t> & payload, size_t off) -> int32_t {
      return static_cast<int32_t>(
        static_cast<uint32_t>(payload[off]) |
        (static_cast<uint32_t>(payload[off + 1]) << 8) |
        (static_cast<uint32_t>(payload[off + 2]) << 16) |
        (static_cast<uint32_t>(payload[off + 3]) << 24));
    };
  const auto le_f32 = [&](const std::vector<uint8_t> & payload, size_t off) -> float {
      uint32_t u =
        static_cast<uint32_t>(payload[off]) |
        (static_cast<uint32_t>(payload[off + 1]) << 8) |
        (static_cast<uint32_t>(payload[off + 2]) << 16) |
        (static_cast<uint32_t>(payload[off + 3]) << 24);
      float v = 0.0f;
      std::memcpy(&v, &u, sizeof(v));
      return v;
    };

  // TINT16 variant:
  // [0]=0x24 [1]=0x04 [2]=0x00 [3]=mode
  // [5:7]=pos_i16_le /10000, [7:9]=vel_i16_le /4000
  // [11]=0x21 [12]=0x0F [13]=fault
  for (const auto & frame : frames) {
    const auto & payload = frame.second;
    can_id_out = frame.first;
    if (
      payload.size() >= 14 &&
      payload[0] == 0x24 && payload[1] == 0x04 && payload[2] == 0x00 &&
      payload[11] == 0x21 && payload[12] == 0x0F)
    {
      const int16_t pos_i16 = le_i16(payload, 5);
      const int16_t vel_i16 = le_i16(payload, 7);
      pos_out = static_cast<double>(pos_i16) / 10000.0;
      vel_out = static_cast<double>(vel_i16) / 4000.0;
      reason = "parsed_ok_tint16";
      return true;
    }
    if (
      payload.size() >= 22 &&
      payload[0] == 0x28 && payload[1] == 0x04 && payload[2] == 0x00 &&
      payload[19] == 0x21 && payload[20] == 0x0F)
    {
      const int32_t pos_i32 = le_i32(payload, 7);
      const int32_t vel_i32 = le_i32(payload, 11);
      pos_out = static_cast<double>(pos_i32) / 100000.0;
      vel_out = static_cast<double>(vel_i32) / 100000.0;
      reason = "parsed_ok_tint32";
      return true;
    }
    if (
      payload.size() >= 22 &&
      payload[0] == 0x2C && payload[1] == 0x04 && payload[2] == 0x00 &&
      payload[19] == 0x21 && payload[20] == 0x0F)
    {
      const float pos_f32 = le_f32(payload, 7);
      const float vel_f32 = le_f32(payload, 11);
      if (std::isfinite(pos_f32) && std::isfinite(vel_f32)) {
        pos_out = static_cast<double>(pos_f32);
        vel_out = static_cast<double>(vel_f32);
        reason = "parsed_ok_tfloat";
        return true;
      }
    }
  }

  reason = "unknown_hightorque_payload_format_or_ack_only";
  return false;
}

bool is_state_sample_good(const JointState & state)
{
  return state.available && state.online && !state.stale &&
         std::isfinite(state.position) && std::isfinite(state.velocity);
}

double hightorque_model_k(const std::string & model)
{
  static const std::unordered_map<std::string, double> kModel = {
    {"M4438_30", 0.525600},
    {"M4438_32", 0.558400},
    {"M4538_19", 0.445000},
    {"M5043_20", 0.966000},
    {"M5046_20", 0.528000},
    {"M5047_09", 0.533000},
    {"M5047_36", 0.803000},
    {"M6056_36", 0.677000},
    {"M7256_35", 0.677000},
    {"M60SG_35", 0.794200},
    {"M60BM_35", 0.794200},
    {"MGENERAL", 0.500000},
    {"MNONE", 1.000000},
  };
  const auto it = kModel.find(model);
  return it == kModel.end() ? 0.525600 : it->second;
}

double hightorque_tqe_adjust(double tqe_nm, const std::string & model)
{
  return tqe_nm / hightorque_model_k(model);
}

double hightorque_pid_adjust(double pid, const std::string & model)
{
  return pid / hightorque_model_k(model);
}

std::string build_hightorque_stop_int32(const JointRoute & route)
{
  const uint32_t can_id = 0x8000u | static_cast<uint32_t>(route.node_id & 0xFF);
  return build_slcan_D_frame(can_id, {0x01, 0x00, 0x00, 0x18, 0x04, 0x00, 0x11, 0x0F});
}

std::string build_hightorque_brake_int32(const JointRoute & route)
{
  const uint32_t can_id = 0x8000u | static_cast<uint32_t>(route.node_id & 0xFF);
  return build_slcan_D_frame(can_id, {0x01, 0x00, 0x0F, 0x18, 0x04, 0x00, 0x11, 0x0F});
}

std::string build_hightorque_mit2_int32(
  const JointRoute & route,
  double pos_turns,
  double vel_rps,
  double tqe_nm,
  double kp,
  double kd,
  const std::string & model)
{
  const int32_t pos_i = static_cast<int32_t>(std::llround(pos_turns * 100000.0));
  const int32_t vel_i = static_cast<int32_t>(std::llround(vel_rps * 100000.0));
  const int32_t tqe_i = static_cast<int32_t>(std::llround(hightorque_tqe_adjust(tqe_nm, model) * 1000.0));
  const int32_t kp_i = static_cast<int32_t>(std::llround(hightorque_pid_adjust(kp, model) * 1000.0));
  const int32_t kd_i = static_cast<int32_t>(std::llround(hightorque_pid_adjust(kd, model) * 1000.0));

  std::vector<uint8_t> payload = {
    0x01, 0x00, 0x15,
    0x0B, 0x20,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0x0A, 0x2B,
    0, 0, 0, 0,
    0, 0, 0, 0,
    0x18, 0x04, 0x00, 0x11, 0x0F};

  auto pack_i32 = [&](size_t off, int32_t v) {
    payload[off + 0] = static_cast<uint8_t>(v & 0xFF);
    payload[off + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    payload[off + 2] = static_cast<uint8_t>((v >> 16) & 0xFF);
    payload[off + 3] = static_cast<uint8_t>((v >> 24) & 0xFF);
  };
  pack_i32(5, pos_i);
  pack_i32(9, vel_i);
  pack_i32(13, tqe_i);
  pack_i32(19, kp_i);
  pack_i32(23, kd_i);

  const uint32_t can_id = 0x8000u | static_cast<uint32_t>(route.node_id & 0xFF);
  return build_slcan_D_frame(can_id, payload);
}

// -----------------------------------------------------------------------------
// LEGACY PROTOCOL LOGIC BOUNDARY (temporary)
//
// This block intentionally centralizes the remaining protocol frame glue in C++.
// These are legacy protocol helpers and will be replaced by adapter-backed
// device action boundaries in a follow-up migration.
// -----------------------------------------------------------------------------
struct LegacyProtocolDeviceActions final
{
  // Temporary adapter-backed runtime path.
  // This path shells out to Python adapters from C++.
  // If adapter runtime call fails, caller should fallback to legacy protocol path.
  static bool read_hightorque_state_via_adapter(
    const std::string & serial_device,
    int node_id,
    double & position_out,
    double & velocity_out,
    std::string & reason_out)
  {
    std::ostringstream py;
    py
      << "import json;"
      << "from robot_arm_hardware_hightorque_canfd.hightorque_fdcan_adapter import HightorqueFdcanAdapter;"
      << "a=HightorqueFdcanAdapter(serial_device='" << serial_device << "');"
      << "a.open();a.initialize_bridge();"
      << "s=a.read_joint_state(" << node_id << ");"
      << "a.close();"
      << "print(json.dumps(s))";
    std::string output;
    if (!run_python_snippet(py.str(), output)) {
      reason_out = "python_adapter_invocation_failed";
      return false;
    }
    double pos = 0.0;
    double vel = 0.0;
    bool online = false;
    if (!extract_json_number(output, "position", pos) || !extract_json_number(output, "velocity", vel)) {
      reason_out = "python_adapter_parse_failed_position_velocity";
      return false;
    }
    if (!extract_json_bool(output, "online", online) || !online) {
      reason_out = "python_adapter_offline_or_missing_online";
      return false;
    }
    position_out = pos;
    velocity_out = vel;
    reason_out = "python_adapter_ok";
    return true;
  }

  static bool read_yiyou_state_via_adapter(
    const std::string & serial_device,
    int node_id,
    double & position_out,
    double & velocity_out,
    int & enable_out,
    std::string & reason_out)
  {
    std::ostringstream py;
    py
      << "import json;"
      << "from robot_arm_hardware_yiyou_can20a.yiyou_can_adapter import YiyouCanAdapter;"
      << "a=YiyouCanAdapter(serial_device='" << serial_device << "',node_id=" << node_id << ");"
      << "a.open();a.initialize_bridge();"
      << "p=a.read_position();v=a.read_velocity();e=a.read_enable_state();"
      << "a.close();"
      << "print(json.dumps({'position':p.get('position'),'velocity':v.get('velocity'),'enabled':e.get('enabled')}))";
    std::string output;
    if (!run_python_snippet(py.str(), output)) {
      reason_out = "python_adapter_invocation_failed";
      return false;
    }
    double pos = 0.0;
    double vel = 0.0;
    bool enabled = false;
    if (!extract_json_number(output, "position", pos) || !extract_json_number(output, "velocity", vel)) {
      reason_out = "python_adapter_parse_failed_position_velocity";
      return false;
    }
    if (!extract_json_bool(output, "enabled", enabled)) {
      reason_out = "python_adapter_parse_failed_enabled";
      return false;
    }
    position_out = pos;
    velocity_out = vel;
    enable_out = enabled ? 1 : 0;
    reason_out = "python_adapter_ok";
    return true;
  }

  static bool run_python_snippet(const std::string & snippet, std::string & output)
  {
    std::string cmd = "python3 -c \"";
    for (const char c : snippet) {
      if (c == '\"') {
        cmd += "\\\"";
      } else if (c == '\n') {
        cmd += ' ';
      } else {
        cmd.push_back(c);
      }
    }
    cmd += "\" 2>/dev/null";
    FILE * fp = popen(cmd.c_str(), "r");
    if (fp == nullptr) {
      return false;
    }
    std::array<char, 256> buf{};
    output.clear();
    while (fgets(buf.data(), static_cast<int>(buf.size()), fp) != nullptr) {
      output.append(buf.data());
    }
    const int rc = pclose(fp);
    return rc == 0 && !output.empty();
  }

  static bool extract_json_number(const std::string & text, const std::string & key, double & out)
  {
    const std::regex pattern("\"" + key + R"(\"\s*:\s*(-?\d+(?:\.\d+)?))");
    std::smatch match;
    if (!std::regex_search(text, match, pattern)) {
      return false;
    }
    try {
      out = std::stod(match[1].str());
      return true;
    } catch (...) {
      return false;
    }
  }

  static bool extract_json_bool(const std::string & text, const std::string & key, bool & out)
  {
    const std::regex pattern("\"" + key + R"(\"\s*:\s*(true|false))", std::regex::icase);
    std::smatch match;
    if (!std::regex_search(text, match, pattern)) {
      return false;
    }
    const std::string v = match[1].str();
    out = (v == "true" || v == "TRUE" || v == "True");
    return true;
  }

  // HighTorque actions (legacy protocol in C++).
  static std::string hightorque_query(const JointRoute & route)
  {
    return build_hightorque_read_query(route);
  }

  static std::string hightorque_query_full_status(const JointRoute & route)
  {
    return build_hightorque_full_status_query(route);
  }

  static bool parse_hightorque_query_reply(
    const std::string & raw,
    double & pos_out,
    double & vel_out,
    std::string & reason,
    uint32_t & can_id_out)
  {
    return parse_hightorque_full_status(raw, pos_out, vel_out, reason, can_id_out);
  }

  // NOTE: currently not wired into write path; kept to define action boundary.
  static std::string hightorque_stop(const JointRoute & route)
  {
    return build_hightorque_stop_int32(route);
  }

  // NOTE: currently not wired into write path; kept to define action boundary.
  static std::string hightorque_brake(const JointRoute & route)
  {
    return build_hightorque_brake_int32(route);
  }

  // NOTE: currently not wired into write path; kept to define action boundary.
  static std::string hightorque_move(const JointRoute & route, double velocity_rps)
  {
    int32_t vel_i32 = static_cast<int32_t>(std::llround(velocity_rps * 100000.0));
    std::vector<uint8_t> payload = {
      0x01, 0x00, 0x0A, 0x08, 0x02, 0x20, 0x00, 0x00,
      0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x50, 0x50};
    payload[10] = static_cast<uint8_t>(vel_i32 & 0xFF);
    payload[11] = static_cast<uint8_t>((vel_i32 >> 8) & 0xFF);
    payload[12] = static_cast<uint8_t>((vel_i32 >> 16) & 0xFF);
    payload[13] = static_cast<uint8_t>((vel_i32 >> 24) & 0xFF);
    const uint32_t can_id = 0x8000u | static_cast<uint32_t>(route.node_id & 0xFF);
    return build_slcan_D_frame(can_id, payload);
  }

  // Experimental/transition family: mode2 position-target + velocity feed.
  // target_turns is encoded in int32 with 1e5 turns scale.
  static std::string hightorque_move_position_target(
    const JointRoute & route,
    double target_turns,
    double velocity_rps,
    double tqe_nm,
    double kp,
    double kd,
    const std::string & model)
  {
    return build_hightorque_mit2_int32(route, target_turns, velocity_rps, tqe_nm, kp, kd, model);
  }

  // Yiyou actions (legacy protocol in C++).
  static std::string yiyou_read_position(const JointRoute & route)
  {
    return build_yiyou_read_query(route, 0x07);
  }

  static std::string yiyou_read_velocity(const JointRoute & route)
  {
    return build_yiyou_read_query(route, 0x06);
  }

  static std::string yiyou_read_enable(const JointRoute & route)
  {
    return build_yiyou_read_query(route, 0x10);
  }

  static std::string yiyou_read_alarm(const JointRoute & route)
  {
    return build_yiyou_read_query(route, 0x15);
  }

  static std::string yiyou_read_register(const JointRoute & route, uint8_t reg_addr)
  {
    switch (reg_addr) {
      case 0x07:
        return yiyou_read_position(route);
      case 0x06:
        return yiyou_read_velocity(route);
      case 0x10:
        return yiyou_read_enable(route);
      case 0x15:
        return yiyou_read_alarm(route);
      default:
        return build_yiyou_read_query(route, reg_addr);
    }
  }

  static bool parse_yiyou_read_reply(
    const std::string & raw,
    uint8_t reg_addr,
    int32_t & value_out,
    std::string & reason)
  {
    return parse_yiyou_read_u32_reply(raw, reg_addr, value_out, reason);
  }

  // NOTE: currently not wired into write path; kept to define action boundary.
  static std::string yiyou_enable(const JointRoute & route)
  {
    return build_yiyou_write_u32_query(route, 0x10, 1);
  }

  // NOTE: currently not wired into write path; kept to define action boundary.
  static std::string yiyou_disable(const JointRoute & route)
  {
    return build_yiyou_write_u32_query(route, 0x10, 0);
  }

  // NOTE: currently not wired into write path; kept to define action boundary.
  static std::string yiyou_stop(const JointRoute & route)
  {
    return build_yiyou_write_u32_query(route, 0x11, 1);
  }

  // NOTE: currently not wired into write path; kept to define action boundary.
  static std::string yiyou_write_target_position(const JointRoute & route, int32_t value)
  {
    return build_yiyou_write_u32_query(route, 0x0A, value);
  }

  // NOTE: currently not wired into write path; kept to define action boundary.
  static std::string yiyou_write_target_speed(const JointRoute & route, int32_t value)
  {
    return build_yiyou_write_u32_query(route, 0x09, value);
  }
};

bool configure_serial_fd(int fd)
{
  termios tio{};
  if (tcgetattr(fd, &tio) != 0) {
    return false;
  }
  auto log_termios = [&](const char * phase, const termios & t) {
      const unsigned int cs = static_cast<unsigned int>(t.c_cflag & CSIZE);
      RCLCPP_INFO(
        rclcpp::get_logger("RealMixedRobotBackend"),
        "[Hightorque] termios %s: ICANON=%d ECHO=%d ECHONL=%d ISIG=%d IEXTEN=%d ICRNL=%d INLCR=%d IGNCR=%d IXON=%d IXOFF=%d IXANY=%d OPOST=%d PARENB=%d CSTOPB=%d CSIZE=0x%X CS8=%d CRTSCTS=%d VMIN=%d VTIME=%d",
        phase,
        (t.c_lflag & ICANON) ? 1 : 0,
        (t.c_lflag & ECHO) ? 1 : 0,
        (t.c_lflag & ECHONL) ? 1 : 0,
        (t.c_lflag & ISIG) ? 1 : 0,
        (t.c_lflag & IEXTEN) ? 1 : 0,
        (t.c_iflag & ICRNL) ? 1 : 0,
        (t.c_iflag & INLCR) ? 1 : 0,
        (t.c_iflag & IGNCR) ? 1 : 0,
        (t.c_iflag & IXON) ? 1 : 0,
        (t.c_iflag & IXOFF) ? 1 : 0,
        (t.c_iflag & IXANY) ? 1 : 0,
        (t.c_oflag & OPOST) ? 1 : 0,
        (t.c_cflag & PARENB) ? 1 : 0,
        (t.c_cflag & CSTOPB) ? 1 : 0,
        cs,
        ((t.c_cflag & CSIZE) == CS8) ? 1 : 0,
        (t.c_cflag & CRTSCTS) ? 1 : 0,
        static_cast<int>(t.c_cc[VMIN]),
        static_cast<int>(t.c_cc[VTIME]));
    };
  log_termios("before", tio);

  // Align with pyserial raw 8N1 non-blocking style.
  cfmakeraw(&tio);
  tio.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR);
  tio.c_lflag &= ~(ICANON | ECHO | ECHONL | ISIG | IEXTEN);
  tio.c_oflag &= ~OPOST;
  tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
  tio.c_cflag |= CS8;
  cfsetispeed(&tio, B115200);
  cfsetospeed(&tio, B115200);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  if (tcsetattr(fd, TCSANOW, &tio) != 0) {
    return false;
  }
  termios verify{};
  if (tcgetattr(fd, &verify) == 0) {
    log_termios("after", verify);
  }
  tcflush(fd, TCIOFLUSH);
  return true;
}

void reset_input_buffer_only(const char * channel_name, int fd)
{
  const int in_rc = tcflush(fd, TCIFLUSH);
  RCLCPP_INFO(
    rclcpp::get_logger("RealMixedRobotBackend"),
    "[%s] reset_input_buffer=%s reset_output_buffer=skipped",
    channel_name,
    in_rc == 0 ? "called" : "failed");
}

void log_modem_lines(const char * channel_name, const char * phase, int fd)
{
  int status = 0;
  if (ioctl(fd, TIOCMGET, &status) != 0) {
    RCLCPP_WARN(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[%s] modem lines %s read failed: errno=%d (%s)",
      channel_name,
      phase,
      errno,
      std::strerror(errno));
    return;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("RealMixedRobotBackend"),
    "[%s] modem lines %s: RTS=%d DTR=%d CTS=%d DSR=%d CD=%d RI=%d",
    channel_name,
    phase,
    (status & TIOCM_RTS) ? 1 : 0,
    (status & TIOCM_DTR) ? 1 : 0,
    (status & TIOCM_CTS) ? 1 : 0,
    (status & TIOCM_DSR) ? 1 : 0,
    (status & TIOCM_CAR) ? 1 : 0,
    (status & TIOCM_RI) ? 1 : 0);
}

void set_modem_lines_active(const char * channel_name, int fd)
{
  int status = 0;
  if (ioctl(fd, TIOCMGET, &status) != 0) {
    RCLCPP_WARN(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[%s] modem lines pre-set read failed: errno=%d (%s)",
      channel_name,
      errno,
      std::strerror(errno));
    return;
  }
  status |= (TIOCM_RTS | TIOCM_DTR);
  if (ioctl(fd, TIOCMSET, &status) != 0) {
    RCLCPP_WARN(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[%s] set RTS/DTR active failed: errno=%d (%s)",
      channel_name,
      errno,
      std::strerror(errno));
    return;
  }
  RCLCPP_INFO(rclcpp::get_logger("RealMixedRobotBackend"), "[%s] set RTS/DTR active ok", channel_name);
}

bool ensure_open(
  const char * channel_name,
  const std::string & device,
  int & fd,
  bool & initialized,
  const std::vector<std::string> & init_cmds)
{
  if (fd < 0) {
    fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      RCLCPP_ERROR(
        rclcpp::get_logger("RealMixedRobotBackend"),
        "[%s] open failed: device=%s",
        channel_name,
        device.c_str());
      return false;
    }
    if (!configure_serial_fd(fd)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("RealMixedRobotBackend"),
        "[%s] serial config failed: device=%s baud=115200",
        channel_name,
        device.c_str());
      ::close(fd);
      fd = -1;
      return false;
    }
    RCLCPP_INFO(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[%s] opened device=%s baud=115200",
      channel_name,
      device.c_str());
    set_modem_lines_active(channel_name, fd);
    log_modem_lines(channel_name, "after_open", fd);
    RCLCPP_INFO(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[%s] serial params: baudrate=115200 timeout=%dms write_timeout=%dms inter_byte_timeout=%dms xonxoff=off rtscts=off dsrdtr=off",
      channel_name,
      kSerialTimeoutMs,
      kWriteTimeoutMs,
      kReadIdleGapMs);
  }

  if (!initialized) {
    for (const auto & cmd : init_cmds) {
      RCLCPP_INFO(
        rclcpp::get_logger("RealMixedRobotBackend"),
        "[%s] init command ascii_escaped='%s' hex=%s",
        channel_name,
        ascii_escaped(cmd).c_str(),
        hex_digest(cmd).c_str());
      const ssize_t n = ::write(fd, cmd.data(), cmd.size());
      if (n < 0 || static_cast<size_t>(n) != cmd.size()) {
        RCLCPP_ERROR(
          rclcpp::get_logger("RealMixedRobotBackend"),
          "[%s] init command send failed: cmd='%s'",
          channel_name,
          cmd.c_str());
        return false;
      }
      usleep(15000);
      if (cmd == "O\r") {
        constexpr useconds_t kPostOpenSleepUs = 100000;
        RCLCPP_INFO(
          rclcpp::get_logger("RealMixedRobotBackend"),
          "[%s] post-open sleep after O: %u us",
          channel_name,
          static_cast<unsigned int>(kPostOpenSleepUs));
        usleep(kPostOpenSleepUs);
      }
    }
    initialized = true;
    RCLCPP_INFO(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[%s] initialization sequence sent (%zu commands)",
      channel_name,
      init_cmds.size());
  }

  return true;
}

bool send_query(
  const char * channel_name,
  int fd,
  const std::string & query,
  uint64_t & send_failures,
  uint64_t attempts,
  bool drain_after_write = true)
{
  if (attempts <= 3 || attempts % 200 == 0) {
    RCLCPP_INFO(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[%s] read_joint query ascii_escaped='%s' tx_hex=%s tx_len=%zu",
      channel_name,
      ascii_escaped(query).c_str(),
      hex_digest(query).c_str(),
      query.size());
  }
  fd_set write_set;
  FD_ZERO(&write_set);
  FD_SET(fd, &write_set);
  timeval wtv{};
  wtv.tv_sec = kWriteTimeoutMs / 1000;
  wtv.tv_usec = (kWriteTimeoutMs % 1000) * 1000;
  const int wrc = select(fd + 1, nullptr, &write_set, nullptr, &wtv);
  if (wrc <= 0) {
    ++send_failures;
    RCLCPP_WARN(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[%s] read_joint write timeout (write_timeout=%dms)",
      channel_name,
      kWriteTimeoutMs);
    return false;
  }
  const ssize_t n = ::write(fd, query.data(), query.size());
  if (n < 0 || static_cast<size_t>(n) != query.size()) {
    ++send_failures;
    if (send_failures == 1 || send_failures % 50 == 0) {
      RCLCPP_WARN(
        rclcpp::get_logger("RealMixedRobotBackend"),
        "[%s] read_joint send failed (attempt=%lu, fail_count=%lu)",
        channel_name,
        static_cast<unsigned long>(attempts),
        static_cast<unsigned long>(send_failures));
    }
    return false;
  }
  if (drain_after_write) {
    // Ensure query bytes are physically pushed out before entering read window.
    RCLCPP_INFO(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[%s] write flush/drain: tcdrain=called",
      channel_name);
    if (tcdrain(fd) != 0) {
      RCLCPP_WARN(
        rclcpp::get_logger("RealMixedRobotBackend"),
        "[%s] read_joint tcdrain failed",
        channel_name);
    }
    // Small post-write settling delay to improve first-reply capture.
    usleep(2000);
  } else {
    // Async servo writer path: avoid blocking drain in the control-adjacent loop.
    usleep(500);
  }
  if (attempts <= 3 || attempts % 200 == 0) {
    RCLCPP_INFO(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[%s] read_joint write ok (wrote=%ld expected=%zu)",
      channel_name,
      static_cast<long>(n),
      query.size());
  }
  return true;
}

bool read_raw_frame(
  int fd,
  std::string & raw,
  uint64_t & recv_timeouts,
  const char * channel_name,
  uint64_t attempts)
{
  raw.clear();
  const auto now_us = []() -> uint64_t {
      timeval tv{};
      gettimeofday(&tv, nullptr);
      return static_cast<uint64_t>(tv.tv_sec) * 1000000ULL + static_cast<uint64_t>(tv.tv_usec);
    };
  const uint64_t start_us = now_us();
  uint64_t last_data_us = 0;
  bool got_any = false;
  uint64_t poll_loops = 0;
  int last_fionread = -1;
  const char * window_expired_reason = "unknown";
  if (attempts <= 3 || attempts % 200 == 0) {
    RCLCPP_INFO(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[%s] read window: total=%dms idle_gap=%dms poll_slice=20ms",
      channel_name,
      kReadWindowMs,
      kReadIdleGapMs);
  }

  while (true) {
    ++poll_loops;
    const uint64_t t_us = now_us();
    const int elapsed_ms = static_cast<int>((t_us - start_us) / 1000ULL);
    if (!got_any && elapsed_ms >= kReadWindowMs) {
      window_expired_reason = "window_no_data";
      break;
    }
    if (got_any && static_cast<int>((t_us - last_data_us) / 1000ULL) >= kReadIdleGapMs) {
      window_expired_reason = "idle_gap";
      break;
    }
    if (elapsed_ms >= kReadWindowMs) {
      window_expired_reason = "window_limit";
      break;
    }

    int waiting = 0;
    last_fionread = waiting;
    if (ioctl(fd, FIONREAD, &waiting) != 0 || waiting <= 0) {
      last_fionread = waiting;
      usleep(5000);  // 5ms tick, aligned to verified Python in_waiting polling rhythm.
      continue;
    }
    last_fionread = waiting;

    const size_t chunk = static_cast<size_t>(std::min(waiting, static_cast<int>(kReadBufSize)));
    char buffer[kReadBufSize];
    const ssize_t n = ::read(fd, buffer, chunk);
    if (n <= 0) {
      continue;
    }
    const std::string chunk_str(buffer, static_cast<size_t>(n));
    RCLCPP_INFO(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[%s] chunk_len=%ld chunk_hex=%s chunk_ascii_escaped='%s'",
      channel_name,
      static_cast<long>(n),
      hex_digest(chunk_str).c_str(),
      ascii_escaped(chunk_str).c_str());
    raw.append(buffer, static_cast<size_t>(n));
    got_any = true;
    last_data_us = now_us();
  }

  if (raw.empty()) {
    RCLCPP_INFO(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[%s] poll summary: poll_loops=%lu last_fionread=%d window_expired_reason=%s",
      channel_name,
      static_cast<unsigned long>(poll_loops),
      last_fionread,
      window_expired_reason);
    ++recv_timeouts;
    if (recv_timeouts == 1 || recv_timeouts % 100 == 0) {
      RCLCPP_WARN(
        rclcpp::get_logger("RealMixedRobotBackend"),
        "[%s] read_joint timeout/no-response (attempt=%lu, timeout_count=%lu)",
        channel_name,
        static_cast<unsigned long>(attempts),
        static_cast<unsigned long>(recv_timeouts));
    }
    return true;
  }
  return true;
}

void maybe_log_success(
  const char * channel,
  uint64_t attempts,
  const JointRoute & route,
  const std::string & raw,
  const JointState & state)
{
  if (attempts <= 3 || attempts % 200 == 0) {
    RCLCPP_INFO(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[%s] read_joint ok attempt=%lu joint=%s raw_len=%zu raw_hex=%s pos=%.6f vel=%.6f",
      channel,
      static_cast<unsigned long>(attempts),
      route.joint_name.c_str(),
      raw.size(),
      hex_digest(raw).c_str(),
      state.position,
      state.velocity);
  }
}
}  // namespace

bool HightorqueReadChannel::discover(const JointRouteTable & routes, std::unordered_map<std::string, bool> & seen)
{
  if (!std::filesystem::exists(bridge_device)) {
    RCLCPP_WARN(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[Hightorque] bridge device missing: %s",
      bridge_device.c_str());
    return false;
  }
  if (!ensure_open("Hightorque", bridge_device, fd, initialized, {"C\r", "S8\r", "Y5\r", "M0\r", "A1\r", "O\r"})) {
    return false;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("RealMixedRobotBackend"),
    "[Hightorque] bridge device ready: %s",
    bridge_device.c_str());
  for (const auto & route : routes) {
    if (route.driver == "hightorque_canfd") {
      seen[route.joint_name] = true;
    }
  }
  return true;
}

bool HightorqueReadChannel::sync_current_positions(
  const JointRouteTable & routes,
  std::unordered_map<std::string, JointState> & out_states)
{
  latest_states.clear();
  for (const auto & route : routes) {
    if (route.driver != "hightorque_canfd") {
      continue;
    }
    JointState state{};
    if (!read_joint(route, state) || !state.available) {
      return false;
    }
    latest_states[route.joint_name] = state;
    out_states[route.joint_name] = state;
  }
  return true;
}

bool HightorqueReadChannel::read_joint(const JointRoute & route, JointState & out_state)
{
  std::unique_lock<std::mutex> io_lock(io_mutex, std::try_to_lock);
  if (!io_lock.owns_lock()) {
    out_state.available = false;
    out_state.online = false;
    out_state.stale = true;
    out_state.last_error = "hightorque_io_busy_skip_poll";
    return true;
  }
  if (debug_single_joint && route.joint_name != debug_joint_name) {
    RCLCPP_INFO(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[Hightorque] debug single-joint mode: skip joint=%s (only %s queried)",
      route.joint_name.c_str(),
      debug_joint_name.c_str());
    out_state.available = false;
    out_state.online = false;
    return true;
  }

  static std::atomic<bool> query_inflight{false};
  bool expected = false;
  if (!query_inflight.compare_exchange_strong(expected, true)) {
    RCLCPP_WARN(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[Hightorque] outstanding query exists, skip this cycle");
    return true;
  }
  auto clear_inflight = +[](std::atomic<bool> & inflight) {inflight.store(false);};

  ++read_attempts;
  out_state.available = false;
  out_state.online = false;

  const bool strict_replay = []() {
      const char * v = std::getenv("ROBOT_ARM_HT_STRICT_REPLAY");
      return v != nullptr && std::string(v) == "1";
    }();
  const bool strict_exclusive = strict_replay;
  const std::vector<std::string> init_cmds = {"C\r", "S8\r", "Y5\r", "M0\r", "A1\r", "O\r"};
  if (!ensure_open("Hightorque", bridge_device, fd, initialized, init_cmds)) {
    clear_inflight(query_inflight);
    return false;
  }
  if (strict_replay) {
    std::string strict_target_joint = debug_joint_name;
    int strict_target_node = -1;
    const char * strict_joint_env = std::getenv("ROBOT_ARM_HT_STRICT_JOINT");
    if (strict_joint_env != nullptr && std::strlen(strict_joint_env) > 0) {
      strict_target_joint = std::string(strict_joint_env);
    }
    const char * strict_node_env = std::getenv("ROBOT_ARM_HT_STRICT_NODE");
    if (strict_node_env != nullptr && std::strlen(strict_node_env) > 0) {
      try {
        strict_target_node = std::stoi(strict_node_env);
      } catch (...) {
        strict_target_node = -1;
      }
    }
    const bool by_node = strict_target_node >= 0;
    const bool route_is_target = by_node ? (route.node_id == strict_target_node) : (route.joint_name == strict_target_joint);
    RCLCPP_INFO(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[Hightorque][STRICT_REPLAY] target joint=%s target node=%d route joint=%s route node=%d mode=%s",
      strict_target_joint.c_str(),
      strict_target_node,
      route.joint_name.c_str(),
      route.node_id,
      by_node ? "node" : "joint");
    if (!route_is_target) {
      RCLCPP_INFO(
        rclcpp::get_logger("RealMixedRobotBackend"),
        "[Hightorque][STRICT_REPLAY] exclusive=true normal_polling_disabled=true skip joint=%s node=%d keep only=%s=%s",
        route.joint_name.c_str(),
        route.node_id,
        by_node ? "node" : "joint",
        by_node ? std::to_string(strict_target_node).c_str() : strict_target_joint.c_str());
      clear_inflight(query_inflight);
      return true;
    }
    const char * raw_query_env = std::getenv("ROBOT_ARM_HT_RAW_QUERY");
    std::string raw_query = raw_query_env != nullptr ?
      std::string(raw_query_env) :
      std::string("D00008006A01000A0802200000000801027000005050");
    if (raw_query.find('\r') == std::string::npos) {
      raw_query.push_back('\r');
    }
    RCLCPP_INFO(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[Hightorque][STRICT_REPLAY] exclusive=%s normal_polling_disabled=%s target joint=%s target node=%d raw_query_ascii='%s' raw_query_hex=%s",
      strict_exclusive ? "true" : "false",
      strict_exclusive ? "true" : "false",
      strict_target_joint.c_str(),
      strict_target_node,
      ascii_escaped(raw_query).c_str(),
      hex_digest(raw_query).c_str());
    if (send_query("Hightorque", fd, raw_query, send_failures, read_attempts)) {
      std::string replay_raw;
      read_raw_frame(fd, replay_raw, recv_timeouts, "Hightorque", read_attempts);
      RCLCPP_INFO(
        rclcpp::get_logger("RealMixedRobotBackend"),
        "[Hightorque][STRICT_REPLAY] raw_len=%zu raw_ascii='%s' raw_hex=%s classify=%s",
        replay_raw.size(),
        ascii_escaped(replay_raw).c_str(),
        hex_digest(replay_raw).c_str(),
        replay_raw.empty() ? "no_data" : "non_empty");
    }
    strict_replay_done = true;
    clear_inflight(query_inflight);
    return true;
  }

  // Hot-path policy: default disable shell-out Python adapter in per-cycle read.
  // Enable only for diagnostics via ROBOT_ARM_ENABLE_ADAPTER_SHELLOUT=1.
  const bool enable_adapter_shellout = []() {
      const char * v = std::getenv("ROBOT_ARM_ENABLE_ADAPTER_SHELLOUT");
      return v != nullptr && std::string(v) == "1";
    }();
  if (enable_adapter_shellout) {
    double py_pos = 0.0;
    double py_vel = 0.0;
    std::string py_reason;
    if (LegacyProtocolDeviceActions::read_hightorque_state_via_adapter(
        bridge_device, route.node_id, py_pos, py_vel, py_reason))
    {
      out_state.position = py_pos;
      out_state.velocity = py_vel;
      out_state.available = true;
      out_state.online = true;
      ++valid_samples;
      maybe_log_success("Hightorque(adapter)", read_attempts, route, "python_adapter", out_state);
      clear_inflight(query_inflight);
      return true;
    }
    RCLCPP_WARN(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[Hightorque] adapter-backed read failed, fallback to legacy protocol path: reason=%s",
      py_reason.c_str());
  }

  set_modem_lines_active("Hightorque", fd);
  log_modem_lines("Hightorque", "before_query", fd);
  reset_input_buffer_only("Hightorque", fd);
  // Legacy protocol path: temporary C++ action boundary call.
  const std::string query = LegacyProtocolDeviceActions::hightorque_query(route);
  if (!send_query("Hightorque", fd, query, send_failures, read_attempts)) {
    clear_inflight(query_inflight);
    return true;
  }

  std::string raw;
  if (!read_raw_frame(fd, raw, recv_timeouts, "Hightorque", read_attempts)) {
    clear_inflight(query_inflight);
    return false;
  }
  if (raw.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("RealMixedRobotBackend"), "[Hightorque] classify=no_data");
    clear_inflight(query_inflight);
    return true;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("RealMixedRobotBackend"),
    "[Hightorque] read_joint raw frame attempt=%lu raw_len=%zu raw_hex=%s raw_ascii_escaped='%s'",
    static_cast<unsigned long>(read_attempts),
    raw.size(),
    hex_digest(raw).c_str(),
    ascii_escaped(raw).c_str());

  ++frames_seen;
  out_state.online = true;
  if (is_hightorque_bridge_ack(raw)) {
    static uint64_t ack_count = 0;
    ++ack_count;
    RCLCPP_INFO(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[Hightorque] short_ack/bridge_ack detected (ack_count=%lu, raw_len=%zu, raw_hex=%s, raw_ascii_escaped='%s', next_step=need_full_status_query)",
      static_cast<unsigned long>(ack_count),
      raw.size(),
      hex_digest(raw).c_str(),
      ascii_escaped(raw).c_str());
    // Official read path already uses read_motor_state_int32 query frame.
    // If bridge returns short ACK only, retry the same official query once.
    const std::string full_status_query = LegacyProtocolDeviceActions::hightorque_query(route);
    RCLCPP_INFO(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[Hightorque] full_status_query tx ascii_escaped='%s' tx_hex=%s tx_len=%zu",
      ascii_escaped(full_status_query).c_str(),
      hex_digest(full_status_query).c_str(),
      full_status_query.size());
    if (!send_query("Hightorque", fd, full_status_query, send_failures, read_attempts)) {
      clear_inflight(query_inflight);
      return true;
    }
    std::string full_raw;
    if (!read_raw_frame(fd, full_raw, recv_timeouts, "Hightorque", read_attempts)) {
      clear_inflight(query_inflight);
      return false;
    }
    if (full_raw.empty()) {
      RCLCPP_WARN(
        rclcpp::get_logger("RealMixedRobotBackend"),
        "[Hightorque] full_status_response missing after bridge_ack");
      clear_inflight(query_inflight);
      return true;
    }
    RCLCPP_INFO(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[Hightorque] full_status_response raw_len=%zu raw_hex=%s raw_ascii_escaped='%s'",
      full_raw.size(),
      hex_digest(full_raw).c_str(),
      ascii_escaped(full_raw).c_str());
    raw = full_raw;
  }
  double parsed_pos = 0.0;
  double parsed_vel = 0.0;
  uint32_t parsed_can_id = 0;
  std::string parse_reason;
  const bool parsed_ok = LegacyProtocolDeviceActions::parse_hightorque_query_reply(
    raw, parsed_pos, parsed_vel, parse_reason, parsed_can_id);
  if (!parsed_ok) {
    ++parse_failures;
    RCLCPP_WARN(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[Hightorque] classify=malformed_or_unparsed parse failed (attempt=%lu, parse_failures=%lu, reason=%s, raw_len=%zu, raw_hex=%s, raw_ascii_escaped='%s')",
      static_cast<unsigned long>(read_attempts),
      static_cast<unsigned long>(parse_failures),
      parse_reason.c_str(),
      raw.size(),
      hex_digest(raw).c_str(),
      ascii_escaped(raw).c_str());
    clear_inflight(query_inflight);
    return true;
  }

  const uint8_t parsed_node_id = hightorque_node_id_from_can_id(parsed_can_id);
  if (parsed_node_id == 0) {
    ++parse_failures;
    RCLCPP_WARN(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[Hightorque] classify=node_reject reason=node_id_zero route_node=%d raw_hex=%s",
      route.node_id,
      hex_digest(raw).c_str());
    clear_inflight(query_inflight);
    return true;
  }
  if (parsed_node_id != static_cast<uint8_t>(route.node_id & 0xFF)) {
    ++parse_failures;
    RCLCPP_WARN(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[Hightorque] classify=node_reject reason=node_mismatch parsed_node=%u route_node=%d raw_hex=%s",
      static_cast<unsigned int>(parsed_node_id),
      route.node_id,
      hex_digest(raw).c_str());
    clear_inflight(query_inflight);
    return true;
  }

  out_state.position = parsed_pos;
  out_state.velocity = parsed_vel;
  out_state.available = true;
  RCLCPP_INFO(
    rclcpp::get_logger("RealMixedRobotBackend"),
    "[Hightorque] parsed state classify=parsed_ok node_id=0x%X position=%.6f velocity=%.6f available=true online=true",
    static_cast<unsigned int>(parsed_node_id),
    out_state.position,
    out_state.velocity);
  ++valid_samples;
  maybe_log_success("Hightorque", read_attempts, route, raw, out_state);
  clear_inflight(query_inflight);
  return true;
}

bool YiyouReadChannel::discover(const JointRouteTable & routes, std::unordered_map<std::string, bool> & seen)
{
  if (!std::filesystem::exists(bridge_device)) {
    RCLCPP_WARN(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[Yiyou] bridge device missing: %s",
      bridge_device.c_str());
    return false;
  }
  if (!ensure_open("Yiyou", bridge_device, fd, initialized, {"C\r", "S8\r", "M0\r", "A1\r", "O\r"})) {
    return false;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("RealMixedRobotBackend"),
    "[Yiyou] bridge device ready: %s",
    bridge_device.c_str());
  for (const auto & route : routes) {
    if (route.driver == "yiyou_can20a") {
      seen[route.joint_name] = true;
    }
  }
  return true;
}

bool YiyouReadChannel::sync_current_positions(
  const JointRouteTable & routes,
  std::unordered_map<std::string, JointState> & out_states)
{
  latest_joint2_state.reset();
  for (const auto & route : routes) {
    if (route.driver != "yiyou_can20a") {
      continue;
    }
    JointState state{};
    if (!read_joint(route, state) || !state.available) {
      return false;
    }
    latest_joint2_state = state;
    out_states[route.joint_name] = state;
  }
  return true;
}

bool YiyouReadChannel::read_joint(const JointRoute & route, JointState & out_state)
{
  std::unique_lock<std::mutex> io_lock(io_mutex, std::try_to_lock);
  if (!io_lock.owns_lock()) {
    out_state.available = false;
    out_state.online = false;
    out_state.stale = true;
    out_state.last_error = "yiyou_io_busy_skip_poll";
    return true;
  }
  ++read_attempts;
  out_state.available = false;
  out_state.online = false;

  const bool strict_replay = []() {
      const char * v = std::getenv("ROBOT_ARM_YIYOU_STRICT_REPLAY");
      return v != nullptr && std::string(v) == "1";
    }();
  const char * yiyou_speed_cmd_env = std::getenv("ROBOT_ARM_YIYOU_BRIDGE_SPEED_CMD");
  std::string speed_cmd = (yiyou_speed_cmd_env != nullptr && std::strlen(yiyou_speed_cmd_env) > 0) ?
    std::string(yiyou_speed_cmd_env) : std::string("S6\r");
  if (!speed_cmd.empty() && speed_cmd.back() == '\n') {
    speed_cmd.back() = '\r';
  }
  if (speed_cmd.empty() || speed_cmd.back() != '\r') {
    speed_cmd.push_back('\r');
  }
  const std::vector<std::string> init_cmds = strict_replay ?
    std::vector<std::string>{"C\r", "S8\r", "M0\r", "A1\r", "O\r"} :
    std::vector<std::string>{"C\r", speed_cmd, "M0\r", "A1\r", "O\r"};
  if (!ensure_open("Yiyou", bridge_device, fd, initialized, init_cmds)) {
    return false;
  }
  if (strict_replay && !strict_replay_done) {
    strict_replay_done = true;
    const std::vector<std::string> strict_steps = {
      "t0026010F00000005\r",
      "t0026011000000001\r",
      "t00280109000100000000\r",
      "t00220307\r"};
    RCLCPP_INFO(rclcpp::get_logger("RealMixedRobotBackend"), "[Yiyou][STRICT_REPLAY] start");
    for (size_t i = 0; i < strict_steps.size(); ++i) {
      const auto & step = strict_steps[i];
      RCLCPP_INFO(
        rclcpp::get_logger("RealMixedRobotBackend"),
        "[Yiyou][STRICT_REPLAY] step=%zu tx_ascii='%s' tx_hex=%s",
        i + 1,
        ascii_escaped(step).c_str(),
        hex_digest(step).c_str());
      if (!send_query("Yiyou", fd, step, send_failures, read_attempts)) {
        RCLCPP_WARN(
          rclcpp::get_logger("RealMixedRobotBackend"),
          "[Yiyou][STRICT_REPLAY] step=%zu send_failed",
          i + 1);
        continue;
      }
      std::string step_raw;
      read_raw_frame(fd, step_raw, recv_timeouts, "Yiyou", read_attempts);
      RCLCPP_INFO(
        rclcpp::get_logger("RealMixedRobotBackend"),
        "[Yiyou][STRICT_REPLAY] step=%zu rx_len=%zu rx_ascii='%s' rx_hex=%s classify=%s",
        i + 1,
        step_raw.size(),
        ascii_escaped(step_raw).c_str(),
        hex_digest(step_raw).c_str(),
        step_raw.empty() ? "no_reply" : "has_reply");
      if (!step_raw.empty()) {
        SlcanStdFrame frame{};
        std::string reason;
        if (parse_slcan_t_frame(step_raw, frame, reason) && frame.payload.size() >= 2) {
          int32_t value = 0;
          if (frame.payload.size() >= 6) {
            value = static_cast<int32_t>(
              (static_cast<uint32_t>(frame.payload[2]) << 24) |
              (static_cast<uint32_t>(frame.payload[3]) << 16) |
              (static_cast<uint32_t>(frame.payload[4]) << 8) |
              static_cast<uint32_t>(frame.payload[5]));
          }
          RCLCPP_INFO(
            rclcpp::get_logger("RealMixedRobotBackend"),
            "[Yiyou][STRICT_REPLAY] step=%zu parsed cmd=0x%02X addr=0x%02X value=%d",
            i + 1,
            static_cast<unsigned int>(frame.payload[0]),
            static_cast<unsigned int>(frame.payload[1]),
            static_cast<int>(value));
        } else {
          RCLCPP_WARN(
            rclcpp::get_logger("RealMixedRobotBackend"),
            "[Yiyou][STRICT_REPLAY] step=%zu parse_failed reason=%s",
            i + 1,
            reason.c_str());
        }
      }
    }
  }

  // Hot-path policy: default disable shell-out Python adapter in per-cycle read.
  // Enable only for diagnostics via ROBOT_ARM_ENABLE_ADAPTER_SHELLOUT=1.
  const bool enable_adapter_shellout = []() {
      const char * v = std::getenv("ROBOT_ARM_ENABLE_ADAPTER_SHELLOUT");
      return v != nullptr && std::string(v) == "1";
    }();
  if (enable_adapter_shellout) {
    double py_pos = 0.0;
    double py_vel = 0.0;
    int py_enable = 0;
    std::string py_reason;
    if (LegacyProtocolDeviceActions::read_yiyou_state_via_adapter(
        bridge_device, route.node_id, py_pos, py_vel, py_enable, py_reason))
    {
      out_state.position = py_pos;
      out_state.velocity = py_vel;
      out_state.available = true;
      out_state.online = true;
      ++valid_samples;
      if (read_attempts <= 3 || read_attempts % 100 == 0) {
        RCLCPP_INFO(
          rclcpp::get_logger("RealMixedRobotBackend"),
          "[Yiyou] adapter-backed read ok enable=%d position=%.6f velocity=%.6f",
          py_enable,
          out_state.position,
          out_state.velocity);
      }
      return true;
    }
    RCLCPP_WARN(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[Yiyou] adapter-backed read failed, fallback to legacy protocol path: reason=%s",
      py_reason.c_str());
  }

  const auto read_u32 = [&](uint8_t reg_addr, int32_t & out, const char * reg_name) -> bool {
      // Legacy protocol path: temporary C++ action boundary call.
      const std::string query = LegacyProtocolDeviceActions::yiyou_read_register(route, reg_addr);
      RCLCPP_INFO(
        rclcpp::get_logger("RealMixedRobotBackend"),
        "[Yiyou] raw tx reg=%s ascii='%s' hex=%s",
        reg_name,
        ascii_escaped(query).c_str(),
        hex_digest(query).c_str());
      if (!send_query("Yiyou", fd, query, send_failures, read_attempts)) {
        return false;
      }
      std::string raw;
      if (!read_raw_frame(fd, raw, recv_timeouts, "Yiyou", read_attempts)) {
        return false;
      }
      RCLCPP_INFO(
        rclcpp::get_logger("RealMixedRobotBackend"),
        "[Yiyou] raw rx reg=%s raw_len=%zu ascii='%s' hex=%s",
        reg_name,
        raw.size(),
        ascii_escaped(raw).c_str(),
        hex_digest(raw).c_str());
      if (raw.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("RealMixedRobotBackend"), "[Yiyou] no reply reg=%s", reg_name);
        return false;
      }
      std::string reason;
      if (!LegacyProtocolDeviceActions::parse_yiyou_read_reply(raw, reg_addr, out, reason)) {
        RCLCPP_WARN(
          rclcpp::get_logger("RealMixedRobotBackend"),
          "[Yiyou] parse failed reg=%s reason=%s raw_ascii='%s' raw_hex=%s",
          reg_name,
          reason.c_str(),
          ascii_escaped(raw).c_str(),
          hex_digest(raw).c_str());
        return false;
      }
      RCLCPP_INFO(
        rclcpp::get_logger("RealMixedRobotBackend"),
        "[Yiyou] parsed reg=%s cmd=0x04 addr=0x%02X value=%d",
        reg_name,
        static_cast<unsigned int>(reg_addr),
        static_cast<int>(out));
      return true;
    };

  int32_t pos_raw = 0;
  int32_t vel_raw = 0;
  int32_t en_raw = 0;
  int32_t alarm_raw = 0;
  const bool pos_ok = read_u32(0x07, pos_raw, "position_0x07");
  const bool vel_ok = read_u32(0x06, vel_raw, "velocity_0x06");
  const bool en_ok = read_u32(0x10, en_raw, "enable_0x10");
  const bool alarm_ok = read_u32(0x15, alarm_raw, "alarm_0x15");
  if (en_ok && alarm_ok && (read_attempts <= 3 || read_attempts % 100 == 0)) {
    RCLCPP_INFO(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[Yiyou] parsed status enable=%d alarm=%d",
      static_cast<int>(en_raw),
      static_cast<int>(alarm_raw));
  }
  if (pos_ok || vel_ok) {
    // Yiyou engineering units:
    // - position register raw -> turns by /65536
    // - velocity register raw -> rpm by *60/65536
    out_state.position = static_cast<double>(pos_raw) / 65536.0;
    out_state.velocity = static_cast<double>(vel_raw) * 60.0 / 65536.0;
    out_state.available = true;
    out_state.online = true;
    ++valid_samples;
    return true;
  }
  out_state.available = false;
  out_state.online = false;
  return true;
}

bool RealMixedRobotBackend::configure(const JointRouteTable & routes)
{
  stop_polling_worker();
  routes_ = routes;
  hightorque_routes_.clear();
  yiyou_routes_.clear();
  discovered_.clear();
  last_command_position_.clear();
  hightorque_hold_targets_.clear();
  hightorque_hold_ready_.clear();
  hightorque_last_sent_position_.clear();
  hightorque_last_send_time_sec_.clear();
  hightorque_desired_commands_.clear();
  hightorque_filtered_velocity_.clear();
  hightorque_last_velocity_target_.clear();
  hightorque_last_velocity_target_time_sec_.clear();
  hightorque_joint_mit2_config_.clear();
  hightorque_position_hold_active_ = false;
  write_path_warned_ = false;
  {
    std::scoped_lock<std::mutex> lock(cache_mutex_);
    latest_cache_.clear();
    latest_poll_result_.clear();
    last_good_cache_.clear();
    last_good_time_sec_.clear();
    for (const auto & route : routes_) {
      JointState s{};
      s.online = false;
      s.available = false;
      s.stale = true;
      s.source = "init_default";
      s.last_error = "no_sample_yet";
      latest_cache_[route.joint_name] = s;
      latest_poll_result_[route.joint_name] = s;

      if (route.driver == "hightorque_canfd") {
        hightorque_routes_.push_back(route);
      } else if (route.driver == "yiyou_can20a") {
        yiyou_routes_.push_back(route);
      }
    }
  }
  enabled_ = false;
  faulted_ = false;
  hightorque_mode_log_once_ = false;
  hightorque_command_family_ = "official_mit2_int32";
  hightorque_position_hold_supported_ = true;

  if (const char * v = std::getenv("ROBOT_ARM_HT_MIT2_MODEL")) {
    if (std::strlen(v) > 0) { hightorque_mit2_config_.model = std::string(v); }
  }
  if (const char * v = std::getenv("ROBOT_ARM_HT_MIT2_KP")) {
    hightorque_mit2_config_.kp = std::atof(v);
  }
  if (const char * v = std::getenv("ROBOT_ARM_HT_MIT2_KD")) {
    hightorque_mit2_config_.kd = std::atof(v);
  }
  if (const char * v = std::getenv("ROBOT_ARM_HT_MIT2_TQE_NM")) {
    hightorque_mit2_config_.tqe_nm = std::atof(v);
  }
  if (const char * v = std::getenv("ROBOT_ARM_HT_MIT2_MAX_VEL_RPS")) {
    hightorque_mit2_config_.max_velocity_rps = std::max(0.0, std::atof(v));
  }
  if (const char * v = std::getenv("ROBOT_ARM_HT_MIT2_VEL_LPF_ALPHA")) {
    hightorque_mit2_config_.vel_lpf_alpha = std::clamp(std::atof(v), 0.0, 1.0);
  }
  if (const char * v = std::getenv("ROBOT_ARM_HT_MIT2_PERIOD_MS")) {
    hightorque_mit2_config_.write_period_ms = std::max(5, std::atoi(v));
  }
  if (const char * v = std::getenv("ROBOT_ARM_SAMPLE_RECENCY_WINDOW_SEC")) {
    sample_recency_window_sec_ = std::max(0.05, std::atof(v));
  }
  if (const char * v = std::getenv("ROBOT_ARM_SYNC_WAIT_TIMEOUT_MS")) {
    sync_wait_timeout_ms_ = std::max(100, std::atoi(v));
  }

  if (const char * v = std::getenv("ROBOT_ARM_HT_BRIDGE_DEVICE")) {
    if (std::strlen(v) > 0) {
      hightorque_channel_.bridge_device = std::string(v);
    }
  }
  if (const char * v = std::getenv("ROBOT_ARM_YIYOU_BRIDGE_DEVICE")) {
    if (std::strlen(v) > 0) {
      yiyou_channel_.bridge_device = std::string(v);
    }
  }

  for (const auto & route : hightorque_routes_) {
    auto cfg = hightorque_mit2_config_;
    maybe_override_joint_mit2_model(route.joint_name, cfg.model);
    maybe_override_joint_mit2_param(route.joint_name, "KP", cfg.kp);
    maybe_override_joint_mit2_param(route.joint_name, "KD", cfg.kd);
    maybe_override_joint_mit2_param(route.joint_name, "TQE_NM", cfg.tqe_nm);
    maybe_override_joint_mit2_param(route.joint_name, "MAX_VEL_RPS", cfg.max_velocity_rps);
    maybe_override_joint_mit2_param(route.joint_name, "VEL_LPF_ALPHA", cfg.vel_lpf_alpha);
    maybe_override_joint_mit2_param(route.joint_name, "PERIOD_MS", cfg.write_period_ms);
    hightorque_joint_mit2_config_[route.joint_name] = cfg;
    RCLCPP_INFO(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "HIGHTORQUE_MIT2_CONFIG joint=%s model=%s kp=%.3f kd=%.3f tqe_nm=%.3f max_vel_rps=%.3f vel_lpf_alpha=%.3f period_ms=%d",
      route.joint_name.c_str(),
      cfg.model.c_str(),
      cfg.kp,
      cfg.kd,
      cfg.tqe_nm,
      cfg.max_velocity_rps,
      cfg.vel_lpf_alpha,
      cfg.write_period_ms);
  }

  RCLCPP_INFO(
    rclcpp::get_logger("RealMixedRobotBackend"),
    "HIGHTORQUE_COMMAND_FAMILY=%s POSITION_HOLD=%s (continuous official MIT2 int32 servo writer)",
    hightorque_command_family_.c_str(),
    hightorque_position_hold_supported_ ? "enabled" : "disabled");

  const char * debug_single_joint_env = std::getenv("ROBOT_ARM_HT_DEBUG_SINGLE_JOINT");
  const bool enable_single_joint =
    debug_single_joint_env != nullptr && std::string(debug_single_joint_env) == "1";
  hightorque_channel_.debug_single_joint = enable_single_joint;
  const char * debug_joint_name_env = std::getenv("ROBOT_ARM_HT_DEBUG_JOINT_NAME");
  if (debug_joint_name_env != nullptr && std::strlen(debug_joint_name_env) > 0) {
    hightorque_channel_.debug_joint_name = std::string(debug_joint_name_env);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("RealMixedRobotBackend"),
    "configure: hightorque debug_single_joint=%s debug_joint_name=%s",
    hightorque_channel_.debug_single_joint ? "true" : "false",
    hightorque_channel_.debug_joint_name.c_str());
  return routes_.size() == 6;
}

bool RealMixedRobotBackend::discover_joints()
{
  discovered_.clear();
  RCLCPP_INFO(rclcpp::get_logger("RealMixedRobotBackend"), "discover_joints: begin");

  if (!resolve_bridge_devices()) {
    RCLCPP_ERROR(rclcpp::get_logger("RealMixedRobotBackend"), "discover_joints: resolve_bridge_devices failed");
    faulted_ = true;
    return false;
  }

  if (!discover_hightorque_joints()) {
    RCLCPP_ERROR(rclcpp::get_logger("RealMixedRobotBackend"), "discover_joints: hightorque discover failed");
    faulted_ = true;
    return false;
  }
  if (!discover_yiyou_joints()) {
    RCLCPP_ERROR(rclcpp::get_logger("RealMixedRobotBackend"), "discover_joints: yiyou discover failed");
    faulted_ = true;
    return false;
  }

  for (const auto & route : routes_) {
    if (!discovered_[route.joint_name]) {
      RCLCPP_ERROR(
        rclcpp::get_logger("RealMixedRobotBackend"),
        "discover_joints: missing joint=%s driver=%s bus=%s node_id=%d",
        route.joint_name.c_str(),
        route.driver.c_str(),
        route.bus.c_str(),
        route.node_id);
      faulted_ = true;
      return false;
    }
    RCLCPP_INFO(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "discover_joints: route_ready joint=%s driver=%s bus=%s node_id=%d (mapping/device ready, not proof of live response)",
      route.joint_name.c_str(),
      route.driver.c_str(),
      route.bus.c_str(),
      route.node_id);
  }

  RCLCPP_INFO(
    rclcpp::get_logger("RealMixedRobotBackend"),
    "discover_joints: success (route registration only; runtime online/available determined by polling samples)");
  return true;
}

bool RealMixedRobotBackend::probe_hightorque_on_device(const std::string & device, const std::set<int> & node_ids)
{
  int fd = -1;
  bool initialized = false;
  if (!ensure_open("HightorqueProbe", device, fd, initialized, {"C\r", "S8\r", "Y5\r", "M0\r", "A1\r", "O\r"})) {
    return false;
  }
  bool ok = false;
  for (const int node_id : node_ids) {
    JointRoute route{};
    route.node_id = node_id;
    const std::string query = build_hightorque_read_query(route);
    uint64_t send_fail = 0;
    uint64_t read_timeout = 0;
    if (!send_query("HightorqueProbe", fd, query, send_fail, 1)) {
      continue;
    }
    std::string raw;
    if (!read_raw_frame(fd, raw, read_timeout, "HightorqueProbe", 1) || raw.empty()) {
      continue;
    }
    double pos = 0.0;
    double vel = 0.0;
    uint32_t can_id = 0;
    std::string reason;
    if (!parse_hightorque_full_status(raw, pos, vel, reason, can_id)) {
      continue;
    }
    if (hightorque_node_id_from_can_id(can_id) == static_cast<uint8_t>(node_id)) {
      ok = true;
      break;
    }
  }
  if (fd >= 0) {
    ::close(fd);
  }
  return ok;
}

bool RealMixedRobotBackend::probe_yiyou_on_device(const std::string & device, int node_id)
{
  int fd = -1;
  bool initialized = false;
  if (!ensure_open("YiyouProbe", device, fd, initialized, {"C\r", "S8\r", "M0\r", "A1\r", "O\r"})) {
    return false;
  }
  JointRoute route{};
  route.node_id = node_id;
  const std::string query = build_yiyou_read_query(route, 0x07);
  uint64_t send_fail = 0;
  uint64_t read_timeout = 0;
  bool ok = false;
  if (send_query("YiyouProbe", fd, query, send_fail, 1)) {
    std::string raw;
    if (read_raw_frame(fd, raw, read_timeout, "YiyouProbe", 1) && !raw.empty()) {
      int32_t value = 0;
      std::string reason;
      ok = parse_yiyou_read_u32_reply(raw, 0x07, value, reason);
    }
  }
  if (fd >= 0) {
    ::close(fd);
  }
  return ok;
}

bool RealMixedRobotBackend::resolve_bridge_devices()
{
  const auto env_or_empty = [](const char * key) -> std::string {
      if (const char * v = std::getenv(key)) {
        if (std::strlen(v) > 0) {
          return std::string(v);
        }
      }
      return {};
    };
  const std::string ht_explicit = env_or_empty("ROBOT_ARM_HT_BRIDGE_DEVICE");
  const std::string yy_explicit = env_or_empty("ROBOT_ARM_YIYOU_BRIDGE_DEVICE");
  if (!ht_explicit.empty() && !yy_explicit.empty()) {
    hightorque_channel_.bridge_device = ht_explicit;
    yiyou_channel_.bridge_device = yy_explicit;
    RCLCPP_INFO(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "bridge bind: explicit config hightorque=%s yiyou=%s",
      hightorque_channel_.bridge_device.c_str(),
      yiyou_channel_.bridge_device.c_str());
    return true;
  }

  std::vector<std::string> candidates;
  const auto push_unique = [&](const std::string & path) {
      if (!path.empty() && std::filesystem::exists(path) &&
        std::find(candidates.begin(), candidates.end(), path) == candidates.end())
      {
        candidates.push_back(path);
      }
    };
  const auto collect_dir = [&](const std::string & dir) {
      if (!std::filesystem::exists(dir)) {
        return;
      }
      for (const auto & entry : std::filesystem::directory_iterator(dir)) {
        if (entry.is_symlink() || entry.is_character_file()) {
          try {
            push_unique(std::filesystem::canonical(entry.path()).string());
          } catch (...) {
            push_unique(entry.path().string());
          }
        }
      }
    };
  collect_dir("/dev/serial/by-id");
  collect_dir("/dev/serial/by-path");
  for (const auto & entry : std::filesystem::directory_iterator("/dev")) {
    const auto name = entry.path().filename().string();
    if (name.rfind("ttyACM", 0) == 0) {
      push_unique(entry.path().string());
    }
  }

  std::set<int> ht_nodes;
  for (const auto & route : hightorque_routes_) {
    ht_nodes.insert(route.node_id);
  }
  int yy_node = 2;
  for (const auto & route : yiyou_routes_) {
    yy_node = route.node_id;
    break;
  }

  std::string ht_found = ht_explicit;
  std::string yy_found = yy_explicit;
  RCLCPP_INFO(
    rclcpp::get_logger("RealMixedRobotBackend"),
    "bridge probe candidates=%zu explicit_ht=%s explicit_yiyou=%s",
    candidates.size(),
    ht_explicit.empty() ? "<none>" : ht_explicit.c_str(),
    yy_explicit.empty() ? "<none>" : yy_explicit.c_str());

  for (const auto & dev : candidates) {
    if (ht_found.empty()) {
      const bool ok = probe_hightorque_on_device(dev, ht_nodes);
      RCLCPP_INFO(
        rclcpp::get_logger("RealMixedRobotBackend"),
        "probe hightorque device=%s result=%s",
        dev.c_str(),
        ok ? "ok" : "fail");
      if (ok) {
        ht_found = dev;
      }
    }
    if (yy_found.empty() && dev != ht_found) {
      const bool ok = probe_yiyou_on_device(dev, yy_node);
      RCLCPP_INFO(
        rclcpp::get_logger("RealMixedRobotBackend"),
        "probe yiyou device=%s result=%s",
        dev.c_str(),
        ok ? "ok" : "fail");
      if (ok) {
        yy_found = dev;
      }
    }
  }

  if (ht_found.empty() || yy_found.empty() || ht_found == yy_found) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "bridge probe failed hightorque=%s yiyou=%s",
      ht_found.empty() ? "<none>" : ht_found.c_str(),
      yy_found.empty() ? "<none>" : yy_found.c_str());
    return false;
  }
  hightorque_channel_.bridge_device = ht_found;
  yiyou_channel_.bridge_device = yy_found;
  RCLCPP_INFO(
    rclcpp::get_logger("RealMixedRobotBackend"),
    "bridge bind result: hightorque=%s yiyou=%s",
    hightorque_channel_.bridge_device.c_str(),
    yiyou_channel_.bridge_device.c_str());
  return true;
}

bool RealMixedRobotBackend::sync_current_positions(std::vector<JointState> & states)
{
  // Activation safety rule:
  // HighTorque joints must have recent valid samples before activation succeeds.
  start_polling_worker();

  const auto t0 = std::chrono::steady_clock::now();
  while (true) {
    bool all_ready = true;
    {
      std::scoped_lock<std::mutex> lock(cache_mutex_);
      for (const auto & route : hightorque_routes_) {
        const auto it = last_good_time_sec_.find(route.joint_name);
        if (it == last_good_time_sec_.end()) {
          all_ready = false;
          break;
        }
        const double age = now_monotonic_sec() - it->second;
        if (age > sample_recency_window_sec_) {
          all_ready = false;
          break;
        }
      }
    }
    if (all_ready) {
      break;
    }
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - t0).count();
    if (elapsed_ms >= sync_wait_timeout_ms_) {
      RCLCPP_ERROR(
        rclcpp::get_logger("RealMixedRobotBackend"),
        "sync_current_positions: timeout waiting recent hightorque samples (%ldms)",
        static_cast<long>(elapsed_ms));
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  states.clear();
  states.reserve(routes_.size());
  {
    std::scoped_lock<std::mutex> lock(cache_mutex_);
    for (const auto & route : routes_) {
      const auto it = last_good_cache_.find(route.joint_name);
      if (it != last_good_cache_.end()) {
        states.push_back(it->second);
        last_command_position_[route.joint_name] = it->second.position;
        if (route.driver == "hightorque_canfd") {
          hightorque_hold_targets_[route.joint_name] = it->second.position;
          hightorque_hold_ready_[route.joint_name] = it->second.available && it->second.online;
        }
      } else {
        RCLCPP_ERROR(
          rclcpp::get_logger("RealMixedRobotBackend"),
          "sync_current_positions: missing last_good sample joint=%s driver=%s",
          route.joint_name.c_str(),
          route.driver.c_str());
        return false;
      }
    }
  }
  RCLCPP_INFO(
    rclcpp::get_logger("RealMixedRobotBackend"),
    "sync_current_positions: seeded from recent last_good samples");
  return true;
}

bool RealMixedRobotBackend::read_all_joint_states(std::vector<JointState> & states)
{
  static uint64_t read_cycles = 0;
  ++read_cycles;
  states.clear();
  states.reserve(routes_.size());

  {
    std::scoped_lock<std::mutex> lock(cache_mutex_);
    for (const auto & route : routes_) {
      JointState snapshot{};
      const auto good_it = last_good_cache_.find(route.joint_name);
      const auto t_it = last_good_time_sec_.find(route.joint_name);
      if (good_it != last_good_cache_.end() && t_it != last_good_time_sec_.end()) {
        snapshot = good_it->second;
        const double age_sec = now_monotonic_sec() - t_it->second;
        if (age_sec <= sample_recency_window_sec_) {
          snapshot.available = true;
          snapshot.online = true;
          snapshot.stale = false;
          snapshot.last_error.clear();
        } else {
          snapshot.available = false;
          snapshot.online = false;
          snapshot.stale = true;
          snapshot.last_error = "last_good_stale";
        }
        snapshot.last_update_time_sec = t_it->second;
        snapshot.source = "last_good_cache";
      } else {
        snapshot.available = false;
        snapshot.online = false;
        snapshot.stale = true;
        snapshot.source = "read_default";
        snapshot.last_error = "no_last_good_sample";
      }
      states.push_back(snapshot);
    }
  }

  if (read_cycles % 200 == 0) {
    RCLCPP_INFO(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "read_all_joint_states: cycle=%lu completed for %zu joints (cache snapshot only; no serial IO in read path)",
      static_cast<unsigned long>(read_cycles),
      states.size());
  }

  return true;
}

bool RealMixedRobotBackend::read_latest_position_turns(const std::string & joint_name, double & position_turns)
{
  JointState state{};
  if (!get_recent_last_good_state(joint_name, state, sample_recency_window_sec_)) {
    return false;
  }
  position_turns = state.position;
  return true;
}

bool RealMixedRobotBackend::send_hightorque_action(
  const JointRoute & route,
  const std::string & semantic,
  const std::string & frame,
  bool wait_reply)
{
  if (!ensure_open(
      "Hightorque", hightorque_channel_.bridge_device, hightorque_channel_.fd, hightorque_channel_.initialized,
      {"C\r", "S8\r", "Y5\r", "M0\r", "A1\r", "O\r"}))
  {
    faulted_ = true;
    return false;
  }

  std::scoped_lock<std::mutex> io_lock(hightorque_channel_.io_mutex);
  ++hightorque_channel_.read_attempts;
  RCLCPP_INFO(
    rclcpp::get_logger("RealMixedRobotBackend"),
    "[Hightorque][WRITE] semantic=%s joint=%s node_id=%d tx_ascii='%s' tx_hex=%s",
    semantic.c_str(),
    route.joint_name.c_str(),
    route.node_id,
    ascii_escaped(frame).c_str(),
    hex_digest(frame).c_str());

  if (!send_query(
      "Hightorque", hightorque_channel_.fd, frame, hightorque_channel_.send_failures,
      hightorque_channel_.read_attempts, wait_reply))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[Hightorque][WRITE] semantic=%s joint=%s send failed",
      semantic.c_str(),
      route.joint_name.c_str());
    return false;
  }

  if (!wait_reply) {
    return true;
  }

  std::string raw;
  read_raw_frame(
    hightorque_channel_.fd, raw, hightorque_channel_.recv_timeouts, "Hightorque", hightorque_channel_.read_attempts);
  if (raw.empty()) {
    RCLCPP_WARN(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[Hightorque][WRITE] semantic=%s joint=%s reply timeout",
      semantic.c_str(),
      route.joint_name.c_str());
    return true;
  }
  RCLCPP_INFO(
    rclcpp::get_logger("RealMixedRobotBackend"),
    "[Hightorque][WRITE] semantic=%s joint=%s rx_ascii='%s' rx_hex=%s",
    semantic.c_str(),
    route.joint_name.c_str(),
    ascii_escaped(raw).c_str(),
    hex_digest(raw).c_str());
  return true;
}

bool RealMixedRobotBackend::send_yiyou_write_u32(
  const JointRoute & route,
  uint8_t reg_addr,
  int32_t value,
  const std::string & semantic)
{
  const std::vector<std::string> init_cmds{"C\r", "S8\r", "M0\r", "A1\r", "O\r"};
  if (!ensure_open("Yiyou", yiyou_channel_.bridge_device, yiyou_channel_.fd, yiyou_channel_.initialized, init_cmds)) {
    faulted_ = true;
    return false;
  }

  const std::string query = build_yiyou_write_u32_query(route, reg_addr, value);
  std::scoped_lock<std::mutex> io_lock(yiyou_channel_.io_mutex);
  ++yiyou_channel_.read_attempts;
  RCLCPP_INFO(
    rclcpp::get_logger("RealMixedRobotBackend"),
    "[Yiyou][WRITE] semantic=%s joint=%s node_id=%d reg=0x%02X value=%d tx_ascii='%s' tx_hex=%s",
    semantic.c_str(),
    route.joint_name.c_str(),
    route.node_id,
    static_cast<unsigned int>(reg_addr),
    static_cast<int>(value),
    ascii_escaped(query).c_str(),
    hex_digest(query).c_str());

  if (!send_query("Yiyou", yiyou_channel_.fd, query, yiyou_channel_.send_failures, yiyou_channel_.read_attempts)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[Yiyou][WRITE] semantic=%s joint=%s send failed",
      semantic.c_str(),
      route.joint_name.c_str());
    return false;
  }

  std::string raw;
  read_raw_frame(yiyou_channel_.fd, raw, yiyou_channel_.recv_timeouts, "Yiyou", yiyou_channel_.read_attempts);
  if (raw.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[Yiyou][WRITE] semantic=%s joint=%s reply timeout",
      semantic.c_str(),
      route.joint_name.c_str());
    return false;
  }

  SlcanStdFrame frame{};
  std::string reason;
  if (!parse_slcan_t_frame(raw, frame, reason) || frame.payload.size() < 2) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[Yiyou][WRITE] semantic=%s joint=%s parse failed reason=%s rx_ascii='%s'",
      semantic.c_str(),
      route.joint_name.c_str(),
      reason.c_str(),
      ascii_escaped(raw).c_str());
    return false;
  }
  if (frame.payload[1] != reg_addr) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "[Yiyou][WRITE] semantic=%s joint=%s unexpected addr rx=0x%02X expect=0x%02X",
      semantic.c_str(),
      route.joint_name.c_str(),
      static_cast<unsigned int>(frame.payload[1]),
      static_cast<unsigned int>(reg_addr));
    return false;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("RealMixedRobotBackend"),
    "[Yiyou][WRITE] semantic=%s joint=%s reply ok cmd=0x%02X addr=0x%02X",
    semantic.c_str(),
    route.joint_name.c_str(),
    static_cast<unsigned int>(frame.payload[0]),
    static_cast<unsigned int>(frame.payload[1]));
  return true;
}

bool RealMixedRobotBackend::write_all_joint_commands(const std::vector<JointCommand> & commands)
{
  const auto write_begin = std::chrono::steady_clock::now();
  if (faulted_) {
    RCLCPP_ERROR(rclcpp::get_logger("RealMixedRobotBackend"), "write_all_joint_commands: rejected faulted backend");
    return false;
  }

  if (!enabled_) {
    static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("RealMixedRobotBackend"),
      steady_clock,
      1000,
      "write_all_joint_commands: backend disabled, skip write");
    return true;
  }

  if (commands.size() != routes_.size()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "write_all_joint_commands: size mismatch commands=%zu routes=%zu",
      commands.size(),
      routes_.size());
    faulted_ = true;
    return false;
  }

  bool has_motion_delta = false;
  constexpr double kCommandDeltaEpsTurns = 1e-4;
  constexpr double kYiyouMoveDeadbandTurns = 2e-4;
  bool fatal_error = false;
  uint64_t hightorque_enqueued = 0;
  uint64_t yiyou_write_count = 0;
  uint64_t yiyou_write_us = 0;
  const double now_sec = now_monotonic_sec();

  {
    std::scoped_lock<std::mutex> tx_lock(hightorque_tx_mutex_);
    for (size_t i = 0; i < commands.size(); ++i) {
      const auto & route = routes_[i];
      const double target = commands[i].position;
      if (!std::isfinite(target)) {
        RCLCPP_ERROR(
          rclcpp::get_logger("RealMixedRobotBackend"),
          "write_all_joint_commands: invalid command joint=%s value=%f",
          route.joint_name.c_str(),
          target);
        faulted_ = true;
        return false;
      }

      const auto it = last_command_position_.find(route.joint_name);
      const double prev = (it == last_command_position_.end()) ? target : it->second;
      const bool target_updated = std::abs(target - prev) > kCommandDeltaEpsTurns;
      if (target_updated) {
        has_motion_delta = true;
      }

      if (route.driver == "hightorque_canfd") {
        auto & desired = hightorque_desired_commands_[route.joint_name];
        desired.position_turns = target;
        desired.stamp_sec = now_sec;
        desired.valid = true;
        if (target_updated) {
          hightorque_hold_targets_[route.joint_name] = target;
        }
        ++hightorque_enqueued;
      } else if (route.driver == "yiyou_can20a") {
        if (std::abs(target - prev) > kYiyouMoveDeadbandTurns) {
          const int32_t target_raw = static_cast<int32_t>(std::llround(target * 65536.0));
          const auto t0 = std::chrono::steady_clock::now();
          if (!send_yiyou_write_u32(route, 0x09, 0x00010000, "write_target_speed")) {
            fatal_error = true;
            break;
          }
          if (!send_yiyou_write_u32(route, 0x0A, target_raw, "write_target_position")) {
            fatal_error = true;
            break;
          }
          const auto t1 = std::chrono::steady_clock::now();
          yiyou_write_us += static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());
          ++yiyou_write_count;
        }
      }

      last_command_position_[route.joint_name] = target;
    }
  }

  hightorque_tx_cv_.notify_all();

  if (fatal_error) {
    faulted_ = true;
    return false;
  }

  if (has_motion_delta && !write_path_warned_) {
    write_path_warned_ = true;
    RCLCPP_INFO(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "write_all_joint_commands: async real transport write path active");
  }

  static uint64_t write_cycles = 0;
  ++write_cycles;
  const auto write_end = std::chrono::steady_clock::now();
  const auto total_us = static_cast<uint64_t>(
    std::chrono::duration_cast<std::chrono::microseconds>(write_end - write_begin).count());
  if (write_cycles <= 10 || write_cycles % 100 == 0) {
    RCLCPP_INFO(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "write_all_joint_commands: cycle=%lu total_us=%lu hightorque_enqueued=%lu yiyou_writes=%lu yiyou_us=%lu",
      static_cast<unsigned long>(write_cycles),
      static_cast<unsigned long>(total_us),
      static_cast<unsigned long>(hightorque_enqueued),
      static_cast<unsigned long>(yiyou_write_count),
      static_cast<unsigned long>(yiyou_write_us));
  }
  return true;
}

bool RealMixedRobotBackend::enable()
{
  write_path_warned_ = false;
  faulted_ = false;
  hightorque_mode_log_once_ = false;
  hightorque_position_hold_active_ = false;
  hightorque_hold_targets_.clear();
  hightorque_hold_ready_.clear();

  RCLCPP_INFO(
    rclcpp::get_logger("RealMixedRobotBackend"),
    "HIGHTORQUE_MODE select=official_mit2_int32_enable HIGHTORQUE_COMMAND_FAMILY=%s POSITION_HOLD=%s",
    hightorque_command_family_.c_str(),
    hightorque_position_hold_active_ ? "active" : "inactive");

  struct PendingSeed
  {
    JointRoute route;
    double target_turns{0.0};
    HightorqueMit2Config cfg{};
  };
  std::vector<PendingSeed> pending_seeds;
  pending_seeds.reserve(hightorque_routes_.size());

  // Phase A: verify all required HighTorque samples without sending any frame.
  for (const auto & route : routes_) {
    if (route.driver == "hightorque_canfd") {
      JointState sample{};
      if (!get_recent_last_good_state(route.joint_name, sample, sample_recency_window_sec_) ||
        !is_state_sample_good(sample))
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("RealMixedRobotBackend"),
          "HIGHTORQUE_ENABLE_REJECT joint=%s reason=no_recent_valid_sample",
          route.joint_name.c_str());
        enabled_ = false;
        hightorque_position_hold_active_ = false;
        return false;
      }
      const auto cfg_it = hightorque_joint_mit2_config_.find(route.joint_name);
      const auto & cfg = cfg_it != hightorque_joint_mit2_config_.end() ? cfg_it->second : hightorque_mit2_config_;
      pending_seeds.push_back(PendingSeed{route, sample.position, cfg});
      continue;
    }
    if (route.driver == "yiyou_can20a") {
      if (!send_yiyou_write_u32(route, 0x10, 1, "enable")) {
        faulted_ = true;
        return false;
      }
    }
  }

  // Phase B: all HighTorque joints passed verification, now seed and activate.
  for (const auto & seed : pending_seeds) {
    hightorque_hold_ready_[seed.route.joint_name] = true;
    hightorque_hold_targets_[seed.route.joint_name] = seed.target_turns;
    hightorque_last_sent_position_[seed.route.joint_name] = seed.target_turns;
    hightorque_last_send_time_sec_[seed.route.joint_name] = now_monotonic_sec();
    hightorque_filtered_velocity_[seed.route.joint_name] = 0.0;
    hightorque_last_velocity_target_[seed.route.joint_name] = seed.target_turns;
    hightorque_last_velocity_target_time_sec_[seed.route.joint_name] = now_monotonic_sec();
    {
      std::scoped_lock<std::mutex> tx_lock(hightorque_tx_mutex_);
      auto & desired = hightorque_desired_commands_[seed.route.joint_name];
      desired.position_turns = seed.target_turns;
      desired.stamp_sec = now_monotonic_sec();
      desired.valid = true;
    }
    RCLCPP_INFO(
      rclcpp::get_logger("RealMixedRobotBackend"),
      "HIGHTORQUE_HOLD_INIT joint=%s hold_target_source=valid_sample frozen_hold_target_turns=%.6f kp=%.3f kd=%.3f tqe_nm=%.3f",
      seed.route.joint_name.c_str(),
      seed.target_turns,
      seed.cfg.kp,
      seed.cfg.kd,
      seed.cfg.tqe_nm);

    const std::string hold_frame = LegacyProtocolDeviceActions::hightorque_move_position_target(
      seed.route, seed.target_turns, 0.0, seed.cfg.tqe_nm, seed.cfg.kp, seed.cfg.kd, seed.cfg.model);
    if (!send_hightorque_action(seed.route, "mit2_enable_hold_seed", hold_frame, false)) {
      faulted_ = true;
      enabled_ = false;
      hightorque_position_hold_active_ = false;
      return false;
    }
  }

  hightorque_position_hold_active_ = true;
  enabled_ = true;
  hightorque_tx_cv_.notify_all();
  return true;
}

bool RealMixedRobotBackend::disable()
{
  hightorque_position_hold_active_ = false;
  hightorque_tx_cv_.notify_all();
  for (const auto & route : routes_) {
    if (route.driver == "hightorque_canfd") {
      if (!send_hightorque_action(
          route, "disable_stop", LegacyProtocolDeviceActions::hightorque_stop(route), true))
      {
        return false;
      }
      continue;
    }
    if (route.driver == "yiyou_can20a") {
      if (!send_yiyou_write_u32(route, 0x10, 0, "disable")) {
        return false;
      }
    }
  }
  {
    std::scoped_lock<std::mutex> tx_lock(hightorque_tx_mutex_);
    hightorque_desired_commands_.clear();
  }
  hightorque_hold_targets_.clear();
  hightorque_hold_ready_.clear();
  hightorque_filtered_velocity_.clear();
  enabled_ = false;
  return true;
}

bool RealMixedRobotBackend::stop()
{
  bool ok = true;
  for (const auto & route : routes_) {
    if (route.driver == "hightorque_canfd") {
      ok = send_hightorque_action(route, "stop", LegacyProtocolDeviceActions::hightorque_stop(route), true) && ok;
      continue;
    }
    if (route.driver == "yiyou_can20a") {
      ok = send_yiyou_write_u32(route, 0x11, 1, "stop") && ok;
    }
  }
  enabled_ = false;
  faulted_ = true;
  return ok;
}

bool RealMixedRobotBackend::clear_fault()
{
  faulted_ = false;
  return true;
}

RealMixedRobotBackend::~RealMixedRobotBackend()
{
  stop_polling_worker();
}

bool RealMixedRobotBackend::discover_hightorque_joints()
{
  return hightorque_channel_.discover(routes_, discovered_);
}

bool RealMixedRobotBackend::get_recent_last_good_state(
  const std::string & joint_name, JointState & out_state, double max_age_sec)
{
  std::scoped_lock<std::mutex> lock(cache_mutex_);
  const auto it = last_good_cache_.find(joint_name);
  const auto t_it = last_good_time_sec_.find(joint_name);
  if (it == last_good_cache_.end() || t_it == last_good_time_sec_.end()) {
    return false;
  }
  const double age = now_monotonic_sec() - t_it->second;
  if (age > max_age_sec) {
    return false;
  }
  out_state = it->second;
  out_state.last_update_time_sec = t_it->second;
  return true;
}

bool RealMixedRobotBackend::discover_yiyou_joints()
{
  return yiyou_channel_.discover(routes_, discovered_);
}

void RealMixedRobotBackend::start_polling_worker()
{
  if (polling_running_.load()) {
    return;
  }
  polling_running_.store(true);
  hightorque_tx_running_.store(true);
  hightorque_polling_thread_ = std::thread(&RealMixedRobotBackend::polling_loop_hightorque, this);
  yiyou_polling_thread_ = std::thread(&RealMixedRobotBackend::polling_loop_yiyou, this);
  hightorque_tx_thread_ = std::thread(&RealMixedRobotBackend::hightorque_tx_loop, this);
  RCLCPP_INFO(rclcpp::get_logger("RealMixedRobotBackend"), "polling workers started (hightorque + yiyou + hightorque_tx)");
}

void RealMixedRobotBackend::stop_polling_worker()
{
  if (!polling_running_.load() && !hightorque_tx_running_.load()) {
    return;
  }
  polling_running_.store(false);
  hightorque_tx_running_.store(false);
  hightorque_tx_cv_.notify_all();
  if (hightorque_polling_thread_.joinable()) {
    hightorque_polling_thread_.join();
  }
  if (yiyou_polling_thread_.joinable()) {
    yiyou_polling_thread_.join();
  }
  if (hightorque_tx_thread_.joinable()) {
    hightorque_tx_thread_.join();
  }
  RCLCPP_INFO(rclcpp::get_logger("RealMixedRobotBackend"), "polling workers stopped");
}

void RealMixedRobotBackend::write_cache_locked(const std::string & joint_name, const JointState & state)
{
  std::scoped_lock<std::mutex> lock(cache_mutex_);
  latest_cache_[joint_name] = state;
}

void RealMixedRobotBackend::polling_loop_hightorque()
{
  uint64_t cycle = 0;
  while (polling_running_.load()) {
    ++cycle;
    const bool armed_poll = enabled_.load();
    size_t begin_idx = 0;
    size_t end_idx = hightorque_routes_.size();
    if (armed_poll && !hightorque_routes_.empty()) {
      begin_idx = static_cast<size_t>(cycle % hightorque_routes_.size());
      end_idx = begin_idx + 1;
    }

    for (size_t idx = begin_idx; idx < end_idx; ++idx) {
      const auto & route = hightorque_routes_[idx];
      JointState state{};
      const bool ok = hightorque_channel_.read_joint(route, state);
      const double now_sec = now_monotonic_sec();
      state.last_update_time_sec = now_sec;
      state.source = "hightorque_worker_latest_poll";

      {
        std::scoped_lock<std::mutex> lock(cache_mutex_);
        latest_poll_result_[route.joint_name] = state;
        if (ok && state.available && state.online && std::isfinite(state.position) && std::isfinite(state.velocity)) {
          JointState good = state;
          good.stale = false;
          good.last_error.clear();
          good.source = "hightorque_worker_last_good";
          last_good_cache_[route.joint_name] = good;
          last_good_time_sec_[route.joint_name] = now_sec;
          latest_cache_[route.joint_name] = good;
          hightorque_channel_.latest_states[route.joint_name] = good;
        } else {
          const auto good_it = last_good_cache_.find(route.joint_name);
          const auto t_it = last_good_time_sec_.find(route.joint_name);
          if (good_it != last_good_cache_.end() && t_it != last_good_time_sec_.end()) {
            JointState snapshot = good_it->second;
            const double age = now_sec - t_it->second;
            snapshot.available = age <= sample_recency_window_sec_;
            snapshot.online = snapshot.available;
            snapshot.stale = !snapshot.available;
            snapshot.last_error = snapshot.available ? "" : "last_good_stale";
            snapshot.last_update_time_sec = t_it->second;
            snapshot.source = "hightorque_worker_last_good_snapshot";
            latest_cache_[route.joint_name] = snapshot;
          } else {
            JointState missing{};
            missing.available = false;
            missing.online = false;
            missing.stale = true;
            missing.last_error = "hightorque_no_sample";
            missing.last_update_time_sec = now_sec;
            missing.source = "hightorque_worker_no_sample";
            latest_cache_[route.joint_name] = missing;
          }
        }
      }

      if (!ok && !faulted_) {
        RCLCPP_WARN(
          rclcpp::get_logger("RealMixedRobotBackend"),
          "polling_loop_hightorque: read failed for joint=%s; keep cached/fallback state",
          route.joint_name.c_str());
      }
      write_cache_locked(route.joint_name, state);
    }
    if (cycle % 100 == 0) {
      RCLCPP_INFO(
        rclcpp::get_logger("RealMixedRobotBackend"),
        "polling_loop_hightorque: mode=%s cycle=%lu refreshed=%zu/%zu joints (decimation=%s)",
        armed_poll ? "ARMED poll" : "UNARMED poll",
        static_cast<unsigned long>(cycle),
        armed_poll ? static_cast<size_t>(1) : hightorque_routes_.size(),
        hightorque_routes_.size(),
        armed_poll ? "enabled skip poll(all->round_robin_1)" : "none");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(armed_poll ? 20 : 10));
  }
}


void RealMixedRobotBackend::hightorque_tx_loop()
{
  uint64_t cycle = 0;
  while (hightorque_tx_running_.load()) {
    ++cycle;

    std::unique_lock<std::mutex> lk(hightorque_tx_mutex_);
    int period_ms = hightorque_mit2_config_.write_period_ms;
    for (const auto & item : hightorque_joint_mit2_config_) {
      period_ms = std::min(period_ms, item.second.write_period_ms);
    }
    period_ms = std::max(5, period_ms);
    hightorque_tx_cv_.wait_for(
      lk,
      std::chrono::milliseconds(period_ms),
      [this]() {return !hightorque_tx_running_.load() || !polling_running_.load() || enabled_.load();});

    if (!hightorque_tx_running_.load() || !polling_running_.load()) {
      break;
    }

    std::vector<std::tuple<JointRoute, HightorqueDesiredCommand, HightorqueMit2Config>> jobs;
    jobs.reserve(hightorque_routes_.size());
    for (const auto & route : hightorque_routes_) {
      const bool hold_ready = hightorque_hold_ready_[route.joint_name];
      if (!enabled_.load() || !hightorque_position_hold_active_ || !hold_ready) {
        continue;
      }

      HightorqueDesiredCommand desired{};
      const auto desired_it = hightorque_desired_commands_.find(route.joint_name);
      if (desired_it != hightorque_desired_commands_.end() && desired_it->second.valid) {
        desired = desired_it->second;
      } else {
        desired.position_turns = hightorque_hold_targets_[route.joint_name];
        desired.stamp_sec = now_monotonic_sec();
        desired.valid = true;
      }
      const auto cfg_it = hightorque_joint_mit2_config_.find(route.joint_name);
      const auto cfg = cfg_it != hightorque_joint_mit2_config_.end() ? cfg_it->second : hightorque_mit2_config_;
      jobs.emplace_back(route, desired, cfg);
    }
    lk.unlock();

    uint64_t cycle_us = 0;
    for (const auto & job : jobs) {
      const auto & route = std::get<0>(job);
      const auto & desired = std::get<1>(job);
      const auto & cfg = std::get<2>(job);

      const double prev_target = hightorque_last_velocity_target_.count(route.joint_name) ?
        hightorque_last_velocity_target_[route.joint_name] : desired.position_turns;
      const double prev_target_t = hightorque_last_velocity_target_time_sec_.count(route.joint_name) ?
        hightorque_last_velocity_target_time_sec_[route.joint_name] : desired.stamp_sec;

      double raw_vel = 0.0;
      if (desired.stamp_sec > prev_target_t + 1e-6) {
        raw_vel = (desired.position_turns - prev_target) / std::max(0.001, desired.stamp_sec - prev_target_t);
      }
      const double vel_limit = std::max(0.1, cfg.max_velocity_rps);
      raw_vel = std::clamp(raw_vel, -vel_limit, vel_limit);

      const double prev_filtered = hightorque_filtered_velocity_.count(route.joint_name) ?
        hightorque_filtered_velocity_[route.joint_name] : 0.0;
      const double alpha = std::clamp(cfg.vel_lpf_alpha, 0.0, 1.0);
      const bool target_fresh = desired.stamp_sec > prev_target_t + 1e-6;
      const double filtered_vel = target_fresh ?
        (alpha * raw_vel + (1.0 - alpha) * prev_filtered) :
        ((1.0 - alpha) * prev_filtered);

      hightorque_filtered_velocity_[route.joint_name] = std::clamp(filtered_vel, -vel_limit, vel_limit);
      hightorque_last_velocity_target_[route.joint_name] = desired.position_turns;
      hightorque_last_velocity_target_time_sec_[route.joint_name] = desired.stamp_sec;

      const std::string frame = LegacyProtocolDeviceActions::hightorque_move_position_target(
        route,
        desired.position_turns,
        hightorque_filtered_velocity_[route.joint_name],
        cfg.tqe_nm,
        cfg.kp,
        cfg.kd,
        cfg.model);

      const auto t0 = std::chrono::steady_clock::now();
      if (!send_hightorque_action(route, "mit2_servo_cycle", frame, false)) {
        faulted_ = true;
        continue;
      }
      const auto t1 = std::chrono::steady_clock::now();
      cycle_us += static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());

      hightorque_last_sent_position_[route.joint_name] = desired.position_turns;
      hightorque_last_send_time_sec_[route.joint_name] = now_monotonic_sec();

      static rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      RCLCPP_INFO_THROTTLE(
        rclcpp::get_logger("RealMixedRobotBackend"),
        steady_clock,
        500,
        "HIGHTORQUE_MIT2_SERVO joint=%s target_turns=%.6f vel_feed_rps=%.6f kp=%.3f kd=%.3f tqe_nm=%.3f period_ms=%d",
        route.joint_name.c_str(),
        desired.position_turns,
        hightorque_filtered_velocity_[route.joint_name],
        cfg.kp,
        cfg.kd,
        cfg.tqe_nm,
        cfg.write_period_ms);
    }

    if (cycle % 100 == 0) {
      RCLCPP_INFO(
        rclcpp::get_logger("RealMixedRobotBackend"),
        "hightorque_tx_loop: cycle=%lu jobs=%zu total_us=%lu",
        static_cast<unsigned long>(cycle),
        jobs.size(),
        static_cast<unsigned long>(cycle_us));
    }
  }
}
void RealMixedRobotBackend::polling_loop_yiyou()
{
  uint64_t cycle = 0;
  while (polling_running_.load()) {
    ++cycle;
    const bool armed_poll = enabled_.load();
    const bool do_poll_this_cycle = !armed_poll || (cycle % 4 == 0);
    for (const auto & route : yiyou_routes_) {
      JointState state{};
      bool ok = true;
      if (do_poll_this_cycle) {
        ok = yiyou_channel_.read_joint(route, state);
      } else if (yiyou_channel_.latest_joint2_state.has_value()) {
        state = *yiyou_channel_.latest_joint2_state;
        state.available = true;
        state.online = true;
        state.stale = false;
      } else {
        state.available = false;
        state.online = false;
        state.stale = true;
        state.last_error = "yiyou_reduced_polling_no_cache";
      }
      if (ok && state.available) {
        yiyou_channel_.latest_joint2_state = state;
        state.stale = false;
        state.last_error.clear();
        {
          std::scoped_lock<std::mutex> lock(cache_mutex_);
          latest_poll_result_[route.joint_name] = state;
          last_good_cache_[route.joint_name] = state;
          last_good_time_sec_[route.joint_name] = now_monotonic_sec();
        }
      } else {
        if (yiyou_channel_.latest_joint2_state.has_value()) {
          state = *yiyou_channel_.latest_joint2_state;
          state.available = false;
          state.online = false;
        } else {
          state.available = false;
          state.online = false;
          state.position = 0.0;
          state.velocity = 0.0;
        }
        state.available = false;
        state.online = false;
        state.stale = true;
        state.last_error = "yiyou_query_not_implemented";
      }
      state.last_update_time_sec = now_monotonic_sec();
      state.source = "yiyou_worker";
      {
        std::scoped_lock<std::mutex> lock(cache_mutex_);
        latest_poll_result_[route.joint_name] = state;
        if (!is_state_sample_good(state)) {
          const auto good_it = last_good_cache_.find(route.joint_name);
          const auto t_it = last_good_time_sec_.find(route.joint_name);
          if (good_it != last_good_cache_.end() && t_it != last_good_time_sec_.end()) {
            JointState snapshot = good_it->second;
            const double age = now_monotonic_sec() - t_it->second;
            snapshot.available = age <= sample_recency_window_sec_;
            snapshot.online = snapshot.available;
            snapshot.stale = !snapshot.available;
            snapshot.last_error = snapshot.available ? "" : "last_good_stale";
            snapshot.last_update_time_sec = t_it->second;
            snapshot.source = "yiyou_worker_last_good_snapshot";
            latest_cache_[route.joint_name] = snapshot;
          } else {
            latest_cache_[route.joint_name] = state;
          }
        } else {
          latest_cache_[route.joint_name] = state;
        }
      }
    }
    if (cycle % 100 == 0) {
      RCLCPP_INFO(
        rclcpp::get_logger("RealMixedRobotBackend"),
        "polling_loop_yiyou: mode=%s cycle=%lu do_poll=%s reduced polling=%s joints=%zu",
        armed_poll ? "ARMED poll" : "UNARMED poll",
        static_cast<unsigned long>(cycle),
        do_poll_this_cycle ? "true" : "false",
        armed_poll ? "every_4_cycles(skip poll)" : "none",
        yiyou_routes_.size());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(armed_poll ? 20 : 20));
  }
}

}  // namespace robot_arm_hardware_system
