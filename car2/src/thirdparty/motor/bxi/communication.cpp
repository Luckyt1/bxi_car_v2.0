// SPDX-License-Identifier: Apache-2.0
// Adapted from bxirobotics/bxi_car chassis.cpp at a02e41f59373b36e38686ed8be83f3421f1959a4.
// Project adaptation: motor/ header layout and chassis CMake export.
// Changes: no ROS/Modbus, deterministic frames, parameter validation, bounded feedback parsing.
#include "motor/bxi/communication.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace bxi
{
namespace
{
bool valid_range(const Range & range)
{
  return std::isfinite(range.min) && std::isfinite(range.max) && range.min < range.max;
}

bool in_range(float value, const Range & range)
{
  return valid_range(range) && std::isfinite(value) && value >= range.min && value <= range.max;
}

std::uint32_t encode(float value, const Range & range, unsigned bits)
{
  // Keep the upstream float arithmetic and truncation for valid inputs.
  const auto maximum = (1U << bits) - 1U;
  const float scaled = (value - range.min) * static_cast<float>(maximum) /
    (range.max - range.min);
  // Custom finite endpoints can overflow intermediate float arithmetic.
  const double encoded = std::isfinite(scaled) ? static_cast<double>(scaled) :
    (static_cast<double>(value) - range.min) * maximum /
    (static_cast<double>(range.max) - range.min);
  return static_cast<std::uint32_t>(std::clamp(encoded, 0.0, static_cast<double>(maximum)));
}

float decode(std::uint32_t value, const Range & range, unsigned bits)
{
  const auto maximum = (1U << bits) - 1U;
  const float decoded = static_cast<float>(value) * (range.max - range.min) /
    static_cast<float>(maximum) + range.min;
  if (std::isfinite(decoded)) {return decoded;}
  return static_cast<float>(static_cast<double>(value) *
         (static_cast<double>(range.max) - range.min) / maximum + range.min);
}
}  // namespace

EncodingRanges encoding_ranges(Model model)
{
  // https://wiki.bxirobotics.cn/actuators/can_communication/ (2026-09-15)
  // 协议量化范围独立于机械额定/峰值扭矩，不用减速比再次缩放。
  EncodingRanges ranges;
  switch (model) {
    case Model::BXI5014_19:  // MOTOR_50
    case Model::BXI5018_19:  // MOTOR_50_L
      return ranges;
    case Model::BXI7010_19:  // MOTOR_70
      ranges.torque = {-80.0F, 80.0F};
      return ranges;
    case Model::BXI8515_19:  // MOTOR_85
      ranges.kd = {0.0F, 20.0F};
      ranges.torque = {-160.0F, 160.0F};
      return ranges;
  }
  throw std::invalid_argument("unknown BXI motor model");
}

bxi::Result<std::array<std::uint8_t, 8>> pack_command(const Command & command)
{
  using Result = bxi::Result<std::array<std::uint8_t, 8>>;
  const auto & ranges = command.ranges;
  if (!in_range(command.position, ranges.position) ||
    !in_range(command.velocity, ranges.velocity) ||
    !in_range(command.kp, ranges.kp) ||
    !in_range(command.kd, ranges.kd) ||
    !in_range(command.torque, ranges.torque))
  {
    return Result::failure(
      bxi::ErrorCode::invalid_argument,
      "BXI MIT command requires finite ordered encoding ranges and finite values within them");
  }
  const auto p = encode(command.position, ranges.position, 16);
  const auto v = encode(command.velocity, ranges.velocity, 12);
  const auto kp = encode(command.kp, ranges.kp, 12);
  const auto kd = encode(command.kd, ranges.kd, 12);
  const auto t = encode(command.torque, ranges.torque, 12);
  const std::array<std::uint8_t, 8> payload{
    static_cast<std::uint8_t>(p >> 8), static_cast<std::uint8_t>(p & 0xffU),
    static_cast<std::uint8_t>(v >> 4),
    static_cast<std::uint8_t>(((v & 0xfU) << 4) | (kp >> 8)),
    static_cast<std::uint8_t>(kp & 0xffU), static_cast<std::uint8_t>(kd >> 4),
    static_cast<std::uint8_t>(((kd & 0xfU) << 4) | (t >> 8)),
    static_cast<std::uint8_t>(t & 0xffU)};
  if (std::all_of(
      payload.begin(), payload.begin() + 7,
      [](std::uint8_t byte) {return byte == 0xff;}) &&
    (payload[7] >= 0xfa && payload[7] <= 0xfe))
  {
    return Result::failure(
      bxi::ErrorCode::invalid_argument,
      "BXI MIT command collides with a special control frame (0xFA..0xFE)");
  }
  return Result::success(payload);
}

bxi::Result<Feedback> decode_feedback(const bxi::CanFrame & frame, const EncodingRanges & ranges)
{
  if (!bxi::valid_frame(frame) || frame.extended || frame.remote || frame.error ||
    frame.size < 5)
  {
    return bxi::Result<Feedback>::failure(
      bxi::ErrorCode::protocol_error,
      "BXI feedback requires a valid standard data frame of at least five bytes");
  }
  if (!valid_range(ranges.position) || !valid_range(ranges.velocity)) {
    return bxi::Result<Feedback>::failure(
      bxi::ErrorCode::invalid_argument,
      "BXI feedback requires finite ordered position and velocity encoding ranges");
  }
  const auto p = (static_cast<std::uint32_t>(frame.data[1]) << 8) | frame.data[2];
  const auto v = (static_cast<std::uint32_t>(frame.data[3]) << 4) | (frame.data[4] >> 4);
  return bxi::Result<Feedback>::success(
    Feedback{frame.data[0], decode(p, ranges.position, 16), decode(v, ranges.velocity, 12)});
}

bxi::Result<void> send_payload(
  bxi::ICanTransport & transport, std::uint32_t command_id, FrameOptions options,
  const std::array<std::uint8_t, 8> & payload)
{
  if (command_id > 0x7ffU || (!options.fd && options.bitrate_switch)) {
    return bxi::Result<void>::failure(
      bxi::ErrorCode::invalid_argument,
      "BXI motor requires a standard CAN ID and valid frame options");
  }
  bxi::CanFrame frame{};
  frame.id = command_id;
  frame.size = 8;
  frame.fd = options.fd;
  frame.bitrate_switch = options.bitrate_switch;
  std::copy(payload.begin(), payload.end(), frame.data.begin());
  return transport.send(frame);
}
}  // namespace bxi
