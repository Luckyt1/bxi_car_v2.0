// SPDX-License-Identifier: Apache-2.0
// Adapted from bxirobotics/bxi_car chassis.cpp at a02e41f59373b36e38686ed8be83f3421f1959a4.
// Changes: no ROS/Modbus, deterministic frames, parameter validation, bounded feedback parsing.
#include "bxi/motor.hpp"

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

iswv::Result<std::array<std::uint8_t, 8>> pack_command(const Command & command)
{
  using Result = iswv::Result<std::array<std::uint8_t, 8>>;
  const auto & ranges = command.ranges;
  if (!in_range(command.position, ranges.position) ||
    !in_range(command.velocity, ranges.velocity) ||
    !in_range(command.kp, ranges.kp) ||
    !in_range(command.kd, ranges.kd) ||
    !in_range(command.torque, ranges.torque))
  {
    return Result::failure(iswv::ErrorCode::invalid_argument,
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
  if (std::all_of(payload.begin(), payload.begin() + 7,
    [](std::uint8_t byte) {return byte == 0xff;}) &&
    (payload[7] == 0xfc || payload[7] == 0xfd))
  {
    return Result::failure(iswv::ErrorCode::invalid_argument,
      "BXI MIT command collides with an enter/exit mode frame");
  }
  return Result::success(payload);
}

iswv::Result<Feedback> decode_feedback(const iswv::CanFrame & frame, const EncodingRanges & ranges)
{
  if (!iswv::valid_frame(frame) || frame.extended || frame.remote || frame.error ||
    frame.size < 5)
  {
    return iswv::Result<Feedback>::failure(iswv::ErrorCode::protocol_error,
      "BXI feedback requires a valid standard data frame of at least five bytes");
  }
  if (!valid_range(ranges.position) || !valid_range(ranges.velocity)) {
    return iswv::Result<Feedback>::failure(iswv::ErrorCode::invalid_argument,
      "BXI feedback requires finite ordered position and velocity encoding ranges");
  }
  const auto p = (static_cast<std::uint32_t>(frame.data[1]) << 8) | frame.data[2];
  const auto v = (static_cast<std::uint32_t>(frame.data[3]) << 4) | (frame.data[4] >> 4);
  return iswv::Result<Feedback>::success(
    Feedback{frame.data[0], decode(p, ranges.position, 16), decode(v, ranges.velocity, 12)});
}

Motor::Motor(iswv::ICanTransport & transport, std::uint32_t command_id, FrameOptions options)
: transport_(transport), command_id_(command_id), options_(options)
{
  if (command_id > 0x7ffU || (!options.fd && options.bitrate_switch)) {
    throw std::invalid_argument("BXI motor requires a standard CAN ID and valid frame options");
  }
}

iswv::Result<void> Motor::send(const std::array<std::uint8_t, 8> & payload)
{
  iswv::CanFrame frame{};
  frame.id = command_id_;
  frame.size = 8;
  frame.fd = options_.fd;
  frame.bitrate_switch = options_.bitrate_switch;
  std::copy(payload.begin(), payload.end(), frame.data.begin());
  return transport_.send(frame);
}

iswv::Result<void> Motor::enter_motor_mode()
{
  return send({0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc});
}

iswv::Result<void> Motor::exit_motor_mode()
{
  return send({0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd});
}

iswv::Result<void> Motor::command(const Command & command)
{
  auto payload = pack_command(command);
  if (!payload) {return iswv::Result<void>::failure(payload.error());}
  return send(payload.value());
}
}  // namespace bxi
