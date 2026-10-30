// SPDX-License-Identifier: Apache-2.0
// Adapted from bxirobotics/bxi_car chassis.cpp at a02e41f59373b36e38686ed8be83f3421f1959a4.
// Project adaptation: motor/ header layout and chassis CMake export.
// Changes: no ROS/Modbus, deterministic frames, parameter validation, bounded feedback parsing.
#include "motor/bxi/motor.h"

#include <stdexcept>

namespace bxi
{
Motor::Motor(iswv::ICanTransport & transport, std::uint32_t command_id, FrameOptions options)
: transport_(transport), command_id_(command_id), options_(options)
{
  if (command_id > 0x7ffU || (!options.fd && options.bitrate_switch)) {
    throw std::invalid_argument("BXI motor requires a standard CAN ID and valid frame options");
  }
}

iswv::Result<void> Motor::send(const std::array<std::uint8_t, 8> & payload)
{
  return send_payload(transport_, command_id_, options_, payload);
}

iswv::Result<void> Motor::enter_motor_mode()
{
  return send({0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc});
}

iswv::Result<void> Motor::exit_motor_mode()
{
  return send({0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd});
}

iswv::Result<void> Motor::save_zero_position()
{
  return send({0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe});
}

iswv::Result<void> Motor::command(const Command & command)
{
  auto payload = pack_command(command);
  if (!payload) {return iswv::Result<void>::failure(payload.error());}
  return send(payload.value());
}
}  // namespace bxi
