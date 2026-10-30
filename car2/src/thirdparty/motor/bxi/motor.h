// SPDX-License-Identifier: Apache-2.0
// Adapted from bxirobotics/bxi_car chassis.cpp at a02e41f59373b36e38686ed8be83f3421f1959a4.
// Project adaptation: motor/ header layout and chassis CMake export.
// Changes: standalone API, explicit frame flags, input and feedback validation.
#pragma once

#include "motor/bxi/communication.h"

namespace bxi
{

/// 单电机 MIT 命令接口；transport 必须比对象存活更久。
/// 构造/析构均不发帧、不控制共享电源，应用负责显式退出和电源清理。
/// 成功仅表示 transport 接受发送，未等待设备确认；没有自动使能或看门狗。
class Motor
{
public:
  /// command_id 是标准 CAN ID（0..0x7ff），不是 CANopen Node-ID。
  /// 默认 FD+BRS 沿用 motor_test.c；可传 {false,false} 选择经典 CAN。
  Motor(
    iswv::ICanTransport & transport, std::uint32_t command_id,
    FrameOptions options = {});

  iswv::Result<void> enter_motor_mode(); ///< FF FF FF FF FF FF FF FC。
  iswv::Result<void> exit_motor_mode();  ///< FF FF FF FF FF FF FF FD。
  iswv::Result<void> save_zero_position(); ///< FF FF FF FF FF FF FF FE，保存当前位置为零点。
  iswv::Result<void> command(const Command & command);

private:
  iswv::Result<void> send(const std::array<std::uint8_t, 8> & payload);

  iswv::ICanTransport & transport_;
  std::uint32_t command_id_;
  FrameOptions options_;
};

}  // namespace bxi
