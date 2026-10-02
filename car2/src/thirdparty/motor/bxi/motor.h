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
    bxi::ICanTransport & transport, std::uint32_t command_id,
    FrameOptions options = {});

  bxi::Result<void> enter_motor_mode(); ///< FF FF FF FF FF FF FF FC。
  bxi::Result<void> exit_motor_mode();  ///< FF FF FF FF FF FF FF FD。
  bxi::Result<void> save_zero_position(); ///< FF FF FF FF FF FF FF FE，保存当前位置为零点。

  /// 三种控制均须显式选择型号，使用该型号默认编码范围；未知型号抛出 std::invalid_argument。
  /// 速度 rad/s；kp=0，kd 为速度反馈增益，前馈力矩为零。
  bxi::Result<void> set_velocity(float velocity, float kd, Model model);
  /// 位置 rad；kp 为位置增益，kd 为阻尼，目标速度和前馈力矩为零。
  bxi::Result<void> set_position(
    float position, float kp, float kd, Model model);
  /// 力矩 Nm；kp=kd=0，仅设置前馈力矩。
  bxi::Result<void> set_torque(float torque, Model model);
  /// 完整 MIT 指令，支持设备自定义编码范围；所有控制接口只发送一帧，需调用方先使能。
  bxi::Result<void> command(const Command & command);

private:
  bxi::Result<void> send(const std::array<std::uint8_t, 8> & payload);

  bxi::ICanTransport & transport_;
  std::uint32_t command_id_;
  FrameOptions options_;
};

}  // namespace bxi
