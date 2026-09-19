// SPDX-License-Identifier: Apache-2.0
// Adapted from bxirobotics/bxi_car chassis.cpp at a02e41f59373b36e38686ed8be83f3421f1959a4.
// Project adaptation: motor/ header layout and chassis CMake export.
// Changes: standalone API, explicit frame flags, input and feedback validation.
#pragma once

#include "iswv/transport.hpp"

#include <array>
#include <cstdint>

namespace bxi
{

struct Range
{
  float min;
  float max;
};

enum class Model
{
  BXI5014_19,
  BXI5018_19,
  BXI7010_19,
  BXI8515_19
};

/// 默认与50系列一致；其他型号用 encoding_ranges()，不是运行时运动限位。
struct EncodingRanges
{
  Range position{-12.5F, 12.5F};
  Range velocity{-45.0F, 45.0F};
  Range kp{0.0F, 500.0F};
  Range kd{0.0F, 5.0F};
  Range torque{-40.0F, 40.0F};
};

/// 官网 MIT 默认映射参数；设备寄存器已改动时，应再覆盖对应范围。
/// 未知枚举值抛出 std::invalid_argument，不发送或改写设备配置。
EncodingRanges encoding_ranges(Model model);

/// MIT 单位：position 为 rad，velocity 为 rad/s，torque 为 Nm。
/// velocity 不会从车轮线速度自动换算，也不是 ISWV/EYOU 的 RPM 接口。
struct Command
{
  float position{0.0F};  ///< 16 bit。
  float velocity{0.0F};  ///< 12 bit。
  float kp{0.0F};        ///< 12 bit。
  float kd{0.0F};        ///< 12 bit。
  float torque{0.0F};    ///< 前馈项，12 bit。
  EncodingRanges ranges{}; ///< 各参数的编码范围，需与电机协议一致。
};

struct Feedback
{
  std::uint8_t prefix{0}; ///< 原始 data[0]，官网协议定义为电机 can_id。
  float position{0.0F};  ///< data[1:3]，按 ranges.position 解码。
  float velocity{0.0F};  ///< data[3] 及 data[4] 高半字节，按 ranges.velocity 解码。
};

/// 合法参数沿用上游量化截断；范围须有限且 min < max。
/// 无效范围、越界、非有限值和 FA..FE 特殊控制帧碰撞返回 invalid_argument。
iswv::Result<std::array<std::uint8_t, 8>> pack_command(const Command & command);

/// 要求有效标准数据帧、至少 5 字节；无效输入返回 protocol_error。
/// 仅解析位置和速度，不改写接收回调。
/// 使用与命令相同的范围；只校验位置和速度范围，无效范围返回 invalid_argument。
/// 调用方按设备 master_id 路由；官网默认 can_id | 0x010，也允许单独修改。
iswv::Result<Feedback> decode_feedback(
  const iswv::CanFrame & frame, const EncodingRanges & ranges = {});

struct FrameOptions
{
  bool fd{true};
  bool bitrate_switch{true}; ///< 只有 fd=true 时才允许 BRS。
};

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
