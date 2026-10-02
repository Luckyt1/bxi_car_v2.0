#pragma once

#include "motor/yiyou/motor.h"
#include "arm/kinematics.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace chassis
{

struct ArmMotionConfig
{
  arm::Configuration kinematics{};
  // q = direction * (encoder - zero_counts) * 2*pi / pulses_per_output_revolution.
  std::array<int, 3> motor_directions{1, 1, 1};
  std::array<std::int32_t, 3> zero_counts{0, 0, 0};
  double speed_rpm{0.6};
  double acceleration_rpm_s{1.0};
  bool home_on_start{true};
};
void validate_arm_motion_config(const ArmMotionConfig & motion);

// Read the same JSON geometry/configuration format as the MuJoCo simulation.
arm::Configuration load_arm_kinematics(const std::string & path);

// EtherCAT 前三轴 PP 平面机械臂，可选第四轴 CST 电流控制。调用方串行访问。
// 共享电源由主程序管理；不保存零点。
class ArmController
{
public:
  ArmController(
    std::shared_ptr<ethercat::Master> master, std::array<std::uint16_t, 3> positions,
    std::function<void()> power_off, std::optional<std::uint16_t> current_position = {},
    const ArmMotionConfig & motion = {});
  // Offline injection; each node represents one CoE slave, never a CAN transport.
  explicit ArmController(
    std::array<std::shared_ptr<ethercat::Node>, 3> nodes,
    std::function<void()> power_off = {}, std::shared_ptr<ethercat::Node> current_node = {},
    const ArmMotionConfig & motion = {});
  ~ArmController();
  ArmController(const ArmController &) = delete;
  ArmController & operator=(const ArmController &) = delete;

  // 前三轴以 PP 保持启动位置；第四轴直接使能 CST 零电流，不回零。
  void initialize();
  // 前三轴返回已标定 q=[0,0,0]；第四轴保持 CST 零电流，不回零。
  // 不设零或写 EEPROM。等待到位期间可取消。
  // 主程序在 initialize 后按 home_on_start 调用，完成前不开放遥控。
  void return_to_zero(
    const std::function<bool()> & keep_running = {},
    std::chrono::milliseconds timeout = std::chrono::minutes(5));
  // 周期检查反馈并推进三轴目标握手；无阻塞轮询，不自动重新使能。
  void update(
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now());
  std::string take_feedback_diagnostic(
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now());
  // 兼容原相对角度接口，只允许 0；任何非零目标均拒绝。
  void set_target_degrees(std::uint32_t id, double degrees);
  double target_degrees(std::uint32_t id) const;
  // Shoulder-frame metres and planar pitch radians, matching the simulation.
  // Validation is pure; set_target_pose only queues a fully validated group.
  arm::JointAngles validate_target_pose(const arm::Pose & pose) const;
  void set_target_pose(const arm::Pose & pose);
  arm::JointAngles actual_joints() const;
  arm::Pose actual_pose() const;
  bool position_command_pending() const noexcept;
  bool has_current_axis() const noexcept {return bool(current_motor_);}
  // 第四逻辑轴；有符号额定电流千分比，允许 [-1000, 1000]，启动为 0。
  // 输出角 <0° 禁止负电流，>1300° 禁止正电流；周期反馈也会清零越界方向电流。
  void set_current_permille(std::int64_t current);
  std::int16_t target_current_permille() const noexcept {return target_current_;}
  std::string fault_reason() const;
  // 第四轴先清零电流并失能，再停止前三轴；任一失败仍继续处理其余轴。
  void stop();

private:
  void stop_noexcept() noexcept;
  void latch_fault(const std::string & reason);
  bool current_direction_blocked(std::int64_t current) const;
  void advance_position_command(std::chrono::steady_clock::time_point now);
  std::array<std::int32_t, 3> position_targets(const arm::JointAngles & joints) const;
  std::shared_ptr<ethercat::Master> master_;
  std::array<YiyouMotor, 3> motors_;
  ArmMotionConfig motion_config_;
  arm::Kinematics kinematics_;
  std::array<double, 3> position_scales_{};
  arm::JointAngles accepted_joints_{};
  arm::JointAngles pending_joints_{};
  std::array<std::int32_t, 3> pending_positions_{};
  enum class PositionPhase {idle, queued, staging, triggering, acknowledging};
  PositionPhase position_phase_{PositionPhase::idle};
  std::size_t staging_axis_{0};
  int elbow_branch_{-1};
  std::chrono::steady_clock::time_point position_deadline_{};
  std::unique_ptr<YiyouMotor> current_motor_;
  std::function<void()> power_off_;
  std::array<std::int32_t, 3> hold_positions_ {};
  std::array<std::int32_t, 3> actual_positions_ {};
  std::array<double, 3> actual_rpm_ {};
  std::int16_t target_current_ {0};
  std::int16_t actual_current_ {0};
  std::int32_t current_axis_position_pulses_ {0};
  double current_axis_position_scale_ {0};
  double current_axis_rpm_ {0};
  bool initialized_ {false};
  std::string fault_reason_;
  std::chrono::steady_clock::time_point next_feedback_ {};
  std::chrono::steady_clock::time_point next_report_ {};
};

}  // namespace chassis
