#pragma once

#include "chassis/motor/bxi_motor.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace chassis
{

// CAN3 上 ID 1/2/3 均为 BXI8515-19（85 系列）。串行调用；共享电源由主程序管理。
class BxiCan3
{
public:
  BxiCan3();
  // 注入 transport 用于离线测试；实际使用时必须传 CAN3。
  // power_off 在反馈锁内、RX 回调上下文同步调用；必须快速返回，不得重入 BxiCan3
  // 或构造/销毁 transport。
  explicit BxiCan3(
    std::shared_ptr<iswv::ICanTransport> transport,
    std::function<void()> power_off = {});
  ~BxiCan3();
  BxiCan3(const BxiCan3 &) = delete;
  BxiCan3 & operator=(const BxiCan3 &) = delete;

  // 主程序上电后调用：让 ID 1、2、3 全部退出电机模式，不自动运动。
  void initialize();
  // initialize() 后、全部失能时调用：给 ID 1/2/3 各发送一次保存当前位置为零点。
  // 主程序仅在启动时显式调用；initialize()、stop() 和析构均不自动保存零点。
  void save_zero_positions();
  // 三个保存零点指令均提交成功后使能并保持当前目标位：v/t=0，Kp=200、Kd=4。
  // 初始目标为 0；set_target_degrees() 只更新目标，下一次 hold_zero() 发送。
  // 主循环和校准回调反复调用；内部按 20 ms 节流，不重复使能或保存零点。
  // stop()/initialize()/发送失败后拒绝自动恢复；需要重新显式保存零点。
  void hold_zero(
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now());
  void enable(std::uint32_t id);
  // 必须先 enable(id)。内部固定使用 BXI8515_19 编码范围，覆盖传入的 command.ranges。
  // 单位 rad、rad/s、Nm；编码范围不代表运行时运动/力矩限制。
  // 每次调用发送一帧；需要持续控制时由主程序周期调用。
  void command(std::uint32_t id, const bxi::Command & command);
  // 每秒汇总 CAN3 接收状态；默认回复 ID 0x11/0x12/0x13，data[0] 须匹配电机 ID。
  // 首次/中断后恢复的有效回复会在下次调用立即输出，不等待下一次定期汇总。
  // REPLIED 仅确认收到普通 MIT 反馈，不确认使能/零点保存。
  // 首次 hold 后任一轴反馈丢失超过 1 秒会锁存故障并请求断电；锁存只能重建对象恢复。
  std::string take_feedback_diagnostic(
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now());
  void set_target_degrees(std::uint32_t id, double degrees);
  double target_degrees(std::uint32_t id) const;
  std::string fault_reason() const;
  // 尝试让全部三个电机退出模式；任一失败仍继续处理其余电机，然后抛出异常。
  void stop();

private:
  struct FeedbackState;
  void install_handlers();
  void stop_noexcept() noexcept;
  void throw_if_fault_latched() const;
  void latch_feedback_loss_if_needed(std::chrono::steady_clock::time_point now);
  static void latch_fault_locked(
    FeedbackState & state, std::uint32_t id, double velocity_rpm,
    const std::string & reason);
  std::shared_ptr<iswv::ICanTransport> transport_;
  std::shared_ptr<FeedbackState> feedback_state_;
  std::array<bxi::Motor, 3> motors_;
  std::array<bool, 3> enabled_{};
  bool initialized_{false};
  bool zero_save_commands_sent_{false};
  std::chrono::steady_clock::time_point next_zero_hold_command_{};
};

}  // namespace chassis
