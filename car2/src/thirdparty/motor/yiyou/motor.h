#pragma once

#include "motor/yiyou/detail/drive.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace chassis
{

// 意优 PHU/RHU：EtherCAT CoE PV/PP/CST 驱动接口；调用方串行访问。
// enable/enable_position/enable_cst 选择对应模式，切换前先 stop()。
// CST 从站必须在通信层 Options::cst_slave 中选择，其他站位使用 PV/PP。
class YiyouMotor
{
public:
  YiyouMotor(ethercat::Master & master, std::uint16_t slave_position);
  explicit YiyouMotor(std::shared_ptr<ethercat::Node> node);
  ~YiyouMotor();
  YiyouMotor(const YiyouMotor &) = delete;
  YiyouMotor & operator=(const YiyouMotor &) = delete;

  // PV 速度为 pulse/s，加减速度为 pulse/s^2；初始目标速度为 0。
  void enable(std::uint32_t acceleration, std::uint32_t deceleration);
  // PP 的速度单位 pulse/s，加减速度单位 pulse/s^2；使能时保持当前位置。
  void enable_position(
    std::uint32_t velocity, std::uint32_t acceleration, std::uint32_t deceleration);
  // CST 循环同步电流模式；6071/6078 单位为额定电流千分比，启动目标为 0。
  void enable_cst();
  void set_current_permille(std::int16_t permille);
  void check_current_mode();
  std::int16_t actual_current_permille();
  // 绝对位置 pulse；等待 set-point acknowledge 握手，不等待运动完成。
  void move_to_position(std::int32_t position);
  // Runtime PP handshake, advanced by the owner without sleep/poll loops.
  // Stage every axis before triggering any axis; the owner enforces a deadline.
  void prepare_position_command(std::int32_t position);
  bool stage_position_command();
  void trigger_position_command();
  bool finish_position_command();
  // 最近一次握手成功的目标：bit10 置位且实际位置误差在主机容差内；无目标时返回 false。
  bool position_reached(std::uint32_t tolerance_pulses = 100);
  bool motion_stopped();
  void set_velocity(std::int32_t pulses_per_second);
  std::int32_t actual_velocity();
  std::int32_t actual_position();
  // 按当前模式停止/清零后失能；不操作共享电源。
  void stop();
  std::string diagnostic_report();

  // 未使能且静止时，以当前位置设零（HM 35）并保存全部可持久化参数。
  // 调用方必须在调用后（含异常）留出 EEPROM 保存时间，再断电；失败不自动重试。
  void save_zero_position(std::uint32_t tolerance_pulses = 100);

  // RPM 指减速器输出轴；使用 RPM API 前必须先读取电机的编码器/减速比标尺。
  double configure_rpm_units();
  std::int32_t velocity_from_rpm(double rpm) const;
  std::int32_t position_from_degrees(double degrees) const;
  void enable_rpm(double acceleration_rpm_per_s, double deceleration_rpm_per_s);
  void set_velocity_rpm(double rpm);
  double actual_velocity_rpm();

private:
  void select_ethercat_control();
  void disable_for_configuration();
  void wait_for_mode_display(const char * operation);
  void enable_mode(
    iswv::cia402::OperationMode mode, std::uint32_t velocity,
    std::uint32_t acceleration, std::uint32_t deceleration);
  std::uint16_t position_status();
  void wait_for_position_ack(bool acknowledged);
  std::shared_ptr<ethercat::Node> node_;
  ethercat::Drive drive_;
  bool active_{false};
  bool enabled_{false};
  std::optional<std::int32_t> position_target_;
  enum class PositionCommand {idle, clearing, staged, triggered, resetting};
  PositionCommand position_command_{PositionCommand::idle};
  std::int32_t pending_position_{0};
  iswv::cia402::OperationMode mode_{iswv::cia402::OperationMode::profile_velocity};
  double pulses_per_output_revolution_{0};
};

}  // namespace chassis
