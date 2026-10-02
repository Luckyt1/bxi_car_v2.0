// 底盘输入输出示例，可复制到自己的 .cpp；不提供 main，不加入默认编译。
// 链接 chassis_controller；安装后为 chassis::chassis_controller。
// chassis_inverse 只计算；其余控制函数复用调用方长期持有的 ChassisController。

#include "chassis/control.h"
#include "chassis/swerve_kinematics.hpp"

#include <array>
#include <functional>
#include <optional>
#include <string>

chassis::SwerveSolution chassis_inverse(
  double vx, double vy, double yaw,
  const std::array<chassis::ModuleKinematics, 4> & modules,
  const std::array<double, 4> & actual_angles,
  const chassis::SwerveOptions & options = {},
  std::optional<std::array<chassis::WheelTarget, 4>> previous = std::nullopt)
// 输入：vx/vy 为车体前向/左向速度（m/s），yaw 为偏航角速度（rad/s）。
//       modules 为轮子位置/角度范围/速度上限；actual_angles 为实际转角（rad）。
//       数组顺序统一为前左、后左、后右、前右；previous 可传上一轮解算目标。
// 输出：成功时 targets 给出各轮 angle_rad 和 speed_m_s；不是电机脉冲或 RPM。
//       if (!result) 时读取 status、wheel、reason；speed_scale 为统一限速比例。
// 本函数不发送指令，也不实施控制器的加减速或转向对齐策略。
{
  return chassis::solve_bounded_swerve(vx, vy, yaw, modules, actual_angles, options, previous);
}

chassis::SteeringConfigFile chassis_load_config(const std::string & path)
// 输入：底盘 YAML 配置路径。
// 输出：校验后的配置；路径、格式或参数无效时抛异常。此函数不连接硬件。
{
  return chassis::load_steering_config(path);
}

chassis::ChassisController::State chassis_start(
  chassis::ChassisController & controller, const std::function<bool()> & keep_running)
// 输入：尚未初始化的 controller，以及返回 false 时取消启动的 keep_running 回调。
// 输出：成功后返回 ready 状态；启动/校准失败或取消会抛异常。
// 会打开 CAN0/1/2、上电、搜索转向限位并回中；这是阻塞启动步骤，只执行一次。
{
  controller.initialize(keep_running);
  controller.calibrate(keep_running);
  return controller.state();
}

void chassis_set_velocity(
  chassis::ChassisController & controller, double vx, double vy, double yaw)
// 输入：已启动的 controller，前向/左向速度（m/s）和偏航角速度（rad/s）。
// 输出：无返回值；更新持续执行的速度目标，具体下发由周期 update 完成。
// 无效状态或数值会抛异常；停止发送新目标不会自动停车。
{
  controller.set_velocity(vx, vy, yaw);
}

chassis::ChassisController::State chassis_update(chassis::ChassisController & controller)
// 输入：已启动的 controller；由调用方串行周期调用，建议 20 ms 一次。
// 输出：执行本轮解算、指令输出和反馈检查后的状态；故障抛异常。
// 此状态不是实测车速或里程计；停车后仍需持续调用以监测反馈。
{
  controller.update();
  return controller.state();
}

void chassis_stop(chassis::ChassisController & controller)
// 输入：已启动的 controller。
// 输出：无返回值；下发零速并进入 stopped 状态，失败抛异常。
// 不等待实际静止，也不断电；继续 update 检查反馈，最终销毁控制器时清理电源。
{
  controller.stop();
}

// 调用片段：
// auto config = chassis_load_config("src/config/steering.yaml");
// chassis::ChassisController controller(config);
// const auto state = chassis_start(controller, keep_running);
// chassis_set_velocity(controller, 0.1, 0.0, 0.0);
// 在应用周期内调用 chassis_update(controller)，停车时调用 chassis_stop(controller)。
