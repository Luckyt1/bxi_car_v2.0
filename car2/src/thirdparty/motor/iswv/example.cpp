// ISWV 快速调用：链接 chassis_iswv_motor。调用方先完成通信初始化、上电和等待。
// 创建：chassis::IswvDrive motor(master, iswv::AxisConfiguration::manual_travel(1));
// master 比 motor 活得更久；示例只控制已有电机，不管理共享电源。

#include "motor/iswv/motor.h"

#include <chrono>
#include <thread>

void example_iswv_control(chassis::IswvDrive & motor)
{
  try {
    motor.enable_rpm(10.0, 10.0);  // 输出轴加减速度，RPM/s；初始目标为零。
    motor.set_velocity_rpm(1.0);   // 输出轴目标速度，RPM。
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // 需要反馈时：const auto feedback = motor.actual_feedback_pdo();
    // 字段：output_rpm、position_inc、current_raw；缺失/过期/故障会抛异常。
  } catch (...) {
    try {
      motor.stop();
    } catch (...) {
      // 保留原始异常；调用方仍需处理共享电源。
    }
    throw;
  }
  motor.stop();  // 清零速度、快速停止、失能；失败抛异常。
}
