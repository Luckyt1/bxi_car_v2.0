// 意优快速调用：链接 chassis_yiyou_motor。调用方先完成通信初始化、上电和等待。
// 创建：chassis::YiyouMotor motor(master, 1);  // EtherCAT 站位从 1 开始。
// master 比 motor 活得更久；串行访问 motor，调用方周期检查 master.check_health()。

#include "motor/yiyou/motor.h"

#include <chrono>
#include <thread>

void example_yiyou_control(chassis::YiyouMotor & motor)
{
  try {
    motor.configure_rpm_units();  // 读取编码器/减速比，建立输出轴 RPM 标尺。
    motor.enable_rpm(10.0, 10.0); // 输出轴加减速度，RPM/s；初始目标为零。
    motor.set_velocity_rpm(1.0);  // 输出轴目标速度，RPM。
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // 需要反馈时：motor.actual_velocity_rpm()、motor.actual_position()。
  } catch (...) {
    try {
      motor.stop();
    } catch (...) {
      // 保留原始异常；调用方仍需处理共享电源。
    }
    throw;
  }
  motor.stop();  // 按当前模式停止并失能；共享电源由调用方管理。
}

// 其他控制方式：按需替换上面 try 内的速度控制，切换前先 stop()。
// 位置控制：
//   motor.enable_position(1000, 2000, 2000); // pulse/s、pulse/s²、pulse/s²。
//   motor.move_to_position(target_pulses);  // 绝对位置 pulse，等待握手，不等待到位。
//   motor.position_reached(100);           // 周期检查到位；等待应由调用方设置超时。
// 电流控制：主站 Options::cst_slave 必须指定当前站位。
//   motor.enable_cst();
//   motor.set_current_permille(target);    // [-1000,1000]，额定电流千分比。
//   motor.actual_current_permille();       // 电流反馈；零电流不等于保持位置。
