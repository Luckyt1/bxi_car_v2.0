// BXI 电机快速调用框架；链接 chassis_bxi_motor 和 chassis_bxi_transport。

#include "bxi/board.h"        // BxiPciTransport：CAN 通道、共享电源。
#include "motor/bxi/motor.h"  // Motor、Model：使能、位置/速度/力矩控制。

#include <chrono>    // 上电等待和控制周期的时间单位。
#include <iostream>  // 输出反馈和错误信息。
#include <thread>    // std::this_thread::sleep_for
int main()
{
  
chassis::BxiPciTransport can(0);  // CAN0，按实际通道填写
bxi::Motor motor(can, 1);        // CAN ID 为 1，按实际电机填写
// 此前需要完成电机上电和等待。
auto result = motor.enter_motor_mode();
motor.set_position(1.0f, 10.0f, 1.0f, bxi::Model::BXI5014_19);  // 位置控制
motor.set_velocity(1.0f, 1.0f, bxi::Model::BXI5014_19);      // 速度控制
motor.set_torque(1.0f, bxi::Model::BXI5014_19);                // 力矩控制
result = motor.exit_motor_mode();
}