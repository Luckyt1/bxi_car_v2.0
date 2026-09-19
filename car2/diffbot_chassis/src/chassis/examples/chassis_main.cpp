#include "chassis/control/chassis_control.h"
#include "chassis/control/arm_control.h"

#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
volatile std::sig_atomic_t running = 1;
void on_signal(int) {running = 0;}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "Usage: chassis_main steering.yaml [post_calibration_rpm=0.5]\n"
              << "Calibrate CAN0, center all steering axes, then run ISWV travel on CAN1/2.\n"
              << "CAN3 IDs 1/2/3 save startup zero and hold position 0 with Kp=200, Kd=4.\n"
              << "CAN3 absolute speed >=60 RPM cuts shared motor power and latches a fault.\n"
              << "Use RPM 0 to remain stopped after calibration; Ctrl-C stops all axes.\n";
    return 0;
  }
  if (argc < 2 || argc > 3) {
    std::cerr << "Usage: chassis_main steering.yaml [post_calibration_rpm=0.5]\n";
    return 1;
  }
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  try {
    auto config = chassis::load_steering_config(argv[1]);
    double rpm = 0.5;
    if (argc == 3) {
      std::size_t end = 0;
      rpm = std::stod(argv[2], &end);
      if (end != std::string(argv[2]).size()) {throw std::invalid_argument("invalid RPM");}
    }
    if (!std::isfinite(rpm) || rpm < 0.0 || rpm > config.drive_max_output_rpm) {
      throw std::invalid_argument("RPM must be in [0, drive_max_output_rpm]");
    }
    chassis::ChassisController chassis(config);
    auto keep_running = [] {return running != 0;};
    chassis.initialize(keep_running);
    // 声明在 chassis 后，退出时先停止 CAN3 电机，再由底盘断电。
    chassis::ArmController bxi_can3;
    bxi_can3.initialize();
    bxi_can3.save_zero_positions();  // ID 1/2/3 各保存一次当前位置为零点。
    const auto keep_running_with_hold = [&] {
        if (!running) {return false;}
        bxi_can3.update();  // 校准期间也刷新 p/v/t=0、Kp=200、Kd=4。
        const auto report = bxi_can3.take_feedback_diagnostic();
        if (!report.empty()) {std::cout << report << std::endl;}
        return true;
      };
    if (!keep_running_with_hold()) {throw std::runtime_error("startup cancelled");}
    chassis.calibrate(keep_running_with_hold);
    if (running && rpm > 0.0) {chassis.forward(rpm);}
    while (running) {
      // The application can call set_velocity(x, y, yaw) or stop() here for its next operation.
      bxi_can3.update();
      chassis.update();
      const auto report = bxi_can3.take_feedback_diagnostic();
      if (!report.empty()) {std::cout << report << std::endl;}
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    bxi_can3.stop();
    chassis.stop();
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
