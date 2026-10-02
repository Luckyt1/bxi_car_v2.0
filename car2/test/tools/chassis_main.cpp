#include "chassis/control.h"
#include "arm/control.h"
#include "bxi/board.h"

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
    std::cout << "Usage: chassis_main steering.yaml ETHERCAT_INTERFACE [post_calibration_rpm=0.5]\n"
              << "Calibrate CAN0, center all steering axes, then run ISWV travel on CAN1/2.\n"
              <<
      "EtherCAT Yiyou slave positions 1/2/3 hold their startup positions; angle control is disabled.\n"
              << "EtherCAT absolute speed >=60 RPM cuts shared motor power and latches a fault.\n"
              << "Use RPM 0 to remain stopped after calibration; Ctrl-C stops all axes.\n";
    return 0;
  }
  if (argc < 3 || argc > 4) {
    std::cerr <<
      "Usage: chassis_main steering.yaml ETHERCAT_INTERFACE [post_calibration_rpm=0.5]\n";
    return 1;
  }
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  try {
    chassis::ethercat::validate_selection({argv[2], {1, 2, 3}});
    auto config = chassis::load_steering_config(argv[1]);
    double rpm = 0.5;
    if (argc == 4) {
      std::size_t end = 0;
      rpm = std::stod(argv[3], &end);
      if (end != std::string(argv[3]).size()) {throw std::invalid_argument("invalid RPM");}
    }
    if (!std::isfinite(rpm) || rpm < 0.0 || rpm > config.drive_max_output_rpm) {
      throw std::invalid_argument("RPM must be in [0, drive_max_output_rpm]");
    }
    chassis::ChassisController chassis(config);
    auto keep_running = [] {return running != 0;};
    chassis.initialize(keep_running);
    // 声明在 chassis 后，退出时先停止 EtherCAT 电机，再由底盘断电。
    auto master = std::make_shared<chassis::ethercat::Master>(
      chassis::ethercat::Options{argv[2], {1, 2, 3}});
    auto power = std::make_shared<chassis::BxiPciTransport>(3);
    chassis::ArmController arm(master, {1, 2, 3}, [power] {
        const auto result = power->set_motor_power(false);
        if (!result) {throw std::runtime_error(result.error().message);}
      });
    arm.initialize();
    const auto keep_running_with_hold = [&] {
        if (!running) {return false;}
        arm.update();  // 校准期间也检查意优电机的原位保持状态。
        const auto report = arm.take_feedback_diagnostic();
        if (!report.empty()) {std::cout << report << std::endl;}
        return true;
      };
    if (!keep_running_with_hold()) {throw std::runtime_error("startup cancelled");}
    chassis.calibrate(keep_running_with_hold);
    if (running && rpm > 0.0) {chassis.forward(rpm);}
    while (running) {
      // The application can call set_velocity(x, y, yaw) or stop() here for its next operation.
      arm.update();
      chassis.update();
      const auto report = arm.take_feedback_diagnostic();
      if (!report.empty()) {std::cout << report << std::endl;}
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    arm.stop();
    chassis.stop();
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
