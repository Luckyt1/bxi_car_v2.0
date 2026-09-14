#include "chassis/chassis_controller.hpp"

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
    chassis.calibrate(keep_running);
    if (running && rpm > 0.0) {chassis.forward(rpm);}
    while (running) {
      // The application can call set_velocity(x, y, yaw) or stop() here for its next operation.
      chassis.update();
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    chassis.stop();
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
