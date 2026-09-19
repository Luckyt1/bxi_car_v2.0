#include "chassis/motor/bxi_pci_transport.h"

#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
volatile std::sig_atomic_t stop_requested = 0;

void request_stop(int)
{
  stop_requested = 1;
}
}  // namespace

int main(int argc, char ** argv)
{
  using namespace std::chrono_literals;

  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout <<
      "Usage: motor_power_hold\n"
      "Keep the shared BXI motor power enabled until the process is asked to stop.\n"
      "This tool does not enable or command any motor. Run it exclusively.\n";
    return 0;
  }
  if (argc != 1) {
    std::cerr << "Usage: motor_power_hold\n";
    return 1;
  }

  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  std::signal(SIGHUP, request_stop);

  try {
    auto transport = std::make_shared<chassis::BxiPciTransport>(0);
    const auto power_on = transport->set_motor_power(true);
    if (!power_on) {
      throw std::runtime_error(power_on.error().message);
    }

    std::cout << "共享电机电源已开启；不会使能或驱动电机。\n";
    while (!stop_requested) {
      std::this_thread::sleep_for(200ms);
    }

    const auto power_off = transport->set_motor_power(false);
    if (!power_off) {
      throw std::runtime_error("failed to switch motor power off: " + power_off.error().message);
    }
    std::cout << "共享电机电源已关闭。\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "Motor power hold: " << error.what() << '\n';
    return 1;
  }
}
