#include "chassis/bxi_pci_transport.hpp"
#include "chassis/eyou_motor.hpp"

#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
volatile std::sig_atomic_t interrupted = 0;
void on_signal(int) {interrupted = 1;}
}  // namespace

int main(int argc, char ** argv)
{
  using namespace std::chrono_literals;
  if (argc > 3 || (argc == 2 && std::string(argv[1]) == "--help")) {
    std::cout << "Usage: eyou_motor_info [NODE_ID=1] [CAN_BUS=2]\n"
              << "Read one motor information snapshot on a BXI CAN bus.\n"
              << "Powers motors on, waits 4 s, powers off on exit. Exclusive use required.\n"
              << "Only SDO reads: no enable, motion, mode change or fault reset.\n";
    return argc > 3 ? 1 : 0;
  }
  try {
    const std::string value = argc >= 2 ? argv[1] : "1";
    std::size_t used = 0;
    const auto node_id = std::stoul(value, &used);
    if (used != value.size() || node_id < 1 || node_id > 127) {
      throw std::invalid_argument("NODE_ID must be 1..127");
    }
    const std::string bus_value = argc == 3 ? argv[2] : "2";
    used = 0;
    const auto bus = std::stoul(bus_value, &used);
    if (used != bus_value.size() || bus >= CANFD_DEVICE_NUM) {
      throw std::invalid_argument("CAN_BUS is outside the BXI device range");
    }
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    auto transport = std::make_shared<chassis::BxiPciTransport>(bus);
    iswv::canopen::CanopenMaster master(transport);
    chassis::EyouMotor motor(master, static_cast<std::uint8_t>(node_id));
    const auto power = transport->set_motor_power(true);
    if (!power) {throw std::runtime_error(power.error().message);}
    std::cout << "CAN" << bus << " node=" << node_id
              << ": waiting 4 s for power-up..." << std::endl;
    for (int i = 0; i < 40 && !interrupted; ++i) {
      std::this_thread::sleep_for(100ms);
    }
    if (interrupted) {return 130;}
    bool complete = false;
    std::cout << motor.information_report(&complete) << std::flush;
    bool electrical_complete = false;
    std::cout << motor.electrical_report(&electrical_complete) << '\n' << std::flush;
    complete = complete && electrical_complete;
    if (interrupted) {return 130;}
    if (!complete) {
      std::cerr << "Incomplete information; check READ FAILED / unavailable fields above.\n";
    }
    return complete ? 0 : 1;
  } catch (const std::exception & error) {
    std::cerr << "Eyou motor info: " << error.what() << '\n';
    return 1;
  }
}
