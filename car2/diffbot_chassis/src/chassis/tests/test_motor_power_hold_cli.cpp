#include "chassis/bxi_pci_transport.hpp"

#include <csignal>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
std::vector<bool> power_commands;

void require(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}
}  // namespace

namespace chassis
{
BxiPciTransport::BxiPciTransport(unsigned int bus, bool trace)
: bus_(bus), trace_(trace)
{
  open_.store(true);
}

BxiPciTransport::~BxiPciTransport()
{
  open_.store(false);
}

iswv::Result<void> BxiPciTransport::send(const iswv::CanFrame &)
{
  return iswv::Result<void>::success();
}

void BxiPciTransport::set_receive_handler(iswv::ReceiveHandler handler)
{
  receive_handler_ = std::move(handler);
}

void BxiPciTransport::set_error_handler(iswv::TransportErrorHandler handler)
{
  error_handler_ = std::move(handler);
}

bool BxiPciTransport::is_open() const noexcept
{
  return open_.load();
}

std::string BxiPciTransport::name() const
{
  return "fake-bxi";
}

iswv::Result<void> BxiPciTransport::set_motor_power(bool enabled)
{
  power_commands.push_back(enabled);
  if (enabled) {
    std::raise(SIGTERM);
  }
  return iswv::Result<void>::success();
}
}  // namespace chassis

#define main motor_power_hold_main
#include "../src/motor_power_hold.cpp"
#undef main

int main()
{
  try {
    char program[] = "motor_power_hold";
    char * argv[] = {program};
    const int result = motor_power_hold_main(1, argv);
    require(result == 0, "signal shutdown must succeed");
    require(
      power_commands == std::vector<bool>({true, false}),
      "program must power on and then power off before exiting");
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
