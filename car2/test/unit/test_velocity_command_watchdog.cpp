#include "example/velocity_command_watchdog.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>

namespace
{
using namespace std::chrono_literals;
using Clock = chassis::VelocityCommandWatchdog::Clock;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

void no_initial_message_never_times_out()
{
  chassis::VelocityCommandWatchdog watchdog(300ms);
  const auto start = Clock::time_point{};
  require(!watchdog.consume_timeout(start), "no command must not time out immediately");
  require(!watchdog.consume_timeout(start + 10s), "no command must keep the chassis stopped");
}

void boundary_triggers_at_timeout()
{
  chassis::VelocityCommandWatchdog watchdog(300ms);
  const auto start = Clock::time_point{};
  watchdog.record_command(start);
  require(!watchdog.consume_timeout(start + 299ms), "299 ms must remain inside the timeout");
  require(watchdog.consume_timeout(start + 300ms), "300 ms must trigger timeout");
  require(!watchdog.consume_timeout(start + 301ms), "timeout must report only once");
}

void normal_fifty_millisecond_input_does_not_timeout()
{
  chassis::VelocityCommandWatchdog watchdog(300ms);
  auto now = Clock::time_point{};
  for (int i = 0; i < 40; ++i) {
    watchdog.record_command(now);
    require(!watchdog.consume_timeout(now + 250ms), "fresh 50 ms input must not time out");
    now += 50ms;
  }
}

void command_stream_loss_times_out_once()
{
  chassis::VelocityCommandWatchdog watchdog(300ms);
  const auto start = Clock::time_point{};
  watchdog.record_command(start);
  require(!watchdog.consume_timeout(start + 250ms), "active command must be held before timeout");
  require(watchdog.consume_timeout(start + 300ms), "lost command stream must time out");
  require(!watchdog.consume_timeout(start + 1s), "lost command stream must not spam timeout");
}

void new_command_recovers_after_timeout()
{
  chassis::VelocityCommandWatchdog watchdog(300ms);
  const auto start = Clock::time_point{};
  watchdog.record_command(start);
  require(watchdog.consume_timeout(start + 300ms), "initial stream must time out");
  watchdog.record_command(start + 350ms);
  require(!watchdog.consume_timeout(start + 600ms), "new command must refresh the deadline");
  require(watchdog.consume_timeout(start + 650ms), "refreshed stream must time out later");
}

void invalid_timeout_is_rejected()
{
  bool rejected = false;
  try {
    chassis::VelocityCommandWatchdog watchdog(0ms);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  require(rejected, "zero timeout must be rejected");
}
}  // namespace

int main()
{
  try {
    no_initial_message_never_times_out();
    boundary_triggers_at_timeout();
    normal_fifty_millisecond_input_does_not_timeout();
    command_stream_loss_times_out_once();
    new_command_recovers_after_timeout();
    invalid_timeout_is_rejected();
    std::cout << "velocity command watchdog tests passed\n";
  } catch (const std::exception & error) {
    std::cerr << "velocity command watchdog: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
