#pragma once

#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace chassis
{

class VelocityCommandWatchdog
{
public:
  using Clock = std::chrono::steady_clock;

  explicit VelocityCommandWatchdog(std::chrono::duration<double> timeout)
  : timeout_(timeout)
  {
    if (!std::isfinite(timeout.count()) || timeout.count() <= 0.0) {
      throw std::invalid_argument("command_timeout_sec must be positive and finite");
    }
  }

  void record_command(Clock::time_point now)
  {
    last_command_ = now;
    timeout_reported_ = false;
  }

  bool consume_timeout(Clock::time_point now)
  {
    if (!last_command_ || timeout_reported_) {return false;}
    if (std::chrono::duration<double>(now - *last_command_) < timeout_) {return false;}
    timeout_reported_ = true;
    return true;
  }

private:
  std::chrono::duration<double> timeout_;
  std::optional<Clock::time_point> last_command_;
  bool timeout_reported_{false};
};

}  // namespace chassis
