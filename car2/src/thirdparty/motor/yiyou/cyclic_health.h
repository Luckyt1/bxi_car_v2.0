#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>

namespace chassis::ethercat
{
enum class HealthDecision {accept, retry, fault};

// Cyclic-thread-only debounce. The master owns the irreversible fault latch.
// A retry never makes an invalid process image safe to publish.
class CyclicHealth
{
public:
  static constexpr unsigned pdo_failure_limit = 3;
  static constexpr unsigned al_failure_limit = 3;
  static constexpr unsigned al_poll_cycles = 10;
  static constexpr auto feedback_deadline = std::chrono::milliseconds(20);

  HealthDecision observe_pdo(int actual_wkc, int expected_wkc)
  {
    if (actual_wkc == expected_wkc && expected_wkc > 0) {
      pdo_failures_ = 0;
      return HealthDecision::accept;
    }
    pdo_failures_ = std::min(pdo_failures_ + 1, pdo_failure_limit);
    return pdo_failures_ >= pdo_failure_limit ? HealthDecision::fault : HealthDecision::retry;
  }

  bool should_poll_al(unsigned cycle, std::uint16_t slave) const
  {
    return cycle % al_poll_cycles == 0 || al_failures_.at(slave) > 0;
  }

  HealthDecision observe_al(std::uint16_t slave, bool received, bool operational)
  {
    auto & failures = al_failures_.at(slave);
    if (received) {
      failures = 0;
      return operational ? HealthDecision::accept : HealthDecision::fault;
    }
    failures = std::min(failures + 1, al_failure_limit);
    return failures >= al_failure_limit ? HealthDecision::fault : HealthDecision::retry;
  }

  unsigned pdo_failures() const {return pdo_failures_;}
  unsigned al_failures(std::uint16_t slave) const {return al_failures_.at(slave);}

  static bool feedback_expired(std::chrono::steady_clock::duration age)
  {
    return age > feedback_deadline;
  }

  static bool can_publish(
    int actual_wkc, int expected_wkc, std::chrono::steady_clock::duration age, bool latched)
  {
    return !latched && actual_wkc == expected_wkc && expected_wkc > 0 && !feedback_expired(age);
  }

private:
  unsigned pdo_failures_{0};
  // validate_selection() restricts selected physical positions to 1..199.
  std::array<unsigned, 200> al_failures_{};
};
}  // namespace chassis::ethercat
