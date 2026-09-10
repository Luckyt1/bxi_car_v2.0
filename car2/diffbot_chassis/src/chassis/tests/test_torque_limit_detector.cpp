#include "chassis/torque_limit_detector.hpp"

#include <chrono>
#include <limits>
#include <stdexcept>

namespace
{

void require(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

}  // namespace

int main()
{
  using namespace std::chrono_literals;
  // First sample can trigger, even with no prior position change or waiting.
  require(chassis::TorqueLimitDetector(0.2, 0.5).update(0.2), "positive threshold inclusive");
  require(chassis::TorqueLimitDetector(0.2, 0.5).update(-0.2), "negative threshold inclusive");
  chassis::TorqueLimitDetector detector(0.2, 0.5);
  require(!detector.update(0.0), "no load");
  require(!detector.update(0.199), "positive below threshold");
  require(!detector.update(-0.199), "negative below threshold");
  require(detector.update(0.3), "above contact and below safety");
  require(!detector.update(0.3, false), "contact ignored until detection is armed");
  require(!detector.update(0.1), "no latched state across samples");
  chassis::TorqueLimitDetector stalled_detector(0.2, 0.5, 5, 800ms);
  const auto start = std::chrono::steady_clock::now();
  require(!stalled_detector.update_contact(0.3, true, 0, start), "stall timer starts first");
  require(!stalled_detector.update_contact(0.3, true, 10, start + 700ms), "motion resets timer");
  require(
    !stalled_detector.update_contact(0.3, true, 12, start + 1400ms),
    "not stalled long enough");
  require(
    stalled_detector.update_contact(
      0.3, true, 13,
      start + 1600ms), "torque plus stall detects contact");
  chassis::TorqueLimitDetector creeping_detector(0.2, 0.5, 5, 800ms);
  for (int sample = 0; sample <= 12; ++sample) {
    require(
      !creeping_detector.update_contact(0.3, true, sample * 2, start + sample * 100ms),
      "small per-sample movements must accumulate instead of reporting a false limit");
  }
  for (double torque : {0.5, -0.5, 0.6, -0.6,
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
  {
    bool aborted = false;
    try {
      (void)detector.update(torque, false);
    } catch (const std::runtime_error &) {
      aborted = true;
    }
    require(aborted, "safety or invalid sample must abort, never record a limit");
  }
  for (double contact : {0.0, -0.1, 0.5, 0.6,
      std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity()})
  {
    bool rejected = false;
    try {
      (void)chassis::TorqueLimitDetector(contact, 0.5);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    require(rejected, "invalid thresholds rejected");
  }
  return 0;
}
