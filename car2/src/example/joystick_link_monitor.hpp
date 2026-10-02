#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace chassis
{

class JoystickLinkMonitor
{
public:
  using Clock = std::chrono::steady_clock;
  using Probe = std::function<bool()>;

  struct Snapshot
  {
    bool healthy{false};
    std::uint64_t generation{0};
    std::string reason;
  };

  static constexpr auto kProbePeriod = std::chrono::milliseconds(100);
  // Live BLE reads reached 339 ms; leave margin for ordinary radio jitter.
  static constexpr auto kProbeTimeout = std::chrono::milliseconds(500);

  explicit JoystickLinkMonitor(std::string device_path);
  explicit JoystickLinkMonitor(Probe probe);
  ~JoystickLinkMonitor();

  JoystickLinkMonitor(const JoystickLinkMonitor &) = delete;
  JoystickLinkMonitor & operator=(const JoystickLinkMonitor &) = delete;

  Snapshot snapshot() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace chassis
