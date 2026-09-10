#pragma once

#include "chassis/steering_config.hpp"
#include "iswv/transport.hpp"

#include <chrono>
#include <functional>
#include <memory>

namespace chassis
{

// Single-owner API: serialize calls; call update() at least every 100 ms while running.
class ChassisController
{
public:
  enum class State {uninitialized, calibrating, ready, forward_wait, commanded, stopped, faulted};

  struct Hardware
  {
    std::function<std::shared_ptr<iswv::ICanTransport>(unsigned int)> open;
    std::function<void(bool)> power;
    std::chrono::milliseconds power_settle{4000};
  };

  explicit ChassisController(SteeringConfigFile config);
  ChassisController(SteeringConfigFile config, Hardware hardware);
  ~ChassisController();
  ChassisController(const ChassisController &) = delete;
  ChassisController & operator=(const ChassisController &) = delete;

  // Starts CAN0/1/2 and inhibits travel axes before any calibration movement.
  void initialize(const std::function<bool()> & keep_running = [] {return true;});
  // Calibrates steering axes in two pairs, confirms midpoints, then enables travel axes.
  void calibrate(const std::function<bool()> & keep_running = [] {return true;});
  // Explicit continuous forward state, superseded by set_velocity() or stop().
  void forward(double output_rpm = 0.5);
  // SI units: m/s, m/s, rad/s. Commands expire after command_timeout_s.
  void set_velocity(double x, double y, double yaw);
  void update();
  void stop();
  State state() const noexcept;
  const SteeringConfigFile & config() const noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace chassis
