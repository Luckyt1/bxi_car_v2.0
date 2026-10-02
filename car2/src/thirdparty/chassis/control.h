#pragma once

#include "chassis/steering_config.hpp"
#include "iswv/transport.hpp"
#include "iswv/iswv.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace chassis
{

iswv::AxisConfiguration steering_axis_configuration(
  const SteeringConfigFile & config, std::size_t index);

// CAN0 四个舵向轴：只负责按配置创建和索引 Axis，校准细节放在 cpp。
class SteeringAxes
{
public:
  SteeringAxes(iswv::canopen::CanopenMaster & can0, const SteeringConfigFile & config);

  std::shared_ptr<iswv::Axis> at(iswv::WheelPosition position) const
  {
    return axes_.at(static_cast<std::size_t>(position));
  }

private:
  std::array<std::shared_ptr<iswv::Axis>, 4> axes_{};
};

struct SteeringCalibrationResult
{
  std::array<std::int32_t, 4> midpoints{};
  std::array<std::int32_t, 4> negative_limits{};
  std::array<std::int32_t, 4> positive_limits{};
};

SteeringCalibrationResult calibrate_steering_with_limits(
  iswv::SteeringLayout & steering,
  SteeringConfigFile & config,
  const std::function<bool()> & keep_running);

std::array<std::int32_t, 4> calibrate_steering(
  iswv::SteeringLayout & steering,
  SteeringConfigFile & config,
  const std::function<bool()> & keep_running);

SteeringCalibrationResult calibrate_steering_with_limits(
  SteeringAxes & steering,
  SteeringConfigFile & config,
  const std::function<bool()> & keep_running);

std::array<std::int32_t, 4> calibrate_steering(
  SteeringAxes & steering,
  SteeringConfigFile & config,
  const std::function<bool()> & keep_running);

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
  // Calibrates all four steering axes, confirms midpoints, then enables travel axes.
  void calibrate(const std::function<bool()> & keep_running = [] {return true;});
  // Explicit continuous forward state, superseded by set_velocity() or stop().
  void forward(double output_rpm = 0.5);
  // SI units: m/s, m/s, rad/s. Holds the command until replaced, stopped or faulted.
  void set_velocity(double x, double y, double yaw);
  void update();
  // Sends zero without waiting for standstill; keep calling update() to monitor feedback.
  void stop();
  // Consumes the latest one-second zero-command feedback report; no CAN reads.
  std::string take_stop_diagnostic();
  State state() const noexcept;
  const SteeringConfigFile & config() const noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace chassis
