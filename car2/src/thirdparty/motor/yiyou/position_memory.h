#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>

namespace chassis
{

// Preserves a bounded, unambiguous output-shaft turn count across power loss.
// Requires the shaft to remain stationary while unpowered. All public calls
// belong to one control thread; only durable file writes run in the background.
class PositionMemory
{
public:
  PositionMemory(
    std::string path, std::string interface, std::uint16_t slave,
    std::array<std::uint32_t, 4> identity, double scale);
  ~PositionMemory();
  PositionMemory(const PositionMemory &) = delete;
  PositionMemory & operator=(const PositionMemory &) = delete;

  static std::string default_path();
  std::int64_t restore(std::int32_t raw);
  std::int64_t logical_position(std::int32_t raw) const;
  std::int32_t raw_target(std::int64_t logical) const;

  // Persist the complete possible motion interval before triggering a target.
  // Cancellation must not discard this reservation or the last triggered goal.
  void queue_target(std::int32_t raw_actual, std::int64_t logical_target);
  bool poll();
  bool pending() const;
  // Immediately before the drive trigger. A direction reversal/restart after
  // halt must first confirm the axis is stopped, so replacing the old goal
  // cannot discard a still-executing deceleration interval on the next write.
  void mark_target(std::int64_t logical);

  // Only call while disabled and stationary, with no outstanding write.
  // Invalidate BEFORE changing the drive's zero; explicit reference restores it.
  void invalidate();
  void set_reference(std::int32_t raw, std::int64_t logical);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace chassis
