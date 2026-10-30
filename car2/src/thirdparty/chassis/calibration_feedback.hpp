#pragma once

#include "motor/iswv/communication.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

namespace chassis
{

struct CalibrationSample
{
  std::uint16_t status_word;
  std::uint16_t error_status;
  std::uint16_t error_status_2;
  std::int32_t position_raw;
  std::int32_t velocity_raw;
  std::int16_t current_raw;
  std::chrono::steady_clock::time_point position_received_at;
};

// Reads the already configured default TPDO mappings without sending CAN
// commands. Each frame retains its own receive time; unrelated traffic cannot
// refresh its age, and an older frame cannot replace newer feedback.
class CalibrationFeedback
{
public:
  explicit CalibrationFeedback(std::shared_ptr<iswv::canopen::CanopenNode> node)
  : cache_(std::make_shared<Cache>())
  {
    if (!node) {throw std::invalid_argument("calibration feedback requires a CANopen node");}
    cache_->node_id = node->id();
    for (std::uint8_t number = 1; number <= 3; ++number) {
      subscriptions_[number - 1] = node->on_tpdo(
        number, [cache = cache_, number](const iswv::canopen::PdoEvent & event) {
          std::lock_guard<std::mutex> lock(cache->mutex);
          auto & frame = cache->frames[number - 1];
          if (!frame || event.received_at >= frame->received_at) {
            // Retain malformed new frames too: a prior healthy frame must not
            // hide a newer format error.
            frame = event;
          }
        });
    }
  }

  iswv::Result<CalibrationSample> snapshot(
    std::chrono::steady_clock::time_point not_before,
    std::optional<std::chrono::steady_clock::time_point> now = std::nullopt) const
  {
    using Result = iswv::Result<CalibrationSample>;
    using Milliseconds = std::chrono::milliseconds;
    std::lock_guard<std::mutex> lock(cache_->mutex);
    // Sample the clock under the same lock as the frames. A callback can publish
    // newer feedback while this caller is waiting to acquire the lock.
    const auto checked_at = now.value_or(std::chrono::steady_clock::now());
    for (std::size_t i = 0; i < cache_->frames.size(); ++i) {
      const auto & frame = cache_->frames[i];
      const auto label = "calibration TPDO" + std::to_string(i + 1);
      if (!frame) {
        return Result::failure(iswv::ErrorCode::timeout, label + " missing");
      }
      if (frame->received_at == std::chrono::steady_clock::time_point{} ||
        frame->received_at > checked_at)
      {
        return Result::failure(iswv::ErrorCode::protocol_error,
          label + " has an invalid receive time");
      }
      // Earlier frames may use the previous mapping. Wait for this configuration's
      // feedback before interpreting its format; invalid receive times still fail.
      if (frame->received_at < not_before) {
        return Result::failure(iswv::ErrorCode::timeout, label + " predates configuration");
      }
      if (frame->node_id != cache_->node_id || frame->number != i + 1 ||
        frame->size != (i == 2 ? 8 : 6))
      {
        return Result::failure(iswv::ErrorCode::protocol_error,
          label + " does not match the configured default mapping");
      }
      if (checked_at - frame->received_at >= Milliseconds{100}) {
        return Result::failure(iswv::ErrorCode::timeout, label + " is stale");
      }
    }
    const auto & position = *cache_->frames[0];
    const auto & current = *cache_->frames[1];
    const auto & errors = *cache_->frames[2];
    const auto skew = position.received_at >= current.received_at ?
      position.received_at - current.received_at : current.received_at - position.received_at;
    if (skew > Milliseconds{20}) {
      return Result::failure(iswv::ErrorCode::timeout,
        "calibration position/current TPDO receive times differ by more than 20 ms");
    }
    using iswv::canopen::decode_little_endian;
    return Result::success({
      decode_little_endian<std::uint16_t>(position.data.data(), 2).value(),
      decode_little_endian<std::uint16_t>(errors.data.data() + 4, 2).value(),
      decode_little_endian<std::uint16_t>(errors.data.data() + 6, 2).value(),
      decode_little_endian<std::int32_t>(position.data.data() + 2, 4).value(),
      decode_little_endian<std::int32_t>(current.data.data(), 4).value(),
      decode_little_endian<std::int16_t>(current.data.data() + 4, 2).value(),
      position.received_at});
  }

private:
  struct Cache
  {
    std::mutex mutex;
    std::uint8_t node_id{0};
    std::array<std::optional<iswv::canopen::PdoEvent>, 3> frames;
  };

  std::shared_ptr<Cache> cache_;
  std::array<iswv::Subscription, 3> subscriptions_;
};

}  // namespace chassis
