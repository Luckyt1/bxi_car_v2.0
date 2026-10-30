#include "chassis/calibration_feedback.hpp"
#include "iswv/fake_transport.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>

namespace
{
using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;
using SampleResult = iswv::Result<chassis::CalibrationSample>;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

void require_error(const SampleResult & result, iswv::ErrorCode code, const char * message)
{
  require(!result && result.error().code == code, message);
}

template<typename T>
void put(iswv::CanFrame & frame, std::size_t offset, T value)
{
  const auto bytes = iswv::canopen::encode_little_endian(value);
  std::copy(bytes.begin(), bytes.end(), frame.data.begin() + offset);
}

iswv::CanFrame pdo(std::uint8_t number, Clock::time_point received_at, std::uint8_t node = 2)
{
  iswv::CanFrame frame;
  frame.id = 0x180U + 0x100U * (number - 1U) + node;
  frame.size = number < 3 ? 6 : 8;
  frame.received_at = received_at;
  if (number == 1) {
    put(frame, 0, std::uint16_t{0x4237});
    put(frame, 2, std::int32_t{-1234567});
  } else if (number == 2) {
    put(frame, 0, std::int32_t{-65432});
    put(frame, 4, std::int16_t{-123});
  }
  return frame;
}

struct Fixture
{
  // These outlive the master, including any callback already queued at teardown.
  std::mutex mutex;
  std::condition_variable condition;
  std::size_t markers{0};
  std::shared_ptr<iswv::FakeTransport> transport = std::make_shared<iswv::FakeTransport>();
  iswv::canopen::CanopenMaster master{transport};
  std::shared_ptr<iswv::canopen::CanopenNode> node = master.node(2);
  chassis::CalibrationFeedback feedback{node};
  iswv::Subscription barrier = master.on_raw_frame([this](const iswv::CanFrame & frame) {
    if (frame.id == 0x555) {
      std::lock_guard<std::mutex> lock(mutex);
      ++markers;
      condition.notify_one();
    }
  });

  void all_frames(Clock::time_point received_at)
  {
    for (std::uint8_t number = 1; number <= 3; ++number) {
      transport->inject(pdo(number, received_at));
    }
  }

  void flush()
  {
    std::unique_lock<std::mutex> lock(mutex);
    const auto expected = markers + 1;
    iswv::CanFrame marker;
    marker.id = 0x555;
    marker.size = 1;
    transport->inject(marker);
    require(condition.wait_for(lock, 1s, [&] {return markers >= expected;}),
      "CAN event queue did not reach the test marker");
    require(transport->sent_frames().empty(), "feedback collection must not send any CAN command");
  }
};

void valid_feedback_preserves_signed_values()
{
  Fixture fixture;
  const auto received_at = Clock::now() - 10ms;
  fixture.all_frames(received_at);
  fixture.flush();
  const auto result = fixture.feedback.snapshot(received_at);
  require(static_cast<bool>(result), "complete fresh feedback must be accepted");
  const auto & sample = result.value();
  require(sample.status_word == 0x4237 && sample.error_status == 0 && sample.error_status_2 == 0,
    "status and error fields must decode from the default mappings");
  require(sample.position_raw == -1234567 && sample.velocity_raw == -65432 && sample.current_raw == -123,
    "signed position, velocity and current must preserve their values");
  require(sample.position_received_at == received_at, "position must retain its own receive time");
  require(fixture.transport->sent_frames().empty(), "snapshot must not issue SDOs or other commands");
}

void every_pdo_is_required_and_other_nodes_cannot_fill_it()
{
  for (std::uint8_t missing = 1; missing <= 3; ++missing) {
    Fixture fixture;
    const auto now = Clock::now();
    for (std::uint8_t number = 1; number <= 3; ++number) {
      if (number != missing) {fixture.transport->inject(pdo(number, now));}
    }
    fixture.transport->inject(pdo(missing, now, 3));
    fixture.transport->inject(pdo(4, now));
    fixture.flush();
    require_error(fixture.feedback.snapshot(now, now), iswv::ErrorCode::timeout,
      "a missing PDO cannot be replaced by another node or PDO number");
  }
}

void every_pdo_expires_independently_despite_other_traffic()
{
  for (std::uint8_t stale = 1; stale <= 3; ++stale) {
    Fixture fixture;
    const auto now = Clock::now();
    for (std::uint8_t number = 1; number <= 3; ++number) {
      fixture.transport->inject(pdo(number, number == stale ? now - 100ms : now));
      fixture.transport->inject(pdo(number, now, 3));
    }
    iswv::CanFrame heartbeat;
    heartbeat.id = 0x702;
    heartbeat.size = 1;
    heartbeat.data[0] = 0x05;
    heartbeat.received_at = now;
    fixture.transport->inject(heartbeat);
    fixture.transport->inject(pdo(4, now));
    fixture.flush();
    require_error(fixture.feedback.snapshot(now - 1s, now), iswv::ErrorCode::timeout,
      "fresh heartbeat and other PDOs must not mask any expired feedback frame");
  }
}

void freshness_boundary_is_strict()
{
  Fixture fixture;
  const auto received_at = Clock::now();
  fixture.all_frames(received_at);
  fixture.flush();
  require(static_cast<bool>(fixture.feedback.snapshot(received_at, received_at + 100ms - 1ns)),
    "feedback just younger than 100 ms must remain valid");
  require_error(fixture.feedback.snapshot(received_at, received_at + 100ms), iswv::ErrorCode::timeout,
    "feedback exactly 100 ms old must expire");
}

void new_malformed_frames_replace_old_healthy_feedback()
{
  for (std::uint8_t number = 1; number <= 3; ++number) {
    const std::uint8_t expected = number == 3 ? 8 : 6;
    for (std::uint8_t size = 0; size <= 8; ++size) {
      if (size == expected) {continue;}
      Fixture fixture;
      const auto now = Clock::now();
      fixture.all_frames(now - 10ms);
      auto malformed = pdo(number, now - 5ms);
      malformed.size = size;
      fixture.transport->inject(malformed);
      fixture.flush();
      require_error(fixture.feedback.snapshot(now - 20ms, now), iswv::ErrorCode::protocol_error,
        "new malformed PDO must invalidate prior healthy feedback");
      fixture.transport->inject(pdo(number, now - 8ms));
      fixture.flush();
      require_error(fixture.feedback.snapshot(now - 20ms, now), iswv::ErrorCode::protocol_error,
        "an older healthy PDO must not hide the latest malformed frame");
      fixture.transport->inject(pdo(number, now));
      fixture.flush();
      require(static_cast<bool>(fixture.feedback.snapshot(now - 20ms, now)),
        "a newer correctly formed PDO can replace the malformed frame");
    }
  }
}

void fault_fields_are_returned_unchanged()
{
  Fixture fixture;
  const auto now = Clock::now();
  auto position = pdo(1, now);
  put(position, 0, std::uint16_t{0x0008});
  auto errors = pdo(3, now);
  put(errors, 4, std::uint16_t{0x1234});
  put(errors, 6, std::uint16_t{0x8000});
  fixture.transport->inject(position);
  fixture.transport->inject(pdo(2, now));
  fixture.transport->inject(errors);
  fixture.flush();
  const auto result = fixture.feedback.snapshot(now, now);
  require(result && result.value().status_word == 0x0008 && result.value().error_status == 0x1234 &&
    result.value().error_status_2 == 0x8000, "existing protection checks must receive all fault bits");
}

void all_frames_must_postdate_configuration()
{
  Fixture fixture;
  const auto not_before = Clock::now();
  fixture.all_frames(not_before - 1ms);
  fixture.flush();
  require_error(fixture.feedback.snapshot(not_before, not_before), iswv::ErrorCode::timeout,
    "feedback from before configuration must not authorize calibration");
  for (std::uint8_t number = 1; number <= 3; ++number) {
    fixture.transport->inject(pdo(number, not_before));
    fixture.flush();
    const auto result = fixture.feedback.snapshot(not_before, not_before);
    if (number < 3) {
      require_error(result, iswv::ErrorCode::timeout, "every PDO must cross the configuration boundary");
    } else {
      require(static_cast<bool>(result), "receive time equal to not_before must be accepted");
    }
  }
}

void old_mapping_format_waits_for_current_configuration()
{
  for (std::uint8_t old_mapping = 1; old_mapping <= 3; ++old_mapping) {
    Fixture fixture;
    const auto not_before = Clock::now();
    for (std::uint8_t number = 1; number <= 3; ++number) {
      if (number != old_mapping) {fixture.transport->inject(pdo(number, not_before));}
    }
    auto malformed = pdo(old_mapping, not_before - 1ms);
    malformed.size = 1;
    fixture.transport->inject(malformed);
    fixture.flush();
    require_error(fixture.feedback.snapshot(not_before, not_before), iswv::ErrorCode::timeout,
      "old mapping length must wait for post-configuration feedback");

    malformed.received_at = not_before;
    fixture.transport->inject(malformed);
    fixture.flush();
    require_error(fixture.feedback.snapshot(not_before, not_before), iswv::ErrorCode::protocol_error,
      "the same malformed length in the current configuration must fail immediately");

    fixture.transport->inject(pdo(old_mapping, not_before + 1ms));
    fixture.flush();
    require(static_cast<bool>(fixture.feedback.snapshot(not_before, not_before + 1ms)),
      "fresh default-mapped feedback must replace the previous mapping");
  }
}

void future_receive_times_are_rejected()
{
  for (std::uint8_t number = 1; number <= 3; ++number) {
    Fixture fixture;
    const auto now = Clock::now();
    fixture.all_frames(now);
    fixture.transport->inject(pdo(number, now + 1ns));
    fixture.flush();
    require_error(fixture.feedback.snapshot(now, now), iswv::ErrorCode::protocol_error,
      "each future-dated PDO must be rejected");
  }
}

void out_of_order_frames_cannot_replace_newer_samples()
{
  Fixture fixture;
  const auto now = Clock::now();
  fixture.all_frames(now);
  auto old_position = pdo(1, now - 1ms);
  put(old_position, 2, std::int32_t{7654321});
  auto old_current = pdo(2, now - 1ms);
  put(old_current, 4, std::int16_t{321});
  auto old_errors = pdo(3, now - 1ms);
  old_errors.size = 0;
  fixture.transport->inject(old_position);
  fixture.transport->inject(old_current);
  fixture.transport->inject(old_errors);
  fixture.flush();
  const auto result = fixture.feedback.snapshot(now, now);
  require(result && result.value().position_raw == -1234567 && result.value().current_raw == -123 &&
    result.value().position_received_at == now, "old frames must not roll back any cached sample");
}

void position_current_skew_is_bounded_in_both_directions()
{
  for (std::uint8_t earlier = 1; earlier <= 2; ++earlier) {
    for (const auto skew : {20000us, 20001us}) {
      Fixture fixture;
      const auto now = Clock::now();
      for (std::uint8_t number = 1; number <= 3; ++number) {
        fixture.transport->inject(pdo(number, number == earlier ? now - skew : now));
      }
      fixture.flush();
      const auto result = fixture.feedback.snapshot(now - 50ms, now);
      if (skew == 20ms) {
        require(static_cast<bool>(result), "20 ms position/current skew must be accepted");
      } else {
        require_error(result, iswv::ErrorCode::timeout, "position/current skew above 20 ms must fail");
      }
    }
  }
}
}  // namespace

int main()
{
  try {
    valid_feedback_preserves_signed_values();
    every_pdo_is_required_and_other_nodes_cannot_fill_it();
    every_pdo_expires_independently_despite_other_traffic();
    freshness_boundary_is_strict();
    new_malformed_frames_replace_old_healthy_feedback();
    fault_fields_are_returned_unchanged();
    all_frames_must_postdate_configuration();
    old_mapping_format_waits_for_current_configuration();
    future_receive_times_are_rejected();
    out_of_order_frames_cannot_replace_newer_samples();
    position_current_skew_is_bounded_in_both_directions();
    std::cout << "calibration feedback tests passed (11 cases)\n";
  } catch (const std::exception & error) {
    std::cerr << "calibration feedback: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
