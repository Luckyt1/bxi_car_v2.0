#ifdef NDEBUG
#undef NDEBUG
#endif

#include "chassis/motor/bxi_can3.hpp"

#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{
constexpr double kPi = 3.14159265358979323846;

class RecordingTransport final : public iswv::ICanTransport
{
public:
  iswv::Result<void> send(const iswv::CanFrame & frame) override
  {
    attempts.push_back(frame);
    if (attempts.size() - 1 == fail_at) {
      if (throw_on_failure) {throw std::runtime_error("injected send exception");}
      return iswv::Result<void>::failure(iswv::ErrorCode::transport_error, failure_message);
    }
    return iswv::Result<void>::success();
  }
  void set_receive_handler(iswv::ReceiveHandler handler) override
  {
    std::lock_guard<std::mutex> lock(handler_mutex);
    receive_handler = std::move(handler);
  }
  void set_error_handler(iswv::TransportErrorHandler handler) override
  {
    std::lock_guard<std::mutex> lock(handler_mutex);
    error_handler = std::move(handler);
  }
  bool is_open() const noexcept override {return open;}
  std::string name() const override {return "offline CAN3";}

  iswv::ReceiveHandler receive_handler_copy() const
  {
    std::lock_guard<std::mutex> lock(handler_mutex);
    return receive_handler;
  }

  iswv::TransportErrorHandler error_handler_copy() const
  {
    std::lock_guard<std::mutex> lock(handler_mutex);
    return error_handler;
  }

  void inject(const iswv::CanFrame & frame) const
  {
    const auto handler = receive_handler_copy();
    if (handler) {handler(frame);}
  }

  void inject_error(iswv::Error error) const
  {
    const auto handler = error_handler_copy();
    if (handler) {handler(std::move(error));}
  }

  std::vector<iswv::CanFrame> attempts;
  std::size_t fail_at{std::numeric_limits<std::size_t>::max()};
  std::string failure_message{"injected failure"};
  bool throw_on_failure{false};
  bool open{true};

private:
  mutable std::mutex handler_mutex;
  iswv::ReceiveHandler receive_handler;
  iswv::TransportErrorHandler error_handler;
};

template<typename Exception, typename Function>
void expect_throw(Function action)
{
  bool caught = false;
  try {
    action();
  } catch (const Exception &) {
    caught = true;
  }
  assert(caught);
}

void assert_mode(const iswv::CanFrame & frame, std::uint32_t id, std::uint8_t mode)
{
  assert(frame.id == id && frame.size == 8);
  assert(frame.fd && frame.bitrate_switch && !frame.extended && !frame.remote);
  for (std::size_t i = 0; i < 7; ++i) {
    assert(frame.data[i] == 0xff);
  }
  assert(frame.data[7] == mode);
}

void assert_all_stopped(const RecordingTransport & transport, std::size_t offset)
{
  assert(transport.attempts.size() == offset + 3);
  for (std::uint32_t id = 1; id <= 3; ++id) {
    assert_mode(transport.attempts[offset + id - 1], id, 0xfd);
  }
}

void assert_zero_hold(const iswv::CanFrame & frame, std::uint32_t id)
{
  const bxi::Command command{
    0.0F, 0.0F, 200.0F, 4.0F, 0.0F, bxi::encoding_ranges(bxi::Model::BXI8515_19)};
  const auto expected = bxi::pack_command(command);
  assert(expected);
  assert(frame.id == id && frame.size == expected.value().size());
  assert(frame.fd && frame.bitrate_switch && !frame.extended && !frame.remote);
  for (std::size_t i = 0; i < expected.value().size(); ++i) {
    assert(frame.data[i] == expected.value()[i]);
  }
}

void assert_hold_target(const iswv::CanFrame & frame, std::uint32_t id, double degrees)
{
  const bxi::Command command{
    static_cast<float>(degrees * kPi / 180.0), 0.0F, 200.0F, 4.0F, 0.0F,
    bxi::encoding_ranges(bxi::Model::BXI8515_19)};
  const auto expected = bxi::pack_command(command);
  assert(expected);
  assert(frame.id == id && frame.size == expected.value().size());
  assert(frame.fd && frame.bitrate_switch && !frame.extended && !frame.remote);
  for (std::size_t i = 0; i < expected.value().size(); ++i) {
    assert(frame.data[i] == expected.value()[i]);
  }
}

bool contains(const std::string & text, const std::string & needle)
{
  return text.find(needle) != std::string::npos;
}

void assert_contains(const std::string & text, const std::string & needle)
{
  if (!contains(text, needle)) {
    std::cerr << "missing diagnostic substring: " << needle << "\nreport:\n" << text << "\n";
  }
  assert(contains(text, needle));
}

void assert_not_contains(const std::string & text, const std::string & needle)
{
  if (contains(text, needle)) {
    std::cerr << "unexpected diagnostic substring: " << needle << "\nreport:\n" << text << "\n";
  }
  assert(!contains(text, needle));
}

std::string motor_line(const std::string & report, std::uint32_t motor_id)
{
  const auto marker = "motor_id=" + std::to_string(motor_id);
  const auto marker_pos = report.find(marker);
  if (marker_pos == std::string::npos) {
    std::cerr << "missing motor line marker: " << marker << "\nreport:\n" << report << "\n";
    assert(false);
  }
  const auto line_start = report.rfind('\n', marker_pos);
  const auto start = line_start == std::string::npos ? 0 : line_start + 1;
  const auto line_end = report.find('\n', marker_pos);
  return report.substr(
    start, line_end == std::string::npos ? std::string::npos : line_end - start);
}

void assert_motor_state(
  const std::string & report, std::uint32_t motor_id, const std::string & state,
  std::uint64_t rx_count)
{
  const auto line = motor_line(report, motor_id);
  assert_contains(line, "motor_id=" + std::to_string(motor_id));
  assert_contains(line, "state=" + state);
  assert_contains(line, "rx_count=" + std::to_string(rx_count));
}

std::string diagnostic_float(float value)
{
  std::ostringstream out;
  out << std::fixed << std::setprecision(4) << value;
  return out.str();
}

void assert_motor_feedback(
  const std::string & report, std::uint32_t motor_id, const iswv::CanFrame & frame)
{
  const auto line = motor_line(report, motor_id);
  const auto decoded =
    bxi::decode_feedback(frame, bxi::encoding_ranges(bxi::Model::BXI8515_19));
  assert(decoded);
  assert_contains(line, "motor_id=" + std::to_string(motor_id));
  assert_contains(line, "reply_id=0x01" + std::to_string(motor_id));
  assert_contains(line, "state=REPLIED");
  assert_contains(line, "rx_count=1");
  assert_contains(line, "position_rad=" + diagnostic_float(decoded.value().position));
  assert_contains(line, "velocity_rad_s=" + diagnostic_float(decoded.value().velocity));
  assert_contains(line, "velocity_rpm=");
  assert_contains(line, "target_deg=");
}

iswv::CanFrame feedback_frame(
  std::uint32_t motor_id, std::uint16_t position_raw, std::uint16_t velocity_raw,
  std::chrono::steady_clock::time_point received_at = std::chrono::steady_clock::now())
{
  iswv::CanFrame frame{};
  frame.id = 0x010U + motor_id;
  frame.size = 8;
  frame.received_at = received_at;
  frame.data[0] = static_cast<std::uint8_t>(motor_id);
  frame.data[1] = static_cast<std::uint8_t>(position_raw >> 8);
  frame.data[2] = static_cast<std::uint8_t>(position_raw & 0xffU);
  frame.data[3] = static_cast<std::uint8_t>(velocity_raw >> 4);
  frame.data[4] = static_cast<std::uint8_t>((velocity_raw & 0x0fU) << 4);
  frame.data[5] = 0xab;
  frame.data[6] = 0xcd;
  frame.data[7] = 0xef;
  return frame;
}

void zero_hold_requires_saved_positions_then_refreshes_the_same_target()
{
  auto transport = std::make_shared<RecordingTransport>();
  chassis::BxiCan3 motors(transport);
  expect_throw<std::logic_error>([&] {motors.hold_zero();});
  assert(transport->attempts.empty());
  motors.initialize();
  expect_throw<std::logic_error>([&] {motors.hold_zero();});
  assert_all_stopped(*transport, 0);
  motors.save_zero_positions();
  motors.hold_zero();
  assert(transport->attempts.size() == 12);
  for (std::uint32_t id = 1; id <= 3; ++id) {
    assert_mode(transport->attempts[id - 1], id, 0xfd);
    assert_mode(transport->attempts[3 + id - 1], id, 0xfe);
    assert_mode(transport->attempts[6 + 2 * (id - 1)], id, 0xfc);
    assert_zero_hold(transport->attempts[7 + 2 * (id - 1)], id);
  }

  for (int refresh = 0; refresh < 2; ++refresh) {
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    const auto before = transport->attempts.size();
    motors.hold_zero();
    assert(transport->attempts.size() == before + 3);
    for (std::uint32_t id = 1; id <= 3; ++id) {
      assert_zero_hold(transport->attempts[before + id - 1], id);
    }
  }
}

void failed_zero_hold_start_stops_the_group_and_cannot_resume()
{
  for (std::uint32_t failed_id = 1; failed_id <= 3; ++failed_id) {
    for (const bool fail_command : {false, true}) {
      auto transport = std::make_shared<RecordingTransport>();
      chassis::BxiCan3 motors(transport);
      motors.initialize();
      motors.save_zero_positions();
      const auto before = transport->attempts.size();
      transport->fail_at = before + 2 * (failed_id - 1) + (fail_command ? 1 : 0);
      expect_throw<std::runtime_error>([&] {motors.hold_zero();});
      for (std::uint32_t id = 1; id <= failed_id; ++id) {
        assert_mode(transport->attempts[before + 2 * (id - 1)], id, 0xfc);
        if (id < failed_id || fail_command) {
          assert_zero_hold(transport->attempts[before + 2 * (id - 1) + 1], id);
        }
      }
      assert_all_stopped(*transport, transport->fail_at + 1);
      expect_throw<std::logic_error>([&] {motors.hold_zero();});
      assert_all_stopped(*transport, transport->fail_at + 1);
      motors.initialize();
      const auto after_reinitialize = transport->attempts.size();
      expect_throw<std::logic_error>([&] {motors.hold_zero();});
      assert(transport->attempts.size() == after_reinitialize);
    }
  }
}

void failed_zero_hold_refresh_stops_the_group_and_cannot_resume()
{
  for (std::uint32_t failed_id = 1; failed_id <= 3; ++failed_id) {
    auto transport = std::make_shared<RecordingTransport>();
    chassis::BxiCan3 motors(transport);
    motors.initialize();
    motors.save_zero_positions();
    motors.hold_zero();
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    const auto before = transport->attempts.size();
    transport->fail_at = before + failed_id - 1;
    expect_throw<std::runtime_error>([&] {motors.hold_zero();});
    for (std::uint32_t id = 1; id <= failed_id; ++id) {
      assert_zero_hold(transport->attempts[before + id - 1], id);
    }
    assert_all_stopped(*transport, before + failed_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    expect_throw<std::logic_error>([&] {motors.hold_zero();});
    assert_all_stopped(*transport, before + failed_id);
  }
}

void stopping_or_reinitializing_requires_an_explicit_new_zero_save()
{
  for (const bool reinitialize : {false, true}) {
    auto transport = std::make_shared<RecordingTransport>();
    chassis::BxiCan3 motors(transport);
    motors.initialize();
    motors.save_zero_positions();
    motors.hold_zero();
    const auto before = transport->attempts.size();
    if (reinitialize) {
      motors.initialize();
    } else {
      motors.stop();
    }
    assert_all_stopped(*transport, before);
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    expect_throw<std::logic_error>([&] {motors.hold_zero();});
    assert_all_stopped(*transport, before);
    motors.save_zero_positions();
    motors.hold_zero();
    assert(transport->attempts.size() == before + 12);
    for (std::uint32_t id = 1; id <= 3; ++id) {
      assert_mode(transport->attempts[before + 3 + id - 1], id, 0xfe);
      assert_mode(transport->attempts[before + 6 + 2 * (id - 1)], id, 0xfc);
      assert_zero_hold(transport->attempts[before + 7 + 2 * (id - 1)], id);
    }
  }
}

void initialization_and_commands_require_explicit_enable()
{
  auto transport = std::make_shared<RecordingTransport>();
  chassis::BxiCan3 motors(transport);
  assert(transport->attempts.empty());
  expect_throw<std::logic_error>([&] {motors.enable(1);});
  assert(transport->attempts.empty());

  motors.initialize();
  assert_all_stopped(*transport, 0);
  const bxi::Command target{1.0F, -2.0F, 3.0F, 0.4F, 5.0F};
  auto expected = target;
  expected.ranges = bxi::encoding_ranges(bxi::Model::BXI8515_19);
  const auto payload = bxi::pack_command(expected);
  assert(payload);
  for (std::uint32_t id = 1; id <= 3; ++id) {
    const auto before = transport->attempts.size();
    expect_throw<std::logic_error>([&] {motors.command(id, target);});
    assert(transport->attempts.size() == before);
    motors.enable(id);
    assert_mode(transport->attempts.back(), id, 0xfc);
    motors.command(id, target);
    assert(transport->attempts.size() == before + 2);
    const auto & frame = transport->attempts.back();
    assert(frame.id == id && frame.size == 8);
    for (std::size_t i = 0; i < payload.value().size(); ++i) {
      assert(frame.data[i] == payload.value()[i]);
    }
  }
  motors.stop();
  for (std::uint32_t id = 1; id <= 3; ++id) {
    expect_throw<std::logic_error>([&] {motors.command(id, target);});
  }
}

void zero_positions_are_saved_once_per_explicit_startup_call()
{
  auto transport = std::make_shared<RecordingTransport>();
  chassis::BxiCan3 motors(transport);
  expect_throw<std::logic_error>([&] {motors.save_zero_positions();});
  assert(transport->attempts.empty());
  motors.initialize();
  assert_all_stopped(*transport, 0);
  motors.save_zero_positions();
  assert(transport->attempts.size() == 6);
  for (std::uint32_t id = 1; id <= 3; ++id) {
    assert_mode(transport->attempts[id - 1], id, 0xfd);
    assert_mode(transport->attempts[3 + id - 1], id, 0xfe);
    expect_throw<std::logic_error>([&] {motors.command(id, {});});
  }
  assert(transport->attempts.size() == 6);

  for (std::uint32_t id = 1; id <= 3; ++id) {
    motors.enable(id);
    const auto before = transport->attempts.size();
    expect_throw<std::logic_error>([&] {motors.save_zero_positions();});
    assert(transport->attempts.size() == before);
    motors.stop();
    assert_all_stopped(*transport, before);
  }
  // Recovery initialization must not redefine the coordinate origin.
  const auto before = transport->attempts.size();
  motors.initialize();
  assert_all_stopped(*transport, before);
}

void zero_save_failure_stops_every_motor_without_saving_remaining_ids()
{
  for (std::uint32_t failed_id = 1; failed_id <= 3; ++failed_id) {
    auto transport = std::make_shared<RecordingTransport>();
    chassis::BxiCan3 motors(transport);
    motors.initialize();
    const auto before = transport->attempts.size();
    transport->fail_at = before + failed_id - 1;
    expect_throw<std::runtime_error>([&] {motors.save_zero_positions();});
    for (std::uint32_t id = 1; id <= failed_id; ++id) {
      assert_mode(transport->attempts[before + id - 1], id, 0xfe);
    }
    assert_all_stopped(*transport, before + failed_id);
    expect_throw<std::logic_error>([&] {motors.hold_zero();});
    expect_throw<std::logic_error>([&] {motors.save_zero_positions();});
    for (std::uint32_t id = 1; id <= 3; ++id) {
      expect_throw<std::logic_error>([&] {motors.enable(id);});
      expect_throw<std::logic_error>([&] {motors.command(id, {});});
    }
    assert_all_stopped(*transport, before + failed_id);
    motors.initialize();
    const auto after_reinitialize = transport->attempts.size();
    expect_throw<std::logic_error>([&] {motors.hold_zero();});
    assert(transport->attempts.size() == after_reinitialize);
  }
}

void all_three_motors_use_85_series_encoding()
{
  auto transport = std::make_shared<RecordingTransport>();
  chassis::BxiCan3 motors(transport);
  motors.initialize();
  // Intentionally retain Command's 50-series defaults; CAN3 owns the model selection.
  const bxi::Command target{0.0F, 0.0F, 0.0F, 10.0F, 80.0F};
  // MOTOR_85: Kd 10/20 -> 0x7ff; torque (80+160)/320 -> 0xbff.
  const std::array<std::uint8_t, 8> expected{0x7f, 0xff, 0x7f, 0xf0, 0x00, 0x7f, 0xfb, 0xff};
  for (std::uint32_t id = 1; id <= 3; ++id) {
    motors.enable(id);
    motors.command(id, target);
    const auto & frame = transport->attempts.back();
    assert(frame.id == id && frame.size == expected.size());
    for (std::size_t i = 0; i < expected.size(); ++i) {
      assert(frame.data[i] == expected[i]);
    }
  }
  assert(target.ranges.kd.max == 5.0F && target.ranges.torque.max == 40.0F);

  for (const auto & invalid : {
      bxi::Command{0.0F, 0.0F, 0.0F, 20.1F, 0.0F},
      bxi::Command{0.0F, 0.0F, 0.0F, 0.0F, 161.0F},
      bxi::Command{0.0F, 0.0F, 0.0F, 0.0F, -161.0F}})
  {
    motors.initialize();
    motors.enable(1);
    const auto before = transport->attempts.size();
    expect_throw<std::runtime_error>([&] {motors.command(1, invalid);});
    assert_all_stopped(*transport, before);
  }
}

void invalid_inputs_never_send_a_motion_command()
{
  expect_throw<std::invalid_argument>([] {chassis::BxiCan3 motors(nullptr);});
  auto transport = std::make_shared<RecordingTransport>();
  transport->open = false;
  expect_throw<std::invalid_argument>([&] {chassis::BxiCan3 motors(transport);});
  transport->open = true;
  chassis::BxiCan3 motors(transport);
  motors.initialize();
  for (const auto id : {0U, 4U, std::numeric_limits<std::uint32_t>::max()}) {
    expect_throw<std::invalid_argument>([&] {motors.enable(id);});
    expect_throw<std::invalid_argument>([&] {motors.command(id, {});});
  }
  assert_all_stopped(*transport, 0);
  motors.enable(1);
  bxi::Command invalid;
  invalid.velocity = std::numeric_limits<float>::quiet_NaN();
  const auto before = transport->attempts.size();
  expect_throw<std::runtime_error>([&] {motors.command(1, invalid);});
  assert_all_stopped(*transport, before);
  expect_throw<std::logic_error>([&] {motors.enable(1);});
}

void send_failure_disables_every_motor_and_requires_reinitialization()
{
  for (const bool fail_enable : {false, true}) {
    auto transport = std::make_shared<RecordingTransport>();
    chassis::BxiCan3 motors(transport);
    motors.initialize();
    motors.enable(1);
    motors.enable(3);
    if (!fail_enable) {
      motors.enable(2);
    }
    const auto before = transport->attempts.size();
    transport->fail_at = before;
    expect_throw<std::runtime_error>(
      [&] {
        if (fail_enable) {
          motors.enable(2);
        } else {
          motors.command(2, {});
        }
      });
    assert(transport->attempts[before].id == 2);
    assert_all_stopped(*transport, before + 1);
    for (std::uint32_t id = 1; id <= 3; ++id) {
      expect_throw<std::logic_error>([&] {motors.command(id, {});});
      expect_throw<std::logic_error>([&] {motors.enable(id);});
    }
    assert_all_stopped(*transport, before + 1);
    motors.initialize();
    motors.enable(2);
    motors.command(2, {});
    assert(transport->attempts.back().id == 2);
  }
}

void failed_stop_or_initialization_still_attempts_every_id()
{
  for (const bool fail_initialization : {false, true}) {
    auto transport = std::make_shared<RecordingTransport>();
    chassis::BxiCan3 motors(transport);
    if (!fail_initialization) {
      motors.initialize();
      for (std::uint32_t id = 1; id <= 3; ++id) {
        motors.enable(id);
      }
    }
    const auto before = transport->attempts.size();
    transport->fail_at = before + 1;
    expect_throw<std::runtime_error>(
      [&] {
        if (fail_initialization) {
          motors.initialize();
        } else {
          motors.stop();
        }
      });
    assert_all_stopped(*transport, before);
    for (std::uint32_t id = 1; id <= 3; ++id) {
      expect_throw<std::logic_error>([&] {motors.enable(id);});
      expect_throw<std::logic_error>([&] {motors.command(id, {});});
    }
    expect_throw<std::logic_error>([&] {motors.save_zero_positions();});
    expect_throw<std::logic_error>([&] {motors.hold_zero();});
    assert_all_stopped(*transport, before);
  }
}

void destructor_attempts_every_stop_even_after_a_send_failure()
{
  auto transport = std::make_shared<RecordingTransport>();
  std::size_t before = 0;
  {
    chassis::BxiCan3 motors(transport);
    motors.initialize();
    for (std::uint32_t id = 1; id <= 3; ++id) {
      motors.enable(id);
    }
    before = transport->attempts.size();
    transport->fail_at = before + 1;
  }
  assert_all_stopped(*transport, before);
}

void diagnostic_reports_waiting_then_no_reply_when_no_feedback_arrives()
{
  auto transport = std::make_shared<RecordingTransport>();
  chassis::BxiCan3 motors(transport);
  motors.initialize();
  const auto base = std::chrono::steady_clock::now();
  const auto first = motors.take_feedback_diagnostic(base);
  assert_contains(first, "motor_id=1");
  assert_contains(first, "motor_id=2");
  assert_contains(first, "motor_id=3");
  assert_contains(first, "state=WAITING");
  assert_contains(first, "rx_count=0");

  assert(motors.take_feedback_diagnostic(base + std::chrono::milliseconds(500)).empty());
  const auto after_timeout =
    motors.take_feedback_diagnostic(base + std::chrono::milliseconds(1100));
  assert_contains(after_timeout, "state=NO_REPLY");
  assert_contains(after_timeout, "unmatched_count=0");
  assert_contains(after_timeout, "invalid_count=0");
  assert_contains(after_timeout, "transport_error_count=0");
}

void diagnostic_reports_all_three_matching_replies()
{
  auto transport = std::make_shared<RecordingTransport>();
  chassis::BxiCan3 motors(transport);
  motors.initialize();
  const auto base = std::chrono::steady_clock::now();
  const auto motor1 = feedback_frame(1, 0x8000, 0x800, base);
  const auto motor2 =
    feedback_frame(2, 0x8001, 0x801, base + std::chrono::milliseconds(10));
  const auto motor3 =
    feedback_frame(3, 0x8002, 0x802, base + std::chrono::milliseconds(20));
  transport->inject(motor1);
  transport->inject(motor2);
  transport->inject(motor3);

  const auto report = motors.take_feedback_diagnostic(base + std::chrono::milliseconds(100));
  assert_motor_feedback(report, 1, motor1);
  assert_motor_feedback(report, 2, motor2);
  assert_motor_feedback(report, 3, motor3);
  assert_contains(motor_line(report, 1), "data=01 80 00 80 00 ab cd ef");
  assert_contains(motor_line(report, 2), "data=02 80 01 80 10 ab cd ef");
  assert_contains(motor_line(report, 3), "data=03 80 02 80 20 ab cd ef");
}

void diagnostic_reports_partial_replies_without_promoting_waiting_axes()
{
  auto transport = std::make_shared<RecordingTransport>();
  chassis::BxiCan3 motors(transport);
  motors.initialize();
  const auto base = std::chrono::steady_clock::now();
  transport->inject(feedback_frame(2, 0x8000, 0x800, base));

  const auto report = motors.take_feedback_diagnostic(base + std::chrono::milliseconds(100));
  assert_motor_state(report, 1, "WAITING", 0);
  assert_motor_state(report, 2, "REPLIED", 1);
  assert_contains(motor_line(report, 2), "reply_id=0x012");
  assert_motor_state(report, 3, "WAITING", 0);
}

void diagnostic_marks_replied_feedback_stale_then_replied_after_recovery()
{
  auto transport = std::make_shared<RecordingTransport>();
  chassis::BxiCan3 motors(transport);
  motors.initialize();
  const auto base = std::chrono::steady_clock::now();
  transport->inject(feedback_frame(1, 0x8000, 0x800, base));
  const auto fresh = motors.take_feedback_diagnostic(base + std::chrono::milliseconds(100));
  assert_contains(fresh, "motor_id=1");
  assert_contains(fresh, "state=REPLIED");

  const auto stale = motors.take_feedback_diagnostic(base + std::chrono::milliseconds(1200));
  assert_contains(stale, "motor_id=1");
  assert_contains(stale, "state=STALE");

  motors.initialize();
  const auto recovered_base = std::chrono::steady_clock::now();
  transport->inject(feedback_frame(1, 0x8001, 0x801, recovered_base));
  const auto recovered = motors.take_feedback_diagnostic(
    recovered_base + std::chrono::milliseconds(100));
  assert_contains(recovered, "motor_id=1");
  assert_contains(recovered, "state=REPLIED");
  assert_contains(recovered, "rx_count=1");
}

void diagnostic_recovers_from_stale_reply_without_reinitializing()
{
  auto transport = std::make_shared<RecordingTransport>();
  chassis::BxiCan3 motors(transport);
  motors.initialize();
  const auto base = std::chrono::steady_clock::now();
  transport->inject(feedback_frame(1, 0x8000, 0x800, base));
  const auto fresh = motors.take_feedback_diagnostic(base);
  assert_motor_state(fresh, 1, "REPLIED", 1);

  std::this_thread::sleep_for(std::chrono::milliseconds(1100));
  const auto stale_now = std::chrono::steady_clock::now();
  const auto stale = motors.take_feedback_diagnostic(stale_now);
  assert_motor_state(stale, 1, "STALE", 1);

  transport->inject(feedback_frame(1, 0x8001, 0x801, std::chrono::steady_clock::now()));
  const auto recovered = motors.take_feedback_diagnostic(std::chrono::steady_clock::now());
  assert_motor_state(recovered, 1, "REPLIED", 2);
  assert_contains(motor_line(recovered, 1), "data=01 80 01 80 10 ab cd ef");
}

void first_reply_is_reported_immediately_but_continuous_feedback_is_throttled()
{
  auto transport = std::make_shared<RecordingTransport>();
  chassis::BxiCan3 motors(transport);
  motors.initialize();
  const auto before = transport->attempts.size();
  assert_motor_state(motors.take_feedback_diagnostic(), 1, "WAITING", 0);
  transport->inject(feedback_frame(1, 0x8000, 0x800, std::chrono::steady_clock::now()));
  assert_motor_state(motors.take_feedback_diagnostic(), 1, "REPLIED", 1);
  transport->inject(feedback_frame(1, 0x8001, 0x801, std::chrono::steady_clock::now()));
  assert(motors.take_feedback_diagnostic().empty());
  transport->inject(feedback_frame(2, 0x8000, 0x800, std::chrono::steady_clock::now()));
  const auto second_motor = motors.take_feedback_diagnostic();
  assert_motor_state(second_motor, 1, "REPLIED", 2);
  assert_motor_state(second_motor, 2, "REPLIED", 1);
  assert_motor_state(second_motor, 3, "WAITING", 0);
  assert(transport->attempts.size() == before);
}

void diagnostic_rejects_short_prefix_mismatch_and_command_echo_frames()
{
  auto transport = std::make_shared<RecordingTransport>();
  chassis::BxiCan3 motors(transport);
  motors.initialize();
  const auto base = std::chrono::steady_clock::now();
  auto short_frame = feedback_frame(1, 0x8000, 0x800, base);
  short_frame.size = 7;
  transport->inject(short_frame);
  auto prefix_mismatch = feedback_frame(1, 0x8000, 0x800, base);
  prefix_mismatch.data[0] = 2;
  transport->inject(prefix_mismatch);
  auto command_echo = feedback_frame(1, 0x8000, 0x800, base);
  command_echo.id = 1;
  transport->inject(command_echo);
  iswv::CanFrame unknown_short{};
  unknown_short.id = 0x7f3;
  unknown_short.size = 2;
  unknown_short.data[0] = 0x0a;
  unknown_short.data[1] = 0x0d;
  transport->inject(unknown_short);

  const auto report = motors.take_feedback_diagnostic(base + std::chrono::milliseconds(100));
  assert_contains(report, "motor_id=1");
  assert_contains(report, "state=WAITING");
  assert_contains(report, "rx_count=0");
  assert_contains(report, "invalid_count=2");
  assert_contains(report, "unmatched_count=2");
  assert_contains(report, "id=0x7f3");
  assert_not_contains(report, "state=REPLIED");
}

void diagnostic_rejects_extended_remote_error_and_transport_error_as_replies()
{
  auto transport = std::make_shared<RecordingTransport>();
  chassis::BxiCan3 motors(transport);
  motors.initialize();
  const auto base = std::chrono::steady_clock::now();
  auto extended = feedback_frame(1, 0x8000, 0x800, base);
  extended.extended = true;
  transport->inject(extended);
  auto remote = feedback_frame(1, 0x8000, 0x800, base);
  remote.remote = true;
  transport->inject(remote);
  auto error = feedback_frame(1, 0x8000, 0x800, base);
  error.error = true;
  transport->inject(error);
  transport->inject_error(iswv::make_error(iswv::ErrorCode::transport_error, "rx bus off"));

  const auto report = motors.take_feedback_diagnostic(base + std::chrono::milliseconds(100));
  assert_contains(report, "state=WAITING");
  assert_contains(report, "rx_count=0");
  assert_contains(report, "invalid_count=3");
  assert_contains(report, "transport_error_count=1");
  assert_contains(report, "rx bus off");
}

void diagnostic_preserves_recent_unmatched_frame_summary()
{
  auto transport = std::make_shared<RecordingTransport>();
  chassis::BxiCan3 motors(transport);
  motors.initialize();
  const auto base = std::chrono::steady_clock::now();
  iswv::CanFrame unmatched{};
  unmatched.id = 0x123;
  unmatched.size = 8;
  unmatched.received_at = base;
  unmatched.data = {0xde, 0xad, 0xbe, 0xef, 0x00, 0x11, 0x22, 0x33};
  transport->inject(unmatched);

  const auto report = motors.take_feedback_diagnostic(base + std::chrono::milliseconds(100));
  assert_contains(report, "unmatched_count=1");
  assert_contains(report, "id=0x123");
  assert_contains(report, "data=de ad be ef 00 11 22 33");
}

void diagnostic_initialize_resets_receive_counts_and_feedback_state()
{
  auto transport = std::make_shared<RecordingTransport>();
  chassis::BxiCan3 motors(transport);
  motors.initialize();
  const auto base = std::chrono::steady_clock::now();
  transport->inject(feedback_frame(3, 0x8000, 0x800, base));
  const auto first = motors.take_feedback_diagnostic(base + std::chrono::milliseconds(100));
  assert_contains(first, "motor_id=3");
  assert_contains(first, "state=REPLIED");
  assert_contains(first, "rx_count=1");

  motors.initialize();
  const auto reset_base = std::chrono::steady_clock::now();
  const auto reset = motors.take_feedback_diagnostic(reset_base);
  assert_contains(reset, "motor_id=3");
  assert_contains(reset, "state=WAITING");
  assert_contains(reset, "rx_count=0");
  assert_contains(reset, "unmatched_count=0");
  assert_contains(reset, "invalid_count=0");
  assert_contains(reset, "transport_error_count=0");
}

void diagnostic_unregisters_handlers_on_destruction_and_copied_callback_is_safe()
{
  auto transport = std::make_shared<RecordingTransport>();
  iswv::ReceiveHandler copied_receive;
  iswv::TransportErrorHandler copied_error;
  const auto base = std::chrono::steady_clock::now();
  {
    chassis::BxiCan3 motors(transport);
    motors.initialize();
    copied_receive = transport->receive_handler_copy();
    copied_error = transport->error_handler_copy();
    assert(copied_receive);
    assert(copied_error);
  }
  assert(!transport->receive_handler_copy());
  assert(!transport->error_handler_copy());
  copied_receive(feedback_frame(1, 0x8000, 0x800, base));
  copied_error(iswv::make_error(iswv::ErrorCode::transport_error, "late copied callback"));

  chassis::BxiCan3 next(transport);
  next.initialize();
  const auto report = next.take_feedback_diagnostic(base + std::chrono::milliseconds(100));
  assert_contains(report, "state=WAITING");
  assert_contains(report, "rx_count=0");
  assert_contains(report, "transport_error_count=0");
}

void diagnostic_callback_and_report_are_safe_to_call_concurrently()
{
  auto transport = std::make_shared<RecordingTransport>();
  chassis::BxiCan3 motors(transport);
  motors.initialize();
  const auto base = std::chrono::steady_clock::now();
  std::atomic<bool> keep_running{true};
  std::thread receiver(
    [&] {
      for (std::uint16_t i = 0; i < 2000; ++i) {
        const auto id = static_cast<std::uint32_t>((i % 3U) + 1U);
        transport->inject(
          feedback_frame(
            id, static_cast<std::uint16_t>(0x7000U + i),
            static_cast<std::uint16_t>(0x800U + (i & 0x0fU)),
            base + std::chrono::milliseconds(i % 900)));
      }
      keep_running = false;
    });
  std::thread reporter(
    [&] {
      for (int i = 0; keep_running || i < 100; ++i) {
        (void)motors.take_feedback_diagnostic(base + std::chrono::milliseconds(1100 + i));
        if (!keep_running && i >= 100) {break;}
      }
    });
  receiver.join();
  reporter.join();

  motors.initialize();
  const auto usable_base = std::chrono::steady_clock::now();
  transport->inject(feedback_frame(1, 0x8000, 0x800, usable_base));
  const auto report = motors.take_feedback_diagnostic(usable_base + std::chrono::milliseconds(100));
  assert_contains(report, "motor_id=1");
  assert_contains(report, "state=REPLIED");
  assert_contains(report, "rx_count=1");
}

void overspeed_feedback_latches_fault_and_requests_power_off()
{
  for (const auto velocity_raw : {0x91eU, 0x6e1U}) {
    auto transport = std::make_shared<RecordingTransport>();
    int power_off_count = 0;
    chassis::BxiCan3 motors(transport, [&] {++power_off_count;});
    motors.initialize();
    const auto before = transport->attempts.size();
    transport->inject(feedback_frame(1, 0x8000, static_cast<std::uint16_t>(velocity_raw)));

    assert(power_off_count == 1);
    assert(transport->attempts.size() == before);
    assert_contains(motors.fault_reason(), "CAN3 motor 1 feedback speed");
    expect_throw<std::logic_error>([&] {motors.enable(1);});
    expect_throw<std::logic_error>([&] {motors.command(1, {});});

    transport->inject(feedback_frame(1, 0x8000, 0x800));
    assert(power_off_count == 1);
    assert_contains(motors.fault_reason(), "CAN3 motor 1 feedback speed");
  }
}

void speed_limit_uses_sixty_rpm_boundary_for_each_axis()
{
  for (std::uint32_t id = 1; id <= 3; ++id) {
    auto transport = std::make_shared<RecordingTransport>();
    int power_off_count = 0;
    chassis::BxiCan3 motors(transport, [&] {++power_off_count;});
    motors.initialize();

    transport->inject(feedback_frame(id, 0x8000, 0x91d));
    assert(power_off_count == 0);
    assert(motors.fault_reason().empty());
    transport->inject(feedback_frame(id, 0x8000, 0x91e));
    assert(power_off_count == 1);
    assert_contains(motors.fault_reason(), "CAN3 motor " + std::to_string(id));
  }
}

void invalid_feedback_never_triggers_speed_cutoff()
{
  auto transport = std::make_shared<RecordingTransport>();
  int power_off_count = 0;
  chassis::BxiCan3 motors(transport, [&] {++power_off_count;});
  motors.initialize();

  auto short_expected = feedback_frame(1, 0x8000, 0x91e);
  short_expected.size = 7;
  transport->inject(short_expected);
  auto prefix_mismatch = feedback_frame(1, 0x8000, 0x91e);
  prefix_mismatch.data[0] = 2;
  transport->inject(prefix_mismatch);
  auto unknown = feedback_frame(1, 0x8000, 0x91e);
  unknown.id = 0x7f3;
  transport->inject(unknown);

  assert(power_off_count == 0);
  assert(motors.fault_reason().empty());
}

void lost_feedback_after_first_hold_latches_fault()
{
  auto transport = std::make_shared<RecordingTransport>();
  int power_off_count = 0;
  chassis::BxiCan3 motors(transport, [&] {++power_off_count;});
  motors.initialize();
  motors.save_zero_positions();
  const auto base = std::chrono::steady_clock::now();
  motors.hold_zero(base);

  expect_throw<std::logic_error>([&] {motors.hold_zero(base + std::chrono::milliseconds(1001));});
  assert(power_off_count == 1);
  assert_contains(motors.fault_reason(), "feedback missing for more than 1s");
  expect_throw<std::logic_error>([&] {motors.initialize();});
}

void target_angle_requires_recent_feedback_and_waits_until_next_hold()
{
  auto transport = std::make_shared<RecordingTransport>();
  chassis::BxiCan3 motors(transport, [] {});
  motors.initialize();
  motors.save_zero_positions();
  const auto base = std::chrono::steady_clock::now();
  motors.hold_zero(base);

  expect_throw<std::logic_error>([&] {motors.set_target_degrees(2, 1.0);});
  transport->inject(feedback_frame(2, 0x8000, 0x800));
  const auto before = transport->attempts.size();
  motors.set_target_degrees(2, 1.0);
  assert(motors.target_degrees(2) == 1.0);
  assert(transport->attempts.size() == before);

  motors.hold_zero(base + std::chrono::milliseconds(25));
  assert(transport->attempts.size() == before + 3);
  assert_hold_target(transport->attempts[before], 1, 0.0);
  assert_hold_target(transport->attempts[before + 1], 2, 1.0);
  assert_hold_target(transport->attempts[before + 2], 3, 0.0);
}

void target_angle_rejects_bad_or_stale_requests()
{
  auto transport = std::make_shared<RecordingTransport>();
  chassis::BxiCan3 motors(transport, [] {});
  motors.initialize();
  motors.save_zero_positions();
  const auto base = std::chrono::steady_clock::now();
  motors.hold_zero(base);
  transport->inject(feedback_frame(1, 0x8000, 0x800));

  expect_throw<std::invalid_argument>(
    [&] {motors.set_target_degrees(1, std::numeric_limits<double>::quiet_NaN());});
  expect_throw<std::invalid_argument>([&] {motors.set_target_degrees(1, 1000.0);});
  std::this_thread::sleep_for(std::chrono::milliseconds(1100));
  expect_throw<std::logic_error>([&] {motors.set_target_degrees(1, 1.0);});
}

void power_off_failure_is_recorded_with_the_latched_fault()
{
  auto transport = std::make_shared<RecordingTransport>();
  chassis::BxiCan3 motors(
    transport, [] {throw std::runtime_error("cutoff relay command failed");});
  motors.initialize();
  transport->inject(feedback_frame(3, 0x8000, 0x91e));

  assert_contains(motors.fault_reason(), "CAN3 motor 3 feedback speed");
  assert_contains(motors.fault_reason(), "cutoff relay command failed");
  const auto report = motors.take_feedback_diagnostic();
  assert_contains(report, "CAN3_FAULT");
  assert_contains(report, "power_off_error=cutoff relay command failed");
}

void empty_send_errors_and_exceptions_still_disable_the_group()
{
  for (bool throw_error : {false, true}) {
    for (bool fail_command : {false, true}) {
      auto transport = std::make_shared<RecordingTransport>();
      chassis::BxiCan3 motors(transport);
      motors.initialize();
      motors.enable(2);
      transport->failure_message.clear();
      transport->throw_on_failure = throw_error;
      const auto before = transport->attempts.size();
      transport->fail_at = before;
      expect_throw<std::runtime_error>(
        [&] {
          if (fail_command) {motors.command(2, {});} else {motors.enable(1);}
        });
      assert_all_stopped(*transport, before + 1);
      expect_throw<std::logic_error>([&] {motors.enable(2);});
    }
  }
}

void overspeed_during_hold_blocks_all_motion_and_cannot_be_reset()
{
  auto transport = std::make_shared<RecordingTransport>();
  int cutoffs = 0;
  chassis::BxiCan3 motors(transport, [&] {++cutoffs;});
  motors.initialize();
  motors.save_zero_positions();
  motors.hold_zero();
  transport->inject(feedback_frame(2, 0x8000, 0x91e));
  transport->inject(feedback_frame(2, 0x8000, 0x800));
  const auto before = transport->attempts.size();
  expect_throw<std::logic_error>([&] {motors.initialize();});
  expect_throw<std::logic_error>([&] {motors.save_zero_positions();});
  expect_throw<std::logic_error>([&] {motors.hold_zero();});
  for (unsigned id = 1; id <= 3; ++id) {
    expect_throw<std::logic_error>([&] {motors.enable(id);});
    expect_throw<std::logic_error>([&] {motors.command(id, {});});
    expect_throw<std::logic_error>([&] {motors.set_target_degrees(id, 1.0);});
  }
  assert(transport->attempts.size() == before);
  assert(cutoffs == 1);
  motors.stop();
  expect_throw<std::logic_error>([&] {motors.initialize();});
}

void missing_one_axis_cuts_power_even_when_the_other_two_reply()
{
  for (unsigned missing = 1; missing <= 3; ++missing) {
    auto transport = std::make_shared<RecordingTransport>();
    int cutoffs = 0;
    chassis::BxiCan3 motors(transport, [&] {++cutoffs;});
    motors.initialize();
    motors.save_zero_positions();
    const auto base = std::chrono::steady_clock::now() - std::chrono::milliseconds(900);
    motors.hold_zero(base);
    for (unsigned id = 1; id <= 3; ++id) {
      if (id != missing) {transport->inject(feedback_frame(id, 0x8000, 0x800));}
    }
    const auto before = transport->attempts.size();
    expect_throw<std::logic_error>([&] {motors.hold_zero(base + std::chrono::seconds(1));});
    assert(cutoffs == 1);
    assert(transport->attempts.size() == before);
    assert_contains(motors.fault_reason(), "CAN3 motor " + std::to_string(missing));
  }
}

void copied_callback_after_destruction_cannot_cut_power()
{
  auto transport = std::make_shared<RecordingTransport>();
  iswv::ReceiveHandler callback;
  int cutoffs = 0;
  {
    chassis::BxiCan3 motors(transport, [&] {++cutoffs;});
    callback = transport->receive_handler_copy();
  }
  callback(feedback_frame(1, 0x8000, 0x91e));
  assert(cutoffs == 0);
}

}  // namespace

int main()
{
  zero_hold_requires_saved_positions_then_refreshes_the_same_target();
  failed_zero_hold_start_stops_the_group_and_cannot_resume();
  failed_zero_hold_refresh_stops_the_group_and_cannot_resume();
  stopping_or_reinitializing_requires_an_explicit_new_zero_save();
  zero_positions_are_saved_once_per_explicit_startup_call();
  zero_save_failure_stops_every_motor_without_saving_remaining_ids();
  all_three_motors_use_85_series_encoding();
  initialization_and_commands_require_explicit_enable();
  invalid_inputs_never_send_a_motion_command();
  send_failure_disables_every_motor_and_requires_reinitialization();
  failed_stop_or_initialization_still_attempts_every_id();
  destructor_attempts_every_stop_even_after_a_send_failure();
  diagnostic_reports_waiting_then_no_reply_when_no_feedback_arrives();
  diagnostic_reports_all_three_matching_replies();
  diagnostic_reports_partial_replies_without_promoting_waiting_axes();
  diagnostic_marks_replied_feedback_stale_then_replied_after_recovery();
  diagnostic_recovers_from_stale_reply_without_reinitializing();
  first_reply_is_reported_immediately_but_continuous_feedback_is_throttled();
  diagnostic_rejects_short_prefix_mismatch_and_command_echo_frames();
  diagnostic_rejects_extended_remote_error_and_transport_error_as_replies();
  diagnostic_preserves_recent_unmatched_frame_summary();
  diagnostic_initialize_resets_receive_counts_and_feedback_state();
  diagnostic_unregisters_handlers_on_destruction_and_copied_callback_is_safe();
  diagnostic_callback_and_report_are_safe_to_call_concurrently();
  overspeed_feedback_latches_fault_and_requests_power_off();
  speed_limit_uses_sixty_rpm_boundary_for_each_axis();
  invalid_feedback_never_triggers_speed_cutoff();
  lost_feedback_after_first_hold_latches_fault();
  target_angle_requires_recent_feedback_and_waits_until_next_hold();
  target_angle_rejects_bad_or_stale_requests();
  power_off_failure_is_recorded_with_the_latched_fault();
  empty_send_errors_and_exceptions_still_disable_the_group();
  overspeed_during_hold_blocks_all_motion_and_cannot_be_reset();
  missing_one_axis_cuts_power_even_when_the_other_two_reply();
  copied_callback_after_destruction_cannot_cut_power();
}
