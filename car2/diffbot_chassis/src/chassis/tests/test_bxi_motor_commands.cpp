#include "chassis/motor/bxi_motor.h"
#include "iswv/fake_transport.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

namespace
{

int source_float_to_uint(float x, float x_min, float x_max, int bits)
{
  const float span = x_max - x_min;
  const float offset = x_min;
  return static_cast<int>((x - offset) * static_cast<float>((1 << bits) - 1) / span);
}

float source_uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  const float span = x_max - x_min;
  const float offset = x_min;
  return static_cast<float>(x_int) * span / static_cast<float>((1 << bits) - 1) + offset;
}

std::array<std::uint8_t, 8> source_pack(const bxi::Command & command)
{
  const auto p_int = static_cast<std::uint16_t>(
    source_float_to_uint(command.position, -12.5f, 12.5f, 16));
  const auto v_int = static_cast<std::uint16_t>(
    source_float_to_uint(command.velocity, -45.0f, 45.0f, 12));
  const auto kp_int = static_cast<std::uint16_t>(
    source_float_to_uint(command.kp, 0.0f, 500.0f, 12));
  const auto kd_int = static_cast<std::uint16_t>(
    source_float_to_uint(command.kd, 0.0f, 5.0f, 12));
  const auto t_int = static_cast<std::uint16_t>(
    source_float_to_uint(command.torque, -40.0f, 40.0f, 12));

  return {
    static_cast<std::uint8_t>(p_int >> 8),
    static_cast<std::uint8_t>(p_int & 0xff),
    static_cast<std::uint8_t>(v_int >> 4),
    static_cast<std::uint8_t>(((v_int & 0xf) << 4) | (kp_int >> 8)),
    static_cast<std::uint8_t>(kp_int & 0xff),
    static_cast<std::uint8_t>(kd_int >> 4),
    static_cast<std::uint8_t>(((kd_int & 0xf) << 4) | (t_int >> 8)),
    static_cast<std::uint8_t>(t_int & 0xff),
  };
}

bool same_bytes(
  const std::array<std::uint8_t, 8> & left,
  const std::array<std::uint8_t, 8> & right)
{
  for (std::size_t i = 0; i < left.size(); ++i) {
    if (left[i] != right[i]) {
      return false;
    }
  }
  return true;
}

void assert_invalid_argument(const iswv::Result<std::array<std::uint8_t, 8>> & result)
{
  assert(!result);
  assert(result.error().code == iswv::ErrorCode::invalid_argument);
}

void assert_invalid_feedback(const iswv::Result<bxi::Feedback> & result)
{
  assert(!result);
  assert(result.error().code == iswv::ErrorCode::protocol_error);
}

void assert_invalid_argument(const iswv::Result<void> & result)
{
  assert(!result);
  assert(result.error().code == iswv::ErrorCode::invalid_argument);
}

void packs_minimum_command_like_source_pack_cmd()
{
  const bxi::Command command{-12.5f, -45.0f, 0.0f, 0.0f, -40.0f};
  const auto packed = bxi::pack_command(command);

  assert(packed);
  assert(same_bytes(packed.value(), source_pack(command)));
}

void packs_zero_command_like_source_pack_cmd()
{
  const bxi::Command command{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  const auto packed = bxi::pack_command(command);

  assert(packed);
  assert(same_bytes(packed.value(), source_pack(command)));
}

void packs_maximum_command_like_source_pack_cmd()
{
  const bxi::Command command{12.5f, 45.0f, 500.0f, 5.0f, 40.0f};
  const auto packed = bxi::pack_command(command);

  assert(packed);
  assert(same_bytes(packed.value(), source_pack(command)));
}

void packs_representative_commands_like_source_pack_cmd()
{
  const std::vector<bxi::Command> commands{
    {-3.25f, 12.75f, 37.5f, 0.25f, -8.0f},
    {4.5f, -18.25f, 128.0f, 2.5f, 7.5f},
    {11.0f, 0.125f, 499.0f, 4.75f, 33.0f},
    {-11.75f, 44.5f, 250.0f, 1.0f, -39.5f},
  };

  for (const auto & command : commands) {
    const auto packed = bxi::pack_command(command);
    assert(packed);
    assert(same_bytes(packed.value(), source_pack(command)));
  }
}

void packs_random_legal_commands_like_source_pack_cmd()
{
  std::mt19937 generator(0x425849U);
  std::uniform_real_distribution<float> position(-12.0f, 12.0f);
  std::uniform_real_distribution<float> velocity(-44.0f, 44.0f);
  std::uniform_real_distribution<float> kp(0.0f, 490.0f);
  std::uniform_real_distribution<float> kd(0.0f, 4.9f);
  std::uniform_real_distribution<float> torque(-39.0f, 39.0f);

  for (int i = 0; i < 64; ++i) {
    const bxi::Command command{
      position(generator), velocity(generator), kp(generator), kd(generator), torque(generator)};
    const auto packed = bxi::pack_command(command);
    assert(packed);
    assert(same_bytes(packed.value(), source_pack(command)));
  }
}

void rejects_gain_outside_source_range()
{
  const bxi::Command command{0.0f, 0.0f, 0.0f, 6.0f, 0.0f};

  assert_invalid_argument(bxi::pack_command(command));
}

void rejects_nan_command_values()
{
  const bxi::Command command{std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f, 0.0f, 0.0f};

  assert_invalid_argument(bxi::pack_command(command));
}

void rejects_infinite_command_values()
{
  const bxi::Command command{0.0f, std::numeric_limits<float>::infinity(), 0.0f, 0.0f, 0.0f};

  assert_invalid_argument(bxi::pack_command(command));
}

void rejects_payload_that_collides_with_enter_motor_mode_control_frame()
{
  const auto torque_for_fffc = source_uint_to_float(0xffc, -40.0f, 40.0f, 12);
  const bxi::Command command{12.5f, 45.0f, 500.0f, 5.0f, torque_for_fffc};

  assert(same_bytes(source_pack(command), {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc}));
  assert_invalid_argument(bxi::pack_command(command));
}

void rejects_payload_that_collides_with_exit_motor_mode_control_frame()
{
  const auto torque_for_fffd = source_uint_to_float(0xffd, -40.0f, 40.0f, 12);
  const bxi::Command command{12.5f, 45.0f, 500.0f, 5.0f, torque_for_fffd};

  assert(same_bytes(source_pack(command), {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd}));
  assert_invalid_argument(bxi::pack_command(command));
}

iswv::CanFrame valid_feedback_frame()
{
  iswv::CanFrame frame{};
  frame.id = 1;
  frame.fd = true;
  frame.bitrate_switch = true;
  frame.size = 5;
  return frame;
}

void decodes_minimum_feedback_endpoint_like_source_feedback()
{
  auto frame = valid_feedback_frame();
  frame.data[0] = 0xa5;
  frame.data[1] = 0x00;
  frame.data[2] = 0x00;
  frame.data[3] = 0x00;
  frame.data[4] = 0x00;

  const auto feedback = bxi::decode_feedback(frame);

  assert(feedback);
  assert(feedback.value().prefix == 0xa5);
  assert(std::fabs(feedback.value().position - -12.5f) < 0.0001f);
  assert(std::fabs(feedback.value().velocity - -45.0f) < 0.0001f);
}

void decodes_maximum_feedback_endpoint_like_source_feedback()
{
  auto frame = valid_feedback_frame();
  frame.data[0] = 0x5a;
  frame.data[1] = 0xff;
  frame.data[2] = 0xff;
  frame.data[3] = 0xff;
  frame.data[4] = 0xf0;

  const auto feedback = bxi::decode_feedback(frame);

  assert(feedback);
  assert(feedback.value().prefix == 0x5a);
  assert(std::fabs(feedback.value().position - 12.5f) < 0.0001f);
  assert(std::fabs(feedback.value().velocity - 45.0f) < 0.0001f);
}

void rejects_short_feedback_frame()
{
  auto frame = valid_feedback_frame();
  frame.size = 4;

  assert_invalid_feedback(bxi::decode_feedback(frame));
}

void rejects_extended_feedback_frame()
{
  auto frame = valid_feedback_frame();
  frame.extended = true;

  assert_invalid_feedback(bxi::decode_feedback(frame));
}

void rejects_remote_feedback_frame()
{
  auto frame = valid_feedback_frame();
  frame.remote = true;

  assert_invalid_feedback(bxi::decode_feedback(frame));
}

void rejects_error_feedback_frame()
{
  auto frame = valid_feedback_frame();
  frame.error = true;

  assert_invalid_feedback(bxi::decode_feedback(frame));
}

void sends_enter_motor_mode_control_frame()
{
  iswv::FakeTransport transport;
  bxi::Motor motor(transport, 1);

  const auto result = motor.enter_motor_mode();

  assert(result);
  const auto frames = transport.sent_frames();
  assert(frames.size() == 1);
  assert(frames.front().id == 1);
  assert(frames.front().size == 8);
  assert(frames.front().fd);
  assert(frames.front().bitrate_switch);
  const std::array<std::uint8_t, 8> expected{0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
  for (std::size_t i = 0; i < expected.size(); ++i) {
    assert(frames.front().data[i] == expected[i]);
  }
}

void sends_exit_motor_mode_control_frame()
{
  iswv::FakeTransport transport;
  bxi::Motor motor(transport, 1);

  const auto result = motor.exit_motor_mode();

  assert(result);
  const auto frames = transport.sent_frames();
  assert(frames.size() == 1);
  assert(frames.front().id == 1);
  assert(frames.front().size == 8);
  assert(frames.front().fd);
  assert(frames.front().bitrate_switch);
  const std::array<std::uint8_t, 8> expected{0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
  for (std::size_t i = 0; i < expected.size(); ++i) {
    assert(frames.front().data[i] == expected[i]);
  }
}

void command_sends_encoded_payload_to_motor_id()
{
  iswv::FakeTransport transport;
  bxi::Motor motor(transport, 2);
  const bxi::Command command{1.5f, -2.5f, 25.0f, 0.5f, 3.0f};

  const auto result = motor.command(command);

  assert(result);
  const auto frames = transport.sent_frames();
  assert(frames.size() == 1);
  assert(frames.front().id == 2);
  assert(frames.front().size == 8);
  assert(frames.front().fd);
  assert(frames.front().bitrate_switch);
  const auto expected = source_pack(command);
  for (std::size_t i = 0; i < expected.size(); ++i) {
    assert(frames.front().data[i] == expected[i]);
  }
}

void command_rejects_invalid_payload_without_sending()
{
  iswv::FakeTransport transport;
  bxi::Motor motor(transport, 1);

  assert_invalid_argument(motor.command({0.0f, 0.0f, 0.0f, 6.0f, 0.0f}));

  assert(transport.sent_frames().empty());
}

void forwards_transport_error_when_enter_motor_mode_send_fails()
{
  iswv::FakeTransport transport;
  bxi::Motor motor(transport, 1);
  transport.close();

  const auto result = motor.enter_motor_mode();

  assert(!result);
  assert(result.error().code == iswv::ErrorCode::transport_closed);
}

void forwards_transport_error_when_exit_motor_mode_send_fails()
{
  iswv::FakeTransport transport;
  bxi::Motor motor(transport, 1);
  transport.close();

  const auto result = motor.exit_motor_mode();

  assert(!result);
  assert(result.error().code == iswv::ErrorCode::transport_closed);
}

void supports_classic_can_frame_options_without_bitrate_switch()
{
  iswv::FakeTransport transport;
  bxi::Motor motor(transport, 1, bxi::FrameOptions{false, false});

  const auto result = motor.command({0.0f, 0.0f, 0.0f, 0.0f, 0.0f});

  assert(result);
  const auto frames = transport.sent_frames();
  assert(frames.size() == 1);
  assert(!frames.front().fd);
  assert(!frames.front().bitrate_switch);
}

void rejects_bitrate_switch_without_can_fd()
{
  iswv::FakeTransport transport;
  bool rejected = false;

  try {
    bxi::Motor motor(transport, 1, bxi::FrameOptions{false, true});
  } catch (const std::invalid_argument &) {
    rejected = true;
  }

  assert(rejected);
}

void custom_ranges_encode_all_fields_and_are_local_to_each_command()
{
  // Deliberately asymmetric test ranges; these are not model specifications.
  bxi::Command command{14.0F, 35.0F, 300.0F, 6.0F, 50.0F};
  command.ranges = {{10.0F, 26.0F}, {-10.0F, 50.0F}, {100.0F, 500.0F},
    {2.0F, 10.0F}, {-20.0F, 120.0F}};
  const std::array<std::uint8_t, 8> expected{0x3f, 0xff, 0xbf, 0xf7, 0xff, 0x7f, 0xf7, 0xff};
  const auto packed = bxi::pack_command(command);
  assert(packed && packed.value() == expected);

  iswv::FakeTransport transport;
  bxi::Motor motor(transport, 2);
  assert(motor.command(command));
  const auto frames = transport.sent_frames();
  assert(frames.size() == 1 && frames[0].id == 2);
  for (std::size_t i = 0; i < expected.size(); ++i) {
    assert(frames[0].data[i] == expected[i]);
  }

  const bxi::Command original{14.0F, 35.0F, 300.0F, 6.0F, 50.0F};
  assert_invalid_argument(bxi::pack_command(original));
  command.position = 10.0F;
  command.velocity = -10.0F;
  command.kp = 100.0F;
  command.kd = 2.0F;
  command.torque = -20.0F;
  const auto minimum = bxi::pack_command(command);
  assert(minimum && same_bytes(minimum.value(), {0, 0, 0, 0, 0, 0, 0, 0}));
  command.position = 26.0F;
  command.velocity = 50.0F;
  command.kp = 500.0F;
  command.kd = 10.0F;
  command.torque = 120.0F;
  const auto maximum = bxi::pack_command(command);
  assert(
    maximum && same_bytes(
      maximum.value(),
      {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}));
}

void invalid_custom_ranges_and_values_never_send()
{
  iswv::FakeTransport transport;
  bxi::Motor motor(transport, 1);
  const std::array<bxi::Range bxi::EncodingRanges::*, 5> fields{
    &bxi::EncodingRanges::position, &bxi::EncodingRanges::velocity,
    &bxi::EncodingRanges::kp, &bxi::EncodingRanges::kd, &bxi::EncodingRanges::torque};
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();
  for (auto field : fields) {
    for (const auto range : {bxi::Range{0, 0}, bxi::Range{1, -1},
        bxi::Range{nan, 1}, bxi::Range{-1, nan}, bxi::Range{-inf, 1},
        bxi::Range{-1, inf}, bxi::Range{1, 2}, bxi::Range{-2, -1}})
    {
      bxi::Command command;
      command.ranges.*field = range;
      assert_invalid_argument(bxi::pack_command(command));
      assert_invalid_argument(motor.command(command));
    }
  }
  assert(transport.sent_frames().empty());
}

void custom_ranges_preserve_mode_collision_rejection()
{
  bxi::Command command{65535.0F, 4095.0F, 4095.0F, 4095.0F, 4092.0F};
  command.ranges = {{0, 65535}, {0, 4095}, {0, 4095}, {0, 4095}, {0, 4095}};
  assert_invalid_argument(bxi::pack_command(command));
  command.torque = 4093.0F;
  assert_invalid_argument(bxi::pack_command(command));
}

void feedback_uses_custom_ranges_and_rejects_invalid_ranges()
{
  auto frame = valid_feedback_frame();
  frame.data[0] = 0xa5;
  bxi::EncodingRanges ranges;
  ranges.position = {10, 26};
  ranges.velocity = {-10, 50};
  const auto minimum = bxi::decode_feedback(frame, ranges);
  assert(minimum && minimum.value().position == 10 && minimum.value().velocity == -10);
  frame.data[1] = 0xff;
  frame.data[2] = 0xff;
  frame.data[3] = 0xff;
  frame.data[4] = 0xf0;
  const auto maximum = bxi::decode_feedback(frame, ranges);
  assert(maximum && maximum.value().position == 26 && maximum.value().velocity == 50);
  assert(maximum.value().prefix == 0xa5);
  frame.data[1] = 0x3f;
  frame.data[3] = 0xbf;
  const auto interior = bxi::decode_feedback(frame, ranges);
  assert(interior && std::fabs(interior.value().position - 14.0F) < 0.001F);
  assert(std::fabs(interior.value().velocity - 35.0F) < 0.02F);

  for (auto field : {&bxi::EncodingRanges::position, &bxi::EncodingRanges::velocity}) {
    for (const auto range : {bxi::Range{1, 1}, bxi::Range{1, -1},
        bxi::Range{std::numeric_limits<float>::quiet_NaN(), 1},
        bxi::Range{-1, std::numeric_limits<float>::infinity()}})
    {
      auto invalid = ranges;
      invalid.*field = range;
      const auto result = bxi::decode_feedback(frame, invalid);
      assert(!result && result.error().code == iswv::ErrorCode::invalid_argument);
    }
  }
}

void large_finite_ranges_do_not_overflow_encoding_or_feedback()
{
  bxi::Command command;
  const float largest = std::numeric_limits<float>::max();
  command.ranges.position = {-largest, largest};
  command.ranges.velocity = {-largest, largest};
  const auto packed = bxi::pack_command(command);
  assert(packed && packed.value()[0] == 0x7f && packed.value()[1] == 0xff);
  assert(packed.value()[2] == 0x7f && (packed.value()[3] >> 4) == 0xf);
  auto frame = valid_feedback_frame();
  const auto minimum = bxi::decode_feedback(frame, command.ranges);
  assert(minimum && minimum.value().position == -largest && minimum.value().velocity == -largest);
  frame.data[1] = 0xff;
  frame.data[2] = 0xff;
  frame.data[3] = 0xff;
  frame.data[4] = 0xf0;
  const auto maximum = bxi::decode_feedback(frame, command.ranges);
  assert(maximum && maximum.value().position == largest && maximum.value().velocity == largest);
}

void official_model_ranges_control_encoding_and_feedback()
{
  const std::array<bxi::Model, 4> models{
    bxi::Model::BXI5014_19, bxi::Model::BXI5018_19,
    bxi::Model::BXI7010_19, bxi::Model::BXI8515_19};
  const std::array<float, 4> torque_max{40, 40, 80, 160};
  const std::array<float, 4> kd_max{5, 5, 5, 20};
  // At torque=40 Nm and Kd=5, different model scales yield different wire values.
  const std::array<unsigned, 4> torque_raw{4095, 4095, 3071, 2559};
  const std::array<unsigned, 4> kd_raw{4095, 4095, 4095, 1023};
  iswv::FakeTransport transport;
  bxi::Motor motor(transport, 1);
  for (std::size_t i = 0; i < models.size(); ++i) {
    auto ranges = bxi::encoding_ranges(models[i]);
    assert(ranges.position.min == -12.5F && ranges.position.max == 12.5F);
    assert(ranges.velocity.min == -45 && ranges.velocity.max == 45);
    assert(ranges.kp.min == 0 && ranges.kp.max == 500);
    assert(ranges.kd.min == 0 && ranges.kd.max == kd_max[i]);
    assert(ranges.torque.min == -torque_max[i] && ranges.torque.max == torque_max[i]);
    bxi::Command command{0, 0, 0, 5, 40, ranges};
    assert(motor.command(command));
    const auto data = transport.sent_frames().back().data;
    assert(((static_cast<unsigned>(data[5]) << 4) | (data[6] >> 4)) == kd_raw[i]);
    assert((((data[6] & 0x0FU) << 8) | data[7]) == torque_raw[i]);
    for (const float sign : {-1.0F, 1.0F}) {
      command.torque = sign * torque_max[i];
      assert(bxi::pack_command(command));
      command.torque = sign * (torque_max[i] + 1);
      const auto before = transport.sent_frames().size();
      assert_invalid_argument(motor.command(command));
      assert(transport.sent_frames().size() == before);
    }
    command.torque = 0;
    command.kd = kd_max[i] + 1;
    assert_invalid_argument(motor.command(command));
    auto frame = valid_feedback_frame();
    const auto feedback = bxi::decode_feedback(frame, ranges);
    assert(feedback && feedback.value().position == -12.5F && feedback.value().velocity == -45);
    // Runtime overrides remain local and do not mutate model defaults.
    ranges.velocity = {-60, 60};
    const auto custom = bxi::decode_feedback(frame, ranges);
    assert(custom && custom.value().velocity == -60);
    assert(bxi::encoding_ranges(models[i]).velocity.max == 45);
  }
  bool rejected = false;
  try {
    bxi::encoding_ranges(static_cast<bxi::Model>(99));
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  assert(rejected);
}

void rejects_all_documented_special_frames_without_sending()
{
  iswv::FakeTransport transport;
  bxi::Motor motor(transport, 1);
  // Identity scales make each target payload exact, independent of float rounding.
  bxi::Command command{65535, 4095, 4095, 4095, 4090};
  command.ranges = {{0, 65535}, {0, 4095}, {0, 4095}, {0, 4095}, {0, 4095}};
  for (unsigned last_byte = 0xfa; last_byte <= 0xfe; ++last_byte) {
    command.torque = static_cast<float>(0xf00U | last_byte);
    assert_invalid_argument(motor.command(command));
  }
  assert(transport.sent_frames().empty());
  command.torque = 4089;  // 0xF9 remains an ordinary payload.
  assert(bxi::pack_command(command));
  command.torque = 4095;  // 0xFF remains an ordinary payload.
  assert(bxi::pack_command(command));
}

}  // namespace

int main()
{
  packs_minimum_command_like_source_pack_cmd();
  packs_zero_command_like_source_pack_cmd();
  packs_maximum_command_like_source_pack_cmd();
  packs_representative_commands_like_source_pack_cmd();
  packs_random_legal_commands_like_source_pack_cmd();
  rejects_gain_outside_source_range();
  rejects_nan_command_values();
  rejects_infinite_command_values();
  rejects_payload_that_collides_with_enter_motor_mode_control_frame();
  rejects_payload_that_collides_with_exit_motor_mode_control_frame();
  decodes_minimum_feedback_endpoint_like_source_feedback();
  decodes_maximum_feedback_endpoint_like_source_feedback();
  rejects_short_feedback_frame();
  rejects_extended_feedback_frame();
  rejects_remote_feedback_frame();
  rejects_error_feedback_frame();
  sends_enter_motor_mode_control_frame();
  sends_exit_motor_mode_control_frame();
  command_sends_encoded_payload_to_motor_id();
  command_rejects_invalid_payload_without_sending();
  forwards_transport_error_when_enter_motor_mode_send_fails();
  forwards_transport_error_when_exit_motor_mode_send_fails();
  supports_classic_can_frame_options_without_bitrate_switch();
  rejects_bitrate_switch_without_can_fd();
  custom_ranges_encode_all_fields_and_are_local_to_each_command();
  invalid_custom_ranges_and_values_never_send();
  custom_ranges_preserve_mode_collision_rejection();
  feedback_uses_custom_ranges_and_rejects_invalid_ranges();
  large_finite_ranges_do_not_overflow_encoding_or_feedback();
  official_model_ranges_control_encoding_and_feedback();
  rejects_all_documented_special_frames_without_sending();
}
