#include "motor/iswv/bxi_transport.h"

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {
    throw std::runtime_error(message);
  }
}

class FakeBxiTransport final : public bxi::ICanTransport
{
public:
  bxi::Result<void> send(const bxi::CanFrame & frame) override
  {
    if (send_error_) {
      return bxi::Result<void>::failure(*send_error_);
    }
    sent_.push_back(frame);
    return bxi::Result<void>::success();
  }

  void set_receive_handler(bxi::ReceiveHandler handler) override
  {
    receive_handler_ = std::move(handler);
    ++receive_handler_sets_;
  }

  void set_error_handler(bxi::TransportErrorHandler handler) override
  {
    error_handler_ = std::move(handler);
    ++error_handler_sets_;
  }

  bool is_open() const noexcept override {return open_;}
  std::string name() const override {return "fake-bxi-adapter";}

  void inject(const bxi::CanFrame & frame)
  {
    if (receive_handler_) {
      receive_handler_(frame);
    }
  }

  void inject_error(const bxi::Error & error)
  {
    if (error_handler_) {
      error_handler_(error);
    }
  }

  void fail_send(bxi::Error error) {send_error_ = std::move(error);}

  bool has_receive_handler() const {return static_cast<bool>(receive_handler_);}
  bool has_error_handler() const {return static_cast<bool>(error_handler_);}

  bool open_{true};
  std::vector<bxi::CanFrame> sent_;
  std::optional<bxi::Error> send_error_;
  bxi::ReceiveHandler receive_handler_;
  bxi::TransportErrorHandler error_handler_;
  unsigned receive_handler_sets_{0};
  unsigned error_handler_sets_{0};
};

iswv::CanFrame sample_iswv_frame()
{
  iswv::CanFrame frame;
  frame.id = 0x1ABCDE;
  frame.extended = true;
  frame.fd = true;
  frame.bitrate_switch = true;
  frame.error_state_indicator = true;
  frame.size = 12;
  for (std::uint8_t i = 0; i < frame.size; ++i) {
    frame.data[i] = static_cast<std::uint8_t>(i + 3U);
  }
  frame.received_at = std::chrono::steady_clock::now() - std::chrono::seconds(3);
  return frame;
}

bxi::CanFrame sample_bxi_frame()
{
  bxi::CanFrame frame;
  frame.id = 0x321;
  frame.remote = true;
  frame.size = 8;
  for (std::uint8_t i = 0; i < frame.size; ++i) {
    frame.data[i] = static_cast<std::uint8_t>(0xf0U - i);
  }
  frame.received_at = std::chrono::steady_clock::now() - std::chrono::milliseconds(7);
  return frame;
}

void require_same_frame(const bxi::CanFrame & actual, const iswv::CanFrame & expected)
{
  require(actual.id == expected.id, "id must be converted");
  require(actual.extended == expected.extended, "extended flag must be converted");
  require(actual.remote == expected.remote, "remote flag must be converted");
  require(actual.error == expected.error, "error flag must be converted");
  require(actual.fd == expected.fd, "fd flag must be converted");
  require(actual.bitrate_switch == expected.bitrate_switch, "brs flag must be converted");
  require(
    actual.error_state_indicator == expected.error_state_indicator,
    "esi flag must be converted");
  require(actual.size == expected.size, "size must be converted");
  require(actual.data == expected.data, "payload must be converted");
  require(actual.received_at == expected.received_at, "timestamp must be converted");
}

void require_same_frame(const iswv::CanFrame & actual, const bxi::CanFrame & expected)
{
  require(actual.id == expected.id, "id must be converted");
  require(actual.extended == expected.extended, "extended flag must be converted");
  require(actual.remote == expected.remote, "remote flag must be converted");
  require(actual.error == expected.error, "error flag must be converted");
  require(actual.fd == expected.fd, "fd flag must be converted");
  require(actual.bitrate_switch == expected.bitrate_switch, "brs flag must be converted");
  require(
    actual.error_state_indicator == expected.error_state_indicator,
    "esi flag must be converted");
  require(actual.size == expected.size, "size must be converted");
  require(actual.data == expected.data, "payload must be converted");
  require(actual.received_at == expected.received_at, "timestamp must be converted");
}

void send_converts_frame_and_errors()
{
  auto fake = std::make_shared<FakeBxiTransport>();
  iswv::BxiTransport adapter(fake);
  const auto frame = sample_iswv_frame();
  require(adapter.send(frame).has_value(), "send success must pass through");
  require(fake->sent_.size() == 1, "send must reach BXI transport once");
  require_same_frame(fake->sent_.front(), frame);

  fake->fail_send({bxi::ErrorCode::transport_closed, "closed"});
  const auto closed = adapter.send(iswv::CanFrame{});
  require(
    !closed && closed.error().code == iswv::ErrorCode::transport_closed &&
    closed.error().message == "closed", "send error must convert without enum casts");
}

void receive_converts_frame_and_replaces_callback()
{
  auto fake = std::make_shared<FakeBxiTransport>();
  iswv::BxiTransport adapter(fake);
  const auto frame = sample_bxi_frame();
  unsigned first_calls = 0;
  unsigned second_calls = 0;
  iswv::CanFrame received;

  adapter.set_receive_handler(
    [&](const iswv::CanFrame &) {
      ++first_calls;
    });
  fake->inject(frame);
  adapter.set_receive_handler(
    [&](const iswv::CanFrame & input) {
      ++second_calls;
      received = input;
    });
  fake->inject(frame);

  require(first_calls == 1, "old receive callback must not be called after replacement");
  require(second_calls == 1, "new receive callback must be called");
  require_same_frame(received, frame);
}

void error_converts_and_replaces_callback()
{
  auto fake = std::make_shared<FakeBxiTransport>();
  iswv::BxiTransport adapter(fake);
  unsigned first_calls = 0;
  unsigned second_calls = 0;
  iswv::Error received;

  adapter.set_error_handler(
    [&](const iswv::Error &) {
      ++first_calls;
    });
  fake->inject_error({bxi::ErrorCode::invalid_argument, "bad"});
  adapter.set_error_handler(
    [&](const iswv::Error & input) {
      ++second_calls;
      received = input;
    });
  fake->inject_error({bxi::ErrorCode::protocol_error, "protocol"});

  require(first_calls == 1, "old error callback must not be called after replacement");
  require(second_calls == 1, "new error callback must be called");
  require(
    received.code == iswv::ErrorCode::protocol_error && received.message == "protocol",
    "error must be converted");
}

void every_non_none_error_code_is_mapped()
{
  auto fake = std::make_shared<FakeBxiTransport>();
  iswv::BxiTransport adapter(fake);
  std::vector<iswv::ErrorCode> codes;
  adapter.set_error_handler(
    [&](const iswv::Error & input) {
      codes.push_back(input.code);
    });

  fake->inject_error({bxi::ErrorCode::invalid_argument, "invalid"});
  fake->inject_error({bxi::ErrorCode::transport_closed, "closed"});
  fake->inject_error({bxi::ErrorCode::transport_error, "transport"});
  fake->inject_error({bxi::ErrorCode::protocol_error, "protocol"});

  const std::vector<iswv::ErrorCode> expected{
    iswv::ErrorCode::invalid_argument,
    iswv::ErrorCode::transport_closed,
    iswv::ErrorCode::transport_error,
    iswv::ErrorCode::protocol_error};
  require(codes == expected, "every non-none BXI error code must map by name");
}

void cleanup_clears_underlying_callbacks()
{
  auto fake = std::make_shared<FakeBxiTransport>();
  {
    iswv::BxiTransport adapter(fake);
    adapter.set_receive_handler([](const iswv::CanFrame &) {});
    adapter.set_error_handler([](const iswv::Error &) {});
    require(fake->has_receive_handler(), "constructor must install receive bridge");
    require(fake->has_error_handler(), "constructor must install error bridge");
  }
  require(!fake->has_receive_handler(), "destructor must clear receive bridge");
  require(!fake->has_error_handler(), "destructor must clear error bridge");
}

void copied_callback_after_destruction_is_safe()
{
  auto fake = std::make_shared<FakeBxiTransport>();
  bxi::ReceiveHandler saved_receive;
  bxi::TransportErrorHandler saved_error;
  unsigned receive_calls = 0;
  unsigned error_calls = 0;
  {
    iswv::BxiTransport adapter(fake);
    adapter.set_receive_handler(
      [&](const iswv::CanFrame &) {
        ++receive_calls;
      });
    adapter.set_error_handler(
      [&](const iswv::Error &) {
        ++error_calls;
      });
    saved_receive = fake->receive_handler_;
    saved_error = fake->error_handler_;
  }

  require(static_cast<bool>(saved_receive), "receive bridge copy must exist");
  require(static_cast<bool>(saved_error), "error bridge copy must exist");
  saved_receive(sample_bxi_frame());
  saved_error({bxi::ErrorCode::transport_error, "late"});
  require(receive_calls == 0, "late receive bridge must not call destroyed adapter handler");
  require(error_calls == 0, "late error bridge must not call destroyed adapter handler");
}

void metadata_and_transport_access_pass_through()
{
  auto fake = std::make_shared<FakeBxiTransport>();
  iswv::BxiTransport adapter(fake);
  require(adapter.is_open(), "open state must pass through");
  fake->open_ = false;
  require(!adapter.is_open(), "closed state must pass through");
  require(adapter.name() == "fake-bxi-adapter", "name must pass through");
  require(adapter.transport() == fake, "underlying transport must be exposed");
}

void null_transport_is_rejected()
{
  bool rejected = false;
  try {
    iswv::BxiTransport adapter(nullptr);
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  require(rejected, "null BXI transport must be rejected");
}
}  // namespace

int main()
{
  send_converts_frame_and_errors();
  receive_converts_frame_and_replaces_callback();
  error_converts_and_replaces_callback();
  every_non_none_error_code_is_mapped();
  cleanup_clears_underlying_callbacks();
  copied_callback_after_destruction_is_safe();
  metadata_and_transport_access_pass_through();
  null_transport_is_rejected();
  return 0;
}
