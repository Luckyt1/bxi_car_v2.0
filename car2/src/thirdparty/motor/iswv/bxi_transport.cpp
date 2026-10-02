#include "motor/iswv/bxi_transport.h"

#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>

namespace iswv
{
namespace
{
bxi::CanFrame to_bxi_frame(const CanFrame & input)
{
  bxi::CanFrame output;
  output.id = input.id;
  output.extended = input.extended;
  output.remote = input.remote;
  output.error = input.error;
  output.fd = input.fd;
  output.bitrate_switch = input.bitrate_switch;
  output.error_state_indicator = input.error_state_indicator;
  output.size = input.size;
  output.data = input.data;
  output.received_at = input.received_at;
  return output;
}

CanFrame to_iswv_frame(const bxi::CanFrame & input)
{
  CanFrame output;
  output.id = input.id;
  output.extended = input.extended;
  output.remote = input.remote;
  output.error = input.error;
  output.fd = input.fd;
  output.bitrate_switch = input.bitrate_switch;
  output.error_state_indicator = input.error_state_indicator;
  output.size = input.size;
  output.data = input.data;
  output.received_at = input.received_at;
  return output;
}

ErrorCode to_iswv_error_code(bxi::ErrorCode code)
{
  switch (code) {
    case bxi::ErrorCode::none:
      return ErrorCode::none;
    case bxi::ErrorCode::invalid_argument:
      return ErrorCode::invalid_argument;
    case bxi::ErrorCode::transport_closed:
      return ErrorCode::transport_closed;
    case bxi::ErrorCode::transport_error:
      return ErrorCode::transport_error;
    case bxi::ErrorCode::protocol_error:
      return ErrorCode::protocol_error;
  }
  return ErrorCode::transport_error;
}

Error to_iswv_error(const bxi::Error & input)
{
  return Error{to_iswv_error_code(input.code), input.message, std::nullopt};
}

Result<void> to_iswv_result(bxi::Result<void> result)
{
  if (result) {
    return Result<void>::success();
  }
  return Result<void>::failure(to_iswv_error(result.error()));
}
}  // namespace

struct BxiTransport::State
{
  std::mutex mutex;
  ReceiveHandler receive_handler;
  TransportErrorHandler error_handler;
};

BxiTransport::BxiTransport(std::shared_ptr<bxi::ICanTransport> transport)
: transport_(std::move(transport)), state_(std::make_shared<State>())
{
  if (!transport_) {
    throw std::invalid_argument("BXI transport must not be null");
  }

  std::weak_ptr<State> weak_state = state_;
  transport_->set_receive_handler(
    [weak_state](const bxi::CanFrame & frame) {
      auto state = weak_state.lock();
      if (!state) {
        return;
      }
      ReceiveHandler handler;
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        handler = state->receive_handler;
      }
      if (handler) {
        handler(to_iswv_frame(frame));
      }
    });

  transport_->set_error_handler(
    [weak_state](const bxi::Error & error) {
      auto state = weak_state.lock();
      if (!state) {
        return;
      }
      TransportErrorHandler handler;
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        handler = state->error_handler;
      }
      if (handler) {
        handler(to_iswv_error(error));
      }
    });
}

BxiTransport::~BxiTransport()
{
  if (state_) {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->receive_handler = {};
    state_->error_handler = {};
  }
  if (transport_) {
    transport_->set_receive_handler({});
    transport_->set_error_handler({});
  }
}

Result<void> BxiTransport::send(const CanFrame & frame)
{
  return to_iswv_result(transport_->send(to_bxi_frame(frame)));
}

void BxiTransport::set_receive_handler(ReceiveHandler handler)
{
  std::lock_guard<std::mutex> lock(state_->mutex);
  state_->receive_handler = std::move(handler);
}

void BxiTransport::set_error_handler(TransportErrorHandler handler)
{
  std::lock_guard<std::mutex> lock(state_->mutex);
  state_->error_handler = std::move(handler);
}

bool BxiTransport::is_open() const noexcept
{
  return transport_->is_open();
}

std::string BxiTransport::name() const
{
  return transport_->name();
}

const std::shared_ptr<bxi::ICanTransport> & BxiTransport::transport() const noexcept
{
  return transport_;
}

}  // namespace iswv
