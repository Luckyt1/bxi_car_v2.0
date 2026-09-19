#include "iswv/fake_transport.hpp"

namespace iswv
{

Result<void> FakeTransport::send(const CanFrame & frame)
{
  if (!valid_frame(frame)) {
    return Result<void>::failure(ErrorCode::invalid_argument, "invalid CAN frame");
  }

  SendHook hook;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!open_) {
      return Result<void>::failure(
        ErrorCode::transport_closed,
        "fake transport is closed");
    }
    sent_.push_back(frame);
    hook = send_hook_;
  }
  sent_cv_.notify_all();
  if (hook) {
    hook(frame);
  }
  return Result<void>::success();
}

void FakeTransport::set_receive_handler(ReceiveHandler handler)
{
  std::lock_guard<std::mutex> lock(mutex_);
  receive_handler_ = std::move(handler);
}

void FakeTransport::set_error_handler(TransportErrorHandler handler)
{
  std::lock_guard<std::mutex> lock(mutex_);
  error_handler_ = std::move(handler);
}

bool FakeTransport::is_open() const noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  return open_;
}

std::string FakeTransport::name() const
{
  return "fake";
}

void FakeTransport::inject(const CanFrame & frame)
{
  ReceiveHandler handler;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!open_) {
      return;
    }
    handler = receive_handler_;
  }
  if (handler) {
    auto copy = frame;
    if (copy.received_at == std::chrono::steady_clock::time_point{}) {
      copy.received_at = std::chrono::steady_clock::now();
    }
    handler(copy);
  }
}

void FakeTransport::inject_error(Error error)
{
  TransportErrorHandler handler;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    handler = error_handler_;
  }
  if (handler) {
    handler(error);
  }
}

void FakeTransport::set_send_hook(SendHook hook)
{
  std::lock_guard<std::mutex> lock(mutex_);
  send_hook_ = std::move(hook);
}

void FakeTransport::close()
{
  std::lock_guard<std::mutex> lock(mutex_);
  open_ = false;
}

void FakeTransport::open()
{
  std::lock_guard<std::mutex> lock(mutex_);
  open_ = true;
}

bool FakeTransport::wait_for_sent(CanFrame & frame, std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lock(mutex_);
  if (!sent_cv_.wait_for(lock, timeout, [this] {return !sent_.empty();})) {
    return false;
  }
  frame = sent_.front();
  sent_.pop_front();
  return true;
}

std::vector<CanFrame> FakeTransport::sent_frames() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return {sent_.begin(), sent_.end()};
}

void FakeTransport::clear_sent()
{
  std::lock_guard<std::mutex> lock(mutex_);
  sent_.clear();
}

}  // namespace iswv
