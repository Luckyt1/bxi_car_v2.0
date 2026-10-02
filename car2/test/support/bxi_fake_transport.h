#pragma once

#include "bxi/transport.h"

#include <vector>

namespace bxi_test
{
// Single-threaded recording transport for offline motor tests; never opens hardware.
class FakeTransport final : public bxi::ICanTransport
{
public:
  bxi::Result<void> send(const bxi::CanFrame & frame) override
  {
    if (!bxi::valid_frame(frame)) {
      return bxi::Result<void>::failure(bxi::ErrorCode::invalid_argument, "invalid CAN frame");
    }
    if (!open_) {
      return bxi::Result<void>::failure(bxi::ErrorCode::transport_closed, "fake is closed");
    }
    frames_.push_back(frame);
    return bxi::Result<void>::success();
  }
  void set_receive_handler(bxi::ReceiveHandler) override {}
  void set_error_handler(bxi::TransportErrorHandler) override {}
  bool is_open() const noexcept override {return open_;}
  std::string name() const override {return "bxi-fake";}
  void close() {open_ = false;}
  const std::vector<bxi::CanFrame> & sent_frames() const {return frames_;}
  void clear_sent() {frames_.clear();}

private:
  bool open_{true};
  std::vector<bxi::CanFrame> frames_;
};
}  // namespace bxi_test
