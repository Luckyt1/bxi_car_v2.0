#pragma once

#include "bxi/transport.h"
#include "iswv/transport.hpp"

#include <memory>

namespace iswv
{

// Adapts one BXI CAN transport for ISWV/CANopen code. While this adapter is
// alive, it owns the supplied transport's receive/error callback slots; do not
// wrap the same transport in multiple adapters or replace the native BXI
// handlers directly.
class BxiTransport final : public ICanTransport
{
public:
  explicit BxiTransport(std::shared_ptr<bxi::ICanTransport> transport);
  ~BxiTransport() override;

  BxiTransport(const BxiTransport &) = delete;
  BxiTransport & operator=(const BxiTransport &) = delete;

  Result<void> send(const CanFrame & frame) override;
  void set_receive_handler(ReceiveHandler handler) override;
  void set_error_handler(TransportErrorHandler handler) override;
  bool is_open() const noexcept override;
  std::string name() const override;

  const std::shared_ptr<bxi::ICanTransport> & transport() const noexcept;

private:
  struct State;

  std::shared_ptr<bxi::ICanTransport> transport_;
  std::shared_ptr<State> state_;
};

}  // namespace iswv
