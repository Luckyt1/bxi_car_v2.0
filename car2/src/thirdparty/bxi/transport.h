#pragma once

#include "bxi/can_frame.h"
#include "bxi/result.h"

#include <functional>
#include <string>

namespace bxi
{

using ReceiveHandler = std::function<void (const CanFrame &)>;
using TransportErrorHandler = std::function<void (const Error &)>;

class ICanTransport
{
public:
  virtual ~ICanTransport() = default;

  virtual Result<void> send(const CanFrame & frame) = 0;
  virtual void set_receive_handler(ReceiveHandler handler) = 0;
  virtual void set_error_handler(TransportErrorHandler handler) = 0;
  virtual bool is_open() const noexcept = 0;
  virtual std::string name() const = 0;
};

}  // namespace bxi
