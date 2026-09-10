#pragma once

#include "bxi_pci_drv.h"
#include "iswv/transport.hpp"

#include <atomic>
#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>

namespace chassis
{

class BxiPciTransport final : public iswv::ICanTransport
{
public:
  explicit BxiPciTransport(unsigned int bus, bool trace = false);
  ~BxiPciTransport() override;

  BxiPciTransport(const BxiPciTransport &) = delete;
  BxiPciTransport & operator=(const BxiPciTransport &) = delete;

  iswv::Result<void> send(const iswv::CanFrame & frame) override;
  void set_receive_handler(iswv::ReceiveHandler handler) override;
  void set_error_handler(iswv::TransportErrorHandler handler) override;
  bool is_open() const noexcept override;
  std::string name() const override;

  iswv::Result<void> set_motor_power(bool enabled);

  static iswv::Result<canfd_packet> packet_from_frame(
    unsigned int bus, const iswv::CanFrame & frame);
  static iswv::Result<iswv::CanFrame> frame_from_packet(
    const canfd_packet & packet);

private:
  static int receive_callback(void * context, canfd_packet * packet);
  void receive(const canfd_packet & packet);
  void report_error(iswv::Error error);

  static std::mutex registry_mutex_;
  static std::array<BxiPciTransport *, CANFD_DEVICE_NUM> transports_;
  static std::size_t transport_count_;

  const unsigned int bus_;
  const bool trace_;
  std::atomic<bool> open_{false};
  mutable std::mutex handler_mutex_;
  iswv::ReceiveHandler receive_handler_;
  iswv::TransportErrorHandler error_handler_;
};

}  // namespace chassis
