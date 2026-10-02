#pragma once

#include "bxi_pci_drv.h"
#include "bxi/transport.h"

#include <atomic>
#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>

namespace chassis
{

// BXI 板卡：管理一条 CAN 通道和共享电机电源；不包含任何电机型号协议。
// 构造打开板卡，set_motor_power 显式上下电；最后一个通道析构时关闭共享电源。
// 电机和控制器应先于板卡析构，以便在通信仍可用时发送停车/失能指令。
class BxiPciTransport final : public bxi::ICanTransport
{
public:
  explicit BxiPciTransport(unsigned int bus, bool trace = false);
  ~BxiPciTransport() override;

  BxiPciTransport(const BxiPciTransport &) = delete;
  BxiPciTransport & operator=(const BxiPciTransport &) = delete;

  bxi::Result<void> send(const bxi::CanFrame & frame) override;
  void set_receive_handler(bxi::ReceiveHandler handler) override;
  void set_error_handler(bxi::TransportErrorHandler handler) override;
  bool is_open() const noexcept override;
  std::string name() const override;

  bxi::Result<void> set_motor_power(bool enabled);

  static bxi::Result<canfd_packet> packet_from_frame(
    unsigned int bus, const bxi::CanFrame & frame);
  static bxi::Result<bxi::CanFrame> frame_from_packet(
    const canfd_packet & packet);

private:
  static int receive_callback(void * context, canfd_packet * packet);
  void receive(const canfd_packet & packet);
  void report_error(bxi::Error error);

  static std::mutex registry_mutex_;
  static std::array<BxiPciTransport *, CANFD_DEVICE_NUM> transports_;
  static std::size_t transport_count_;

  const unsigned int bus_;
  const bool trace_;
  std::atomic<bool> open_{false};
  mutable std::mutex handler_mutex_;
  bxi::ReceiveHandler receive_handler_;
  bxi::TransportErrorHandler error_handler_;
};

}  // namespace chassis
