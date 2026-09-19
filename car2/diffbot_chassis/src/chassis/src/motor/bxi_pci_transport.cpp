#include "chassis/motor/bxi_pci_transport.h"

#include "iswv/can_frame.hpp"

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <linux/can.h>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace chassis
{
std::mutex BxiPciTransport::registry_mutex_;
std::array<BxiPciTransport *, CANFD_DEVICE_NUM> BxiPciTransport::transports_{};
std::size_t BxiPciTransport::transport_count_{0};

namespace
{
void trace_frame(const char * direction, unsigned int bus, const iswv::CanFrame & frame)
{
  static std::mutex output_mutex;
  std::ostringstream line;
  const auto time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  line << "CAN " << direction << " t_ms=" << time_ms << " bus=" << bus
       << " id=0x" << std::hex << std::setfill('0') << std::setw(3) << frame.id
       << std::dec << " dlc=" << static_cast<unsigned>(frame.size)
       << " extended=" << frame.extended << " fd=" << frame.fd
       << " rtr=" << frame.remote << " error=" << frame.error << " data=";
  for (std::size_t i = 0; i < frame.size; ++i) {
    line << std::hex << std::setw(2) << static_cast<unsigned>(frame.data[i]) << ' ';
  }
  std::lock_guard<std::mutex> lock(output_mutex);
  std::cerr << line.str() << std::endl;
}
}  // namespace

BxiPciTransport::BxiPciTransport(unsigned int bus, bool trace)
: bus_(bus), trace_(trace)
{
  if (bus_ >= CANFD_DEVICE_NUM) {
    throw std::invalid_argument("BXI CAN bus is outside the device range");
  }
  std::lock_guard<std::mutex> lock(registry_mutex_);
  if (transports_[bus_] != nullptr) {
    throw std::runtime_error("BXI CAN bus already has an active transport");
  }
  if (transport_count_ == 0 &&
    bxi_pci_init(&BxiPciTransport::receive_callback, nullptr, -1) < 0)
  {
    throw std::runtime_error("failed to initialize BXI PCI CAN interface");
  }
  transports_[bus_] = this;
  ++transport_count_;
  open_.store(true);
}

BxiPciTransport::~BxiPciTransport()
{
  open_.store(false);
  {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    receive_handler_ = {};
    error_handler_ = {};
  }
  std::lock_guard<std::mutex> lock(registry_mutex_);
  if (transports_[bus_] == this) {
    transports_[bus_] = nullptr;
    --transport_count_;
  }
  if (transport_count_ == 0) {
    (void)motor_pwr_set(0);
    (void)bxi_pci_exit();
  }
}

iswv::Result<void> BxiPciTransport::send(const iswv::CanFrame & frame)
{
  if (!open_.load()) {
    return iswv::Result<void>::failure(
      iswv::ErrorCode::transport_closed, "BXI PCI CAN interface is closed");
  }
  auto packet = packet_from_frame(bus_, frame);
  if (!packet) {
    return iswv::Result<void>::failure(packet.error());
  }
  auto value = packet.take_value();
  if (trace_) {trace_frame("TX-attempt", bus_, frame);}
  // All CAN buses enqueue into the vendor's shared g_fifo_tx. Its kfifo
  // producer path is not locked, so serialize callers across every transport.
  // Keep this separate from registry_mutex_: receive callbacks use that lock.
  static std::mutex enqueue_mutex;
  std::lock_guard<std::mutex> lock(enqueue_mutex);
  if (canfd_send_packet(&value, 1) < 0) {
    return iswv::Result<void>::failure(
      iswv::ErrorCode::transport_error, "BXI PCI CAN send failed");
  }
  return iswv::Result<void>::success();
}

void BxiPciTransport::set_receive_handler(iswv::ReceiveHandler handler)
{
  std::lock_guard<std::mutex> lock(handler_mutex_);
  receive_handler_ = std::move(handler);
}

void BxiPciTransport::set_error_handler(iswv::TransportErrorHandler handler)
{
  std::lock_guard<std::mutex> lock(handler_mutex_);
  error_handler_ = std::move(handler);
}

bool BxiPciTransport::is_open() const noexcept
{
  return open_.load();
}

std::string BxiPciTransport::name() const
{
  return "bxi-pci:bus" + std::to_string(bus_);
}

iswv::Result<void> BxiPciTransport::set_motor_power(bool enabled)
{
  if (!open_.load()) {
    return iswv::Result<void>::failure(
      iswv::ErrorCode::transport_closed, "BXI PCI CAN interface is closed");
  }
  if (motor_pwr_set(enabled ? 1U : 0U) < 0) {
    return iswv::Result<void>::failure(
      iswv::ErrorCode::transport_error, "failed to switch motor power");
  }
  return iswv::Result<void>::success();
}

iswv::Result<canfd_packet> BxiPciTransport::packet_from_frame(
  unsigned int bus, const iswv::CanFrame & frame)
{
  if (!iswv::valid_frame(frame)) {
    return iswv::Result<canfd_packet>::failure(
      iswv::ErrorCode::invalid_argument, "invalid CAN frame");
  }

  canfd_packet packet{};
  packet.bus = bus;
  packet.frame.can_id = frame.id;
  if (frame.extended) {
    packet.frame.can_id |= CAN_EFF_FLAG;
  }
  if (frame.remote) {
    packet.frame.can_id |= CAN_RTR_FLAG;
  }
  if (frame.error) {
    packet.frame.can_id |= CAN_ERR_FLAG;
  }
  if (frame.fd) {
    packet.frame.flags |= CANFD_FDF;
  }
  if (frame.bitrate_switch) {
    packet.frame.flags |= CANFD_BRS;
  }
  if (frame.error_state_indicator) {
    packet.frame.flags |= CANFD_ESI;
  }
  packet.frame.len = frame.size;
  std::copy_n(frame.data.begin(), frame.size, packet.frame.data);
  return iswv::Result<canfd_packet>::success(packet);
}

iswv::Result<iswv::CanFrame> BxiPciTransport::frame_from_packet(
  const canfd_packet & packet)
{
  iswv::CanFrame frame;
  frame.error = (packet.frame.can_id & CAN_ERR_FLAG) != 0U;
  frame.extended = !frame.error && (packet.frame.can_id & CAN_EFF_FLAG) != 0U;
  frame.remote = (packet.frame.can_id & CAN_RTR_FLAG) != 0U;
  frame.id = packet.frame.can_id &
    (frame.error ? CAN_ERR_MASK :
    (frame.extended ? CAN_EFF_MASK : CAN_SFF_MASK));
  frame.fd = (packet.frame.flags & CANFD_FDF) != 0U;
  frame.bitrate_switch = (packet.frame.flags & CANFD_BRS) != 0U;
  frame.error_state_indicator = (packet.frame.flags & CANFD_ESI) != 0U;
  frame.size = packet.frame.len;
  frame.received_at = std::chrono::steady_clock::now();
  if (!iswv::valid_frame(frame)) {
    return iswv::Result<iswv::CanFrame>::failure(
      iswv::ErrorCode::protocol_error, "BXI PCI delivered an invalid CAN frame");
  }
  std::copy_n(packet.frame.data, frame.size, frame.data.begin());
  return iswv::Result<iswv::CanFrame>::success(frame);
}

int BxiPciTransport::receive_callback(void * context, canfd_packet * packet)
{
  (void)context;
  if (packet == nullptr || packet->bus >= CANFD_DEVICE_NUM) {
    return -1;
  }
  std::lock_guard<std::mutex> lock(registry_mutex_);
  auto * transport = transports_[packet->bus];
  if (transport != nullptr) {
    transport->receive(*packet);
  }
  return 0;
}

void BxiPciTransport::receive(const canfd_packet & packet)
{
  if (!open_.load() || packet.bus != bus_) {
    return;
  }
  auto frame = frame_from_packet(packet);
  if (!frame) {
    report_error(frame.error());
    return;
  }
  if (trace_) {trace_frame("RX", bus_, frame.value());}

  iswv::ReceiveHandler handler;
  {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    handler = receive_handler_;
  }
  if (handler) {
    handler(frame.value());
  }
}

void BxiPciTransport::report_error(iswv::Error error)
{
  iswv::TransportErrorHandler handler;
  {
    std::lock_guard<std::mutex> lock(handler_mutex_);
    handler = error_handler_;
  }
  if (handler) {
    handler(error);
  }
}

}  // namespace chassis
