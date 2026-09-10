#include "chassis/bxi_pci_transport.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <linux/can.h>

int main()
{
  iswv::CanFrame source;
  source.id = 0x601;
  source.size = 8;
  source.data[0] = 0x40;
  source.data[1] = 0x41;
  source.data[2] = 0x60;

  auto encoded = chassis::BxiPciTransport::packet_from_frame(2, source);
  assert(encoded);
  assert(encoded.value().bus == 2);
  assert(encoded.value().frame.can_id == 0x601);
  assert(encoded.value().frame.flags == 0);
  assert(encoded.value().frame.len == 8);

  auto decoded =
    chassis::BxiPciTransport::frame_from_packet(encoded.value());
  assert(decoded);
  assert(decoded.value().id == source.id);
  assert(!decoded.value().fd);
  assert(decoded.value().data[1] == 0x41);

  source.extended = true;
  source.id = 0x1ABCDE;
  source.fd = true;
  source.bitrate_switch = true;
  source.size = 16;
  encoded = chassis::BxiPciTransport::packet_from_frame(3, source);
  assert(encoded);
  assert((encoded.value().frame.can_id & CAN_EFF_FLAG) != 0U);
  assert((encoded.value().frame.flags & CANFD_FDF) != 0U);
  assert((encoded.value().frame.flags & CANFD_BRS) != 0U);

  decoded = chassis::BxiPciTransport::frame_from_packet(encoded.value());
  assert(decoded);
  assert(decoded.value().extended);
  assert(decoded.value().fd);
  assert(decoded.value().bitrate_switch);
  assert(decoded.value().id == source.id);
  return 0;
}
