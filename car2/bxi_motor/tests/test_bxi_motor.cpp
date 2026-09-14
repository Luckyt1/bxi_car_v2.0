#include "bxi/motor.hpp"
#include "iswv/fake_transport.hpp"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <stdexcept>

int main()
{
  iswv::FakeTransport transport;
  {
    bxi::Motor motor(transport, 1);
    assert(transport.sent_frames().empty());
    assert(motor.exit_motor_mode());
    const auto frames = transport.sent_frames();
    assert(frames.size() == 1);
    const auto & frame = frames.front();
    assert(frame.id == 1 && frame.size == 8);
    assert(frame.fd && frame.bitrate_switch);
    assert(!frame.extended && !frame.remote && !frame.error);
    assert(!frame.error_state_indicator);
    const unsigned char expected[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
    for (unsigned i = 0; i < 8; ++i) {assert(frame.data[i] == expected[i]);}
    transport.close();
    const auto failed = motor.exit_motor_mode();
    assert(!failed && failed.error().code == iswv::ErrorCode::transport_closed);
  }
  // Object destruction must not issue a command or assume ownership of shared power.
  assert(transport.sent_frames().size() == 1);
  bool rejected = false;
  try {bxi::Motor invalid(transport, 0x800);} catch (const std::invalid_argument &) {
    rejected = true;
  }
  assert(rejected);
  transport.open();
  bxi::Motor different_id(transport, 2);
  assert(different_id.exit_motor_mode());
  assert(transport.sent_frames().back().id == 2);
}
