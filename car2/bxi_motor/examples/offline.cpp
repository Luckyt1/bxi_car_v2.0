#include "bxi/motor.hpp"
#include "iswv/fake_transport.hpp"

#include <iostream>

int main()
{
  iswv::FakeTransport transport;
  bxi::Motor left(transport, 1);
  bxi::Motor right(transport, 2);
  bxi::Command command{0.0F, 0.0F, 0.0F, 1.0F, 0.0F};
  command.ranges = bxi::encoding_ranges(bxi::Model::BXI5014_19);
  for (auto * motor : {&left, &right}) {
    if (!motor->enter_motor_mode()) {return 1;}
    if (!motor->command(command)) {return 1;}
    if (!motor->exit_motor_mode()) {return 1;}
  }
  iswv::CanFrame incoming{};
  incoming.id = 0x11;
  incoming.size = 8;
  incoming.data[0] = 1;
  incoming.data[1] = 0x7f;
  incoming.data[2] = 0xff;
  incoming.data[3] = 0x7f;
  incoming.data[4] = 0xf0;
  auto feedback = bxi::decode_feedback(incoming, command.ranges);
  if (!feedback) {return 1;}
  std::cout << "FakeTransport frames: " << transport.sent_frames().size()
            << "; decoded position=" << feedback.value().position
            << ", velocity=" << feedback.value().velocity << '\n';
  return 0;
}
