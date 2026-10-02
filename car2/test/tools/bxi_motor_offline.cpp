#include "motor/bxi/motor.h"
#include "../support/bxi_fake_transport.h"

#include <iostream>

int main()
{
  bxi_test::FakeTransport transport;
  bxi::Motor left(transport, 1);
  bxi::Motor right(transport, 2);
  const auto model = bxi::Model::BXI5014_19;
  for (auto * motor : {&left, &right}) {
    if (!motor->enter_motor_mode()) {return 1;}
    if (!motor->set_velocity(0.0F, 1.0F, model)) {return 1;}
    if (!motor->exit_motor_mode()) {return 1;}
  }
  bxi::CanFrame incoming{};
  incoming.id = 0x11;
  incoming.size = 8;
  incoming.data[0] = 1;
  incoming.data[1] = 0x7f;
  incoming.data[2] = 0xff;
  incoming.data[3] = 0x7f;
  incoming.data[4] = 0xf0;
  auto feedback = bxi::decode_feedback(incoming, bxi::encoding_ranges(model));
  if (!feedback) {return 1;}
  std::cout << "FakeTransport frames: " << transport.sent_frames().size()
            << "; decoded position=" << feedback.value().position
            << ", velocity=" << feedback.value().velocity << '\n';
  return 0;
}
