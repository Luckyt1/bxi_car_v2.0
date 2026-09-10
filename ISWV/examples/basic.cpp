#include "iswv/fake_transport.hpp"
#include "iswv/iswv/steering_layout.hpp"

#include <iostream>
#include <memory>

int main()
{
    // Replace FakeTransport with a ZqwlDeviceHub channel or RosTopicHub bus in a real app.
    auto transport = std::make_shared<iswv::FakeTransport>();
    iswv::canopen::CanopenMaster master(transport);
    iswv::SteeringLayout steering(master);

    const auto front_left = steering.at(iswv::WheelPosition::front_left);
    const auto angle_raw = front_left->steering_radians_to_inc(1.0);
    if (!angle_raw) {
        std::cerr << angle_raw.error().message << '\n';
        return 1;
    }
    std::cout << "Front-left 1 rad raw value: " << angle_raw.value() << '\n';
    std::cout << "The library never enables or moves an axis during construction.\n";
    return 0;
}
