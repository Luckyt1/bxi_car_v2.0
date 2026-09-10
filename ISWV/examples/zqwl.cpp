#include "iswv/iswv.hpp"
#include "iswv/transports/zqwl_transport.hpp"

#include <iostream>

int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cerr << "usage: iswv_zqwl_example /dev/zqwl-can-...\n";
        return 2;
    }

    auto hub = iswv::transports::ZqwlDeviceHub::open(argv[1]);
    zqwl::ChannelConfig configuration;
    configuration.channel = 0;
    configuration.arbitration_bitrate = 500000;
    auto configured = hub->configure_channel(configuration);
    if (!configured) {
        std::cerr << configured.error().message << '\n';
        return 1;
    }
    auto enabled = hub->set_channel_enabled(0, true);
    if (!enabled) {
        std::cerr << enabled.error().message << '\n';
        return 1;
    }

    iswv::canopen::CanopenMaster master(hub->channel(0));
    iswv::SteeringLayout steering(master);
    std::cout << "Created CAN0 steering layout: front-left="
              << static_cast<unsigned>(
                     steering.at(iswv::WheelPosition::front_left)->node()->id())
              << ", rear-left="
              << static_cast<unsigned>(
                     steering.at(iswv::WheelPosition::rear_left)->node()->id())
              << ", rear-right="
              << static_cast<unsigned>(
                     steering.at(iswv::WheelPosition::rear_right)->node()->id())
              << ", front-right="
              << static_cast<unsigned>(
                     steering.at(iswv::WheelPosition::front_right)->node()->id())
              << '\n';
    std::cout << "No motor command was sent.\n";
    return 0;
}
