#include "iswv/iswv.hpp"
#include "iswv/transports/zqwl_transport.hpp"

#include <chrono>
#include <csignal>
#include <iostream>
#include <string>
#include <thread>

namespace {

using namespace std::chrono_literals;

constexpr std::uint8_t kChannel = 0;
constexpr std::uint8_t kNodeId = 1;
constexpr double kMotorRpm = 30.0;
constexpr double kAccelerationRps2 = 10.0;
constexpr auto kRunTime = 5s;

volatile std::sig_atomic_t stop_requested = 0;

void request_stop(int)
{
    stop_requested = 1;
}

void print_error(const char* operation, const iswv::Error& error)
{
    std::cerr << operation << " failed: " << error.message << '\n';
}

void best_effort_stop(iswv::Axis& axis)
{
    // First command zero velocity so profile deceleration can take effect.
    const auto zero = axis.command_motor_velocity_rpm(0.0);
    if (!zero) {
        print_error("zero velocity", zero.error());
    }
    std::this_thread::sleep_for(500ms);

    const auto quick_stop = axis.drive().quick_stop();
    if (!quick_stop) {
        print_error("quick stop", quick_stop.error());
    }
    const auto disabled =
        axis.drive().write_control(iswv::cia402::control::disable_voltage);
    if (!disabled) {
        print_error("disable voltage", disabled.error());
    }
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc != 3 || std::string(argv[2]) != "--run") {
        std::cerr << "usage: iswv_zqwl_slow_rotate_example "
                     "/dev/zqwl-can-... --run\n"
                  << "WARNING: this command enables Node-ID 1 and moves the motor.\n";
        return 2;
    }

    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);

    try {
        auto hub = iswv::transports::ZqwlDeviceHub::open(argv[1]);

        zqwl::ChannelConfig channel;
        channel.channel = kChannel;
        channel.arbitration_bitrate = 500000;
        auto result = hub->configure_channel(channel);
        if (!result) {
            print_error("configure CAN channel", result.error());
            return 1;
        }
        result = hub->set_channel_enabled(kChannel, true);
        if (!result) {
            print_error("enable CAN channel", result.error());
            return 1;
        }

        iswv::canopen::CanopenMaster master(hub->channel(kChannel));
        iswv::Axis axis(master.node(kNodeId),
                        iswv::AxisConfiguration::manual_travel(kNodeId));

        result = axis.node()->send_nmt(
            iswv::canopen::NmtCommand::enter_pre_operational);
        if (!result) {
            print_error("enter pre-operational", result.error());
            return 1;
        }
        result = axis.configure_velocity_mode(false, kAccelerationRps2,
                                              kAccelerationRps2);
        if (!result) {
            print_error("configure velocity mode", result.error());
            return 1;
        }
        result = axis.node()->send_nmt(iswv::canopen::NmtCommand::start);
        if (!result) {
            print_error("start CANopen node", result.error());
            return 1;
        }
        result = axis.drive().enable();
        if (!result) {
            print_error("enable drive", result.error());
            // The transition may have reached an intermediate enabled state before
            // its status read failed, so always send a conservative stop sequence.
            best_effort_stop(axis);
            return 1;
        }

        result = axis.command_motor_velocity_rpm(kMotorRpm);
        if (!result) {
            print_error("command velocity", result.error());
            best_effort_stop(axis);
            return 1;
        }

        std::cout << "Node-ID 1 is rotating at " << kMotorRpm
                  << " motor rpm for up to " << kRunTime.count()
                  << " seconds. Press Ctrl+C to stop.\n";
        const auto deadline = std::chrono::steady_clock::now() + kRunTime;
        while (!stop_requested && std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(50ms);
        }

        best_effort_stop(axis);
        std::cout << "Motor stop sequence completed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "initialization failed: " << error.what() << '\n';
        return 1;
    }
}
