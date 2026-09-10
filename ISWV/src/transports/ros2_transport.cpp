#include "iswv/transports/ros2_transport.hpp"

#include <linux/can.h>

#include <algorithm>
#include <chrono>
#include <stdexcept>
#include <utility>

namespace iswv::transports {

class RosTopicHub::BusTransport final : public ICanTransport {
public:
    BusTransport(std::shared_ptr<RosTopicHub> hub, std::uint8_t bus)
        : hub_(std::move(hub)), bus_(bus)
    {
    }

    Result<void> send(const CanFrame& frame) override { return hub_->send(bus_, frame); }

    void set_receive_handler(ReceiveHandler handler) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        receive_handler_ = std::move(handler);
    }

    void set_error_handler(TransportErrorHandler handler) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        error_handler_ = std::move(handler);
    }

    bool is_open() const noexcept override { return rclcpp::ok(); }

    std::string name() const override
    {
        return "ros2:" + hub_->options().receive_topic + ":bus" + std::to_string(bus_);
    }

    void receive(const CanFrame& frame)
    {
        ReceiveHandler handler;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            handler = receive_handler_;
        }
        if (handler) {
            handler(frame);
        }
    }

private:
    std::shared_ptr<RosTopicHub> hub_;
    const std::uint8_t bus_;
    mutable std::mutex mutex_;
    ReceiveHandler receive_handler_;
    TransportErrorHandler error_handler_;
};

std::shared_ptr<RosTopicHub> RosTopicHub::create(rclcpp::Node& node,
                                                 RosTopicOptions options)
{
    auto hub = std::shared_ptr<RosTopicHub>(
        new RosTopicHub(node, std::move(options)));
    hub->initialize();
    return hub;
}

RosTopicHub::RosTopicHub(rclcpp::Node& node, RosTopicOptions options)
    : node_(&node), options_(std::move(options))
{
    if (options_.transmit_topic.empty() || options_.receive_topic.empty()) {
        throw std::invalid_argument("ROS CAN topic names cannot be empty");
    }
}

void RosTopicHub::initialize()
{
    publisher_ = node_->create_publisher<communication::msg::CANFDPacket>(
        options_.transmit_topic, options_.qos);
    std::weak_ptr<RosTopicHub> weak = shared_from_this();
    subscription_ = node_->create_subscription<communication::msg::CANFDPacket>(
        options_.receive_topic, options_.qos,
        [weak](communication::msg::CANFDPacket::ConstSharedPtr packet) {
            if (packet) {
                if (auto hub = weak.lock()) {
                    hub->receive(*packet);
                }
            }
        });
}

RosTopicHub::~RosTopicHub() = default;

std::shared_ptr<ICanTransport> RosTopicHub::bus(std::uint8_t bus_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = buses_.find(bus_id);
    if (it != buses_.end()) {
        if (auto existing = it->second.lock()) {
            return existing;
        }
    }
    auto created = std::make_shared<BusTransport>(shared_from_this(), bus_id);
    buses_[bus_id] = created;
    return created;
}

const RosTopicOptions& RosTopicHub::options() const noexcept
{
    return options_;
}

Result<void> RosTopicHub::send(std::uint8_t bus_id, const CanFrame& frame)
{
    if (!valid_frame(frame)) {
        return Result<void>::failure(ErrorCode::invalid_argument, "invalid CAN frame");
    }
    if (!rclcpp::ok()) {
        return Result<void>::failure(ErrorCode::transport_closed,
                                     "ROS context is not running");
    }
    communication::msg::CANFDPacket packet;
    packet.header.stamp = node_->now();
    packet.bus = bus_id;
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
    packet.frame.flags = 0;
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
    std::copy_n(frame.data.begin(), frame.size, packet.frame.data.begin());
    try {
        publisher_->publish(std::move(packet));
        return Result<void>::success();
    } catch (const std::exception& error) {
        return Result<void>::failure(ErrorCode::transport_error, error.what());
    }
}

void RosTopicHub::receive(const communication::msg::CANFDPacket& packet)
{
    std::shared_ptr<BusTransport> target;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = buses_.find(packet.bus);
        if (it != buses_.end()) {
            target = it->second.lock();
        }
    }
    if (!target) {
        return;
    }

    CanFrame frame;
    frame.error = (packet.frame.can_id & CAN_ERR_FLAG) != 0U;
    frame.extended = !frame.error && (packet.frame.can_id & CAN_EFF_FLAG) != 0U;
    frame.remote = (packet.frame.can_id & CAN_RTR_FLAG) != 0U;
    frame.id = packet.frame.can_id &
               (frame.error ? CAN_ERR_MASK
                            : (frame.extended ? CAN_EFF_MASK : CAN_SFF_MASK));
    frame.fd = (packet.frame.flags & CANFD_FDF) != 0U;
    frame.bitrate_switch = (packet.frame.flags & CANFD_BRS) != 0U;
    frame.error_state_indicator = (packet.frame.flags & CANFD_ESI) != 0U;
    frame.size = packet.frame.len;
    frame.received_at = std::chrono::steady_clock::now();
    if (!valid_frame(frame)) {
        return;
    }
    std::copy_n(packet.frame.data.begin(), frame.size, frame.data.begin());
    target->receive(frame);
}

}  // namespace iswv::transports
