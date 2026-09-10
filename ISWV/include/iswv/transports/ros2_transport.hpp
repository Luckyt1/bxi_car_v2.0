#pragma once

#include "iswv/transport.hpp"

#include <communication/msg/canfd_packet.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace iswv::transports {

struct RosTopicOptions {
    std::string transmit_topic{"canfd_packet/tx"};
    std::string receive_topic{"canfd_packet/rx"};
    rclcpp::QoS qos{100};
};

class RosTopicHub : public std::enable_shared_from_this<RosTopicHub> {
public:
    static std::shared_ptr<RosTopicHub> create(rclcpp::Node& node,
                                               RosTopicOptions options = {});
    ~RosTopicHub();
    RosTopicHub(const RosTopicHub&) = delete;
    RosTopicHub& operator=(const RosTopicHub&) = delete;

    std::shared_ptr<ICanTransport> bus(std::uint8_t bus);
    const RosTopicOptions& options() const noexcept;

private:
    class BusTransport;
    RosTopicHub(rclcpp::Node& node, RosTopicOptions options);
    void initialize();
    Result<void> send(std::uint8_t bus, const CanFrame& frame);
    void receive(const communication::msg::CANFDPacket& packet);

    rclcpp::Node* node_;
    RosTopicOptions options_;
    rclcpp::Subscription<communication::msg::CANFDPacket>::SharedPtr subscription_;
    rclcpp::Publisher<communication::msg::CANFDPacket>::SharedPtr publisher_;
    mutable std::mutex mutex_;
    std::unordered_map<std::uint8_t, std::weak_ptr<BusTransport>> buses_;
};

}  // namespace iswv::transports
