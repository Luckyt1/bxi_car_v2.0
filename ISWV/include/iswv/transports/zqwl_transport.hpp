#pragma once

#include "iswv/transport.hpp"

#include <zqwl/can.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace iswv::transports {

class ZqwlDeviceHub : public std::enable_shared_from_this<ZqwlDeviceHub> {
public:
    static std::shared_ptr<ZqwlDeviceHub> open(
        const std::string& path,
        const zqwl::OpenOptions& options = {});
    static std::shared_ptr<ZqwlDeviceHub> attach(
        std::shared_ptr<zqwl::Device> device);

    ~ZqwlDeviceHub();
    ZqwlDeviceHub(const ZqwlDeviceHub&) = delete;
    ZqwlDeviceHub& operator=(const ZqwlDeviceHub&) = delete;

    std::shared_ptr<ICanTransport> channel(std::uint8_t channel);
    Result<void> configure_channel(const zqwl::ChannelConfig& configuration);
    Result<void> set_channel_enabled(std::uint8_t channel,
                                     bool enabled,
                                     bool persist_to_flash = false);
    std::shared_ptr<zqwl::Device> device() const noexcept;

private:
    class ChannelTransport;
    explicit ZqwlDeviceHub(std::shared_ptr<zqwl::Device> device);
    void initialize();
    Result<void> send(std::uint8_t channel, const CanFrame& frame);
    void receive(const zqwl::Frame& frame);
    void error(zqwl::ErrorCode code, const std::string& message);

    std::shared_ptr<zqwl::Device> device_;
    mutable std::mutex mutex_;
    std::unordered_map<std::uint8_t, std::weak_ptr<ChannelTransport>> channels_;
};

}  // namespace iswv::transports
