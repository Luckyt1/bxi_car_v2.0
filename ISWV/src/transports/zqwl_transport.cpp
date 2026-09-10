#include "iswv/transports/zqwl_transport.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>
#include <vector>

namespace iswv::transports {
namespace {

Error convert_error(zqwl::ErrorCode code, const std::string& message)
{
    ErrorCode converted = ErrorCode::transport_error;
    switch (code) {
    case zqwl::ErrorCode::success:
        converted = ErrorCode::none;
        break;
    case zqwl::ErrorCode::invalid_argument:
        converted = ErrorCode::invalid_argument;
        break;
    case zqwl::ErrorCode::not_found:
        converted = ErrorCode::not_found;
        break;
    case zqwl::ErrorCode::busy:
        converted = ErrorCode::busy;
        break;
    case zqwl::ErrorCode::timeout:
        converted = ErrorCode::timeout;
        break;
    case zqwl::ErrorCode::unsupported:
        converted = ErrorCode::unsupported;
        break;
    case zqwl::ErrorCode::closed:
        converted = ErrorCode::transport_closed;
        break;
    case zqwl::ErrorCode::permission_denied:
    case zqwl::ErrorCode::io_error:
    case zqwl::ErrorCode::protocol_error:
    case zqwl::ErrorCode::internal_error:
        converted = ErrorCode::transport_error;
        break;
    }
    return make_error(converted, message);
}

}  // namespace

class ZqwlDeviceHub::ChannelTransport final : public ICanTransport {
public:
    ChannelTransport(std::shared_ptr<ZqwlDeviceHub> hub, std::uint8_t channel)
        : hub_(std::move(hub)), channel_(channel)
    {
    }

    Result<void> send(const CanFrame& frame) override
    {
        return hub_->send(channel_, frame);
    }

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

    bool is_open() const noexcept override
    {
        return hub_->device_->is_open();
    }

    std::string name() const override
    {
        return "zqwl:" + hub_->device_->descriptor().path + ":ch" +
               std::to_string(channel_);
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

    void error(const Error& error)
    {
        TransportErrorHandler handler;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            handler = error_handler_;
        }
        if (handler) {
            handler(error);
        }
    }

private:
    std::shared_ptr<ZqwlDeviceHub> hub_;
    const std::uint8_t channel_;
    mutable std::mutex mutex_;
    ReceiveHandler receive_handler_;
    TransportErrorHandler error_handler_;
};

std::shared_ptr<ZqwlDeviceHub> ZqwlDeviceHub::open(
    const std::string& path,
    const zqwl::OpenOptions& options)
{
    auto device = std::make_shared<zqwl::Device>(path, options);
    return attach(std::move(device));
}

std::shared_ptr<ZqwlDeviceHub> ZqwlDeviceHub::attach(
    std::shared_ptr<zqwl::Device> device)
{
    if (!device) {
        throw std::invalid_argument("ZqwlDeviceHub requires a device");
    }
    auto hub = std::shared_ptr<ZqwlDeviceHub>(new ZqwlDeviceHub(std::move(device)));
    hub->initialize();
    return hub;
}

ZqwlDeviceHub::ZqwlDeviceHub(std::shared_ptr<zqwl::Device> device)
    : device_(std::move(device))
{
}

void ZqwlDeviceHub::initialize()
{
    std::weak_ptr<ZqwlDeviceHub> weak = shared_from_this();
    device_->set_frame_callback([weak](const zqwl::Frame& frame) {
        if (auto hub = weak.lock()) {
            hub->receive(frame);
        }
    });
    device_->set_error_callback(
        [weak](zqwl::ErrorCode code, const std::string& message) {
            if (auto hub = weak.lock()) {
                hub->error(code, message);
            }
        });
}

ZqwlDeviceHub::~ZqwlDeviceHub()
{
    if (device_) {
        device_->clear_callbacks();
    }
}

std::shared_ptr<ICanTransport> ZqwlDeviceHub::channel(std::uint8_t channel_id)
{
    if (channel_id >= device_->descriptor().channel_count) {
        throw std::invalid_argument("ZqwlCan channel is outside the device range");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = channels_.find(channel_id);
    if (it != channels_.end()) {
        if (auto existing = it->second.lock()) {
            return existing;
        }
    }
    auto created = std::make_shared<ChannelTransport>(shared_from_this(), channel_id);
    channels_[channel_id] = created;
    return created;
}

Result<void> ZqwlDeviceHub::configure_channel(
    const zqwl::ChannelConfig& configuration)
{
    try {
        device_->configure_channel(configuration);
        return Result<void>::success();
    } catch (const zqwl::Error& error) {
        return Result<void>::failure(convert_error(error.code(), error.what()));
    } catch (const std::exception& error) {
        return Result<void>::failure(ErrorCode::transport_error, error.what());
    }
}

Result<void> ZqwlDeviceHub::set_channel_enabled(std::uint8_t channel_id,
                                                bool enabled,
                                                bool persist_to_flash)
{
    try {
        device_->set_channel_enabled(channel_id, enabled, persist_to_flash);
        return Result<void>::success();
    } catch (const zqwl::Error& error) {
        return Result<void>::failure(convert_error(error.code(), error.what()));
    } catch (const std::exception& error) {
        return Result<void>::failure(ErrorCode::transport_error, error.what());
    }
}

std::shared_ptr<zqwl::Device> ZqwlDeviceHub::device() const noexcept
{
    return device_;
}

Result<void> ZqwlDeviceHub::send(std::uint8_t channel_id, const CanFrame& frame)
{
    if (!valid_frame(frame)) {
        return Result<void>::failure(ErrorCode::invalid_argument, "invalid CAN frame");
    }
    zqwl::Frame converted;
    converted.channel = channel_id;
    converted.id = frame.id;
    converted.format = frame.extended ? zqwl::FrameFormat::extended
                                      : zqwl::FrameFormat::standard;
    converted.type = frame.remote ? zqwl::FrameType::remote : zqwl::FrameType::data;
    converted.protocol = frame.fd ? zqwl::CanProtocol::fd : zqwl::CanProtocol::classic;
    converted.brs = frame.bitrate_switch;
    converted.size = frame.size;
    std::copy_n(frame.data.begin(), frame.size, converted.data.begin());
    try {
        device_->send(converted);
        return Result<void>::success();
    } catch (const zqwl::Error& error) {
        return Result<void>::failure(convert_error(error.code(), error.what()));
    } catch (const std::exception& error) {
        return Result<void>::failure(ErrorCode::transport_error, error.what());
    }
}

void ZqwlDeviceHub::receive(const zqwl::Frame& frame)
{
    std::shared_ptr<ChannelTransport> target;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = channels_.find(frame.channel);
        if (it != channels_.end()) {
            target = it->second.lock();
        }
    }
    if (!target) {
        return;
    }
    if (frame.size > frame.data.size()) {
        target->error(make_error(ErrorCode::protocol_error,
                                 "ZqwlCan delivered an oversized CAN frame"));
        return;
    }
    CanFrame converted;
    converted.id = frame.id;
    converted.extended = frame.format == zqwl::FrameFormat::extended;
    converted.remote = frame.type == zqwl::FrameType::remote;
    converted.fd = frame.protocol == zqwl::CanProtocol::fd;
    converted.bitrate_switch = frame.brs;
    converted.size = frame.size;
    converted.received_at = frame.received_at;
    if (!valid_frame(converted)) {
        target->error(make_error(ErrorCode::protocol_error,
                                 "ZqwlCan delivered an invalid CAN frame"));
        return;
    }
    std::copy_n(frame.data.begin(), frame.size, converted.data.begin());
    target->receive(converted);
}

void ZqwlDeviceHub::error(zqwl::ErrorCode code, const std::string& message)
{
    const auto converted = convert_error(code, message);
    std::vector<std::shared_ptr<ChannelTransport>> targets;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto it = channels_.begin(); it != channels_.end();) {
            if (auto channel = it->second.lock()) {
                targets.push_back(std::move(channel));
                ++it;
            } else {
                it = channels_.erase(it);
            }
        }
    }
    for (const auto& target : targets) {
        target->error(converted);
    }
}

}  // namespace iswv::transports
