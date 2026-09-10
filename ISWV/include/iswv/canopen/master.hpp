#pragma once

#include "iswv/canopen/pdo.hpp"
#include "iswv/subscription.hpp"
#include "iswv/transport.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <vector>

namespace iswv::canopen {

namespace detail {
class MasterState;
class NodeState;
}  // namespace detail

class CanopenNode;

struct MasterOptions {
    std::size_t receive_queue_limit{4096};
    std::size_t event_queue_limit{4096};
    std::size_t sdo_queue_limit_per_node{256};
    std::chrono::milliseconds timer_resolution{2};
};

class CanopenMaster {
public:
    explicit CanopenMaster(std::shared_ptr<ICanTransport> transport,
                           MasterOptions options = {});
    ~CanopenMaster();

    CanopenMaster(const CanopenMaster&) = delete;
    CanopenMaster& operator=(const CanopenMaster&) = delete;
    CanopenMaster(CanopenMaster&&) noexcept;
    CanopenMaster& operator=(CanopenMaster&&) noexcept;

    std::shared_ptr<CanopenNode> node(std::uint8_t node_id);
    Result<void> send_nmt(NmtCommand command, std::uint8_t node_id = 0);
    Result<void> send_sync();
    Result<void> start_periodic_sync(std::chrono::microseconds period);
    void stop_periodic_sync() noexcept;
    SyncStatistics sync_statistics() const;
    Result<void> request_node_guard(std::uint8_t node_id);

    Subscription on_raw_frame(std::function<void(const CanFrame&)> callback);
    Subscription on_transport_error(std::function<void(const Error&)> callback);

    MasterStatistics statistics() const;
    void reset_statistics();
    bool is_internal_thread() const noexcept;
    std::shared_ptr<ICanTransport> transport() const;

private:
    std::shared_ptr<detail::MasterState> state_;
};

class CanopenNode : public std::enable_shared_from_this<CanopenNode> {
public:
    using UploadCallback = std::function<void(Result<SdoResponse>)>;
    using DownloadCallback = std::function<void(Result<void>)>;

    ~CanopenNode();
    CanopenNode(const CanopenNode&) = delete;
    CanopenNode& operator=(const CanopenNode&) = delete;

    std::uint8_t id() const noexcept;

    void upload_async(ObjectAddress object,
                      SdoOptions options,
                      UploadCallback callback);
    void download_async(ObjectAddress object,
                        const std::uint8_t* data,
                        std::size_t size,
                        SdoOptions options,
                        DownloadCallback callback);

    std::future<Result<SdoResponse>> upload(ObjectAddress object,
                                            SdoOptions options = {});
    std::future<Result<void>> download(ObjectAddress object,
                                       const std::uint8_t* data,
                                       std::size_t size,
                                       SdoOptions options = {});

    Result<SdoResponse> upload_blocking(ObjectAddress object,
                                        SdoOptions options = {});
    Result<void> download_blocking(ObjectAddress object,
                                   const std::uint8_t* data,
                                   std::size_t size,
                                   SdoOptions options = {});

    template <typename T>
    Result<T> read(Object<T> object, SdoOptions options = {})
    {
        auto response = upload_blocking(object.address, options);
        if (!response) {
            return Result<T>::failure(response.error());
        }
        return decode_little_endian<T>(response.value().data.data(),
                                       response.value().size);
    }

    template <typename T, typename U>
    Result<void> write(Object<T> object, U value, SdoOptions options = {})
    {
        static_assert(std::is_convertible_v<U, T>, "object value is not convertible");
        const auto bytes = encode_little_endian(static_cast<T>(value));
        return download_blocking(object.address, bytes.data(), bytes.size(), options);
    }

    template <typename T>
    void read_async(Object<T> object,
                    SdoOptions options,
                    std::function<void(Result<T>)> callback)
    {
        upload_async(object.address, options,
                     [callback = std::move(callback)](Result<SdoResponse> response) mutable {
                         if (!response) {
                             callback(Result<T>::failure(response.error()));
                             return;
                         }
                         callback(decode_little_endian<T>(response.value().data.data(),
                                                           response.value().size));
                     });
    }

    template <typename T, typename U>
    void write_async(Object<T> object,
                     U value,
                     SdoOptions options,
                     DownloadCallback callback)
    {
        static_assert(std::is_convertible_v<U, T>, "object value is not convertible");
        const auto bytes = encode_little_endian(static_cast<T>(value));
        download_async(object.address, bytes.data(), bytes.size(), options,
                       std::move(callback));
    }

    Result<void> send_nmt(NmtCommand command);
    Result<void> configure_pdo(const PdoConfiguration& configuration,
                               SdoOptions options = {});
    Result<void> disable_pdo(PdoDirection direction,
                             std::uint8_t number,
                             SdoOptions options = {});
    Result<void> send_rpdo(std::uint8_t number,
                           const std::uint8_t* data,
                           std::size_t size);
    Result<void> send_mapped_rpdo(
        std::uint8_t number,
        const std::function<Result<MappedValue>(ObjectAddress)>& value_provider);
    Result<std::vector<MappedValue>> decode_tpdo(const PdoEvent& event) const;
    Result<void> configure_heartbeat(std::chrono::milliseconds producer_time,
                                     std::chrono::milliseconds local_timeout,
                                     bool write_producer_to_device = true,
                                     SdoOptions options = {});
    Result<void> configure_node_guard(std::chrono::milliseconds guard_time,
                                      std::uint8_t life_time_factor,
                                      bool write_configuration_to_device = true,
                                      SdoOptions options = {});
    void stop_node_guard();
    void set_local_heartbeat_timeout(std::chrono::milliseconds timeout);
    NodeNetworkState network_state() const;

    Subscription on_heartbeat(std::function<void(const HeartbeatEvent&)> callback);
    Subscription on_timeout(std::function<void(const NodeTimeoutEvent&)> callback);
    Subscription on_emergency(std::function<void(const EmergencyEvent&)> callback);
    Subscription on_tpdo(std::uint8_t number,
                         std::function<void(const PdoEvent&)> callback);

private:
    friend class CanopenMaster;
    explicit CanopenNode(std::shared_ptr<detail::NodeState> state);
    std::shared_ptr<detail::NodeState> state_;
};

}  // namespace iswv::canopen
