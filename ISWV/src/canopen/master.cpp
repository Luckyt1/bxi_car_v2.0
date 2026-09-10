#include "iswv/canopen/master.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <map>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>

namespace iswv::canopen {
namespace detail {

class MasterState;

class NodeState {
public:
    NodeState(std::weak_ptr<MasterState> master_in, std::uint8_t id_in)
        : master(std::move(master_in)), id(id_in)
    {
        network.node_id = id;
    }

    std::weak_ptr<MasterState> master;
    const std::uint8_t id;
    mutable std::mutex mutex;
    NodeNetworkState network;
    std::chrono::milliseconds heartbeat_timeout{0};
    bool timeout_reported{false};
    bool supervision_armed{false};
    bool node_guard_active{false};
    std::chrono::milliseconds node_guard_period{0};
    std::chrono::steady_clock::time_point next_node_guard{};
    std::array<std::optional<PdoConfiguration>, 4> rpdo;
    std::array<std::optional<PdoConfiguration>, 4> tpdo;
};

namespace {

thread_local const MasterState* current_internal_master = nullptr;

struct InternalThreadScope {
    explicit InternalThreadScope(const MasterState* state)
        : previous(current_internal_master)
    {
        current_internal_master = state;
    }
    ~InternalThreadScope() { current_internal_master = previous; }
    const MasterState* previous;
};

struct EventJob {
    bool critical{false};
    std::function<void()> function;
};

enum class SdoKind : std::uint8_t { upload, download };

struct SdoTransaction {
    std::uint64_t sequence{0};
    std::uint8_t node_id{0};
    ObjectAddress object;
    SdoKind kind{SdoKind::upload};
    CanFrame request;
    std::chrono::milliseconds timeout{250};
    std::chrono::steady_clock::time_point deadline{};
    bool active{false};
    std::atomic<bool> completed{false};
    std::function<void(Result<SdoResponse>)> completion;
};

template <typename Callback>
struct NodeCallback {
    std::uint8_t node_id{0};
    std::uint8_t pdo_number{0};
    Callback callback;
};

template <typename Callback, typename Event>
void invoke_callbacks(const std::vector<Callback>& callbacks,
                      const Event& event) noexcept
{
    for (const auto& callback : callbacks) {
        try {
            callback(event);
        } catch (...) {
            // One observer must not suppress other observers or safety handlers.
        }
    }
}

NmtState decode_nmt_state(std::uint8_t value)
{
    switch (value) {
    case 0x00:
        return NmtState::initializing;
    case 0x04:
        return NmtState::stopped;
    case 0x05:
        return NmtState::operational;
    case 0x7F:
        return NmtState::pre_operational;
    default:
        return NmtState::unknown;
    }
}

}  // namespace

class MasterState : public std::enable_shared_from_this<MasterState> {
public:
    MasterState(std::shared_ptr<ICanTransport> transport_in, MasterOptions options_in)
        : transport_(std::move(transport_in)), options_(options_in)
    {
        if (!transport_) {
            throw std::invalid_argument("CANopen master requires a transport");
        }
        if (options_.receive_queue_limit == 0 || options_.event_queue_limit == 0 ||
            options_.sdo_queue_limit_per_node == 0 ||
            options_.timer_resolution.count() <= 0) {
            throw std::invalid_argument("CANopen master queue sizes and timer must be positive");
        }
    }

    ~MasterState() { stop(); }

    void start()
    {
        try {
            std::weak_ptr<MasterState> weak = shared_from_this();
            transport_->set_receive_handler([weak](const CanFrame& frame) {
                if (auto self = weak.lock()) {
                    self->receive(frame);
                }
            });
            transport_->set_error_handler([weak](const Error& error) {
                if (auto self = weak.lock()) {
                    self->transport_error(error);
                }
            });
            protocol_thread_ =
                std::thread([self = shared_from_this()] { self->protocol_loop(); });
            event_thread_ =
                std::thread([self = shared_from_this()] { self->event_loop(); });
        } catch (...) {
            stop();
            throw;
        }
    }

    void stop() noexcept
    {
        bool expected = false;
        if (!stopping_.compare_exchange_strong(expected, true)) {
            return;
        }

        stop_periodic_sync();
        try {
            transport_->set_receive_handler({});
        } catch (...) {
        }
        try {
            transport_->set_error_handler({});
        } catch (...) {
        }
        receive_cv_.notify_all();
        if (protocol_thread_.joinable()) {
            if (protocol_thread_.get_id() == std::this_thread::get_id()) {
                protocol_thread_.detach();
            } else {
                protocol_thread_.join();
            }
        }

        std::vector<std::shared_ptr<SdoTransaction>> cancelled;
        {
            std::lock_guard<std::mutex> lock(sdo_mutex_);
            for (auto& item : sdo_queues_) {
                for (auto& transaction : item.second) {
                    cancelled.push_back(transaction);
                }
            }
            sdo_queues_.clear();
        }
        for (const auto& transaction : cancelled) {
            complete_sdo(transaction,
                         Result<SdoResponse>::failure(ErrorCode::cancelled,
                                                      "CANopen master is stopping"));
        }

        {
            std::lock_guard<std::mutex> lock(event_mutex_);
            event_stopping_ = true;
        }
        event_cv_.notify_all();
        if (event_thread_.joinable()) {
            if (event_thread_.get_id() == std::this_thread::get_id()) {
                event_thread_.detach();
            } else {
                event_thread_.join();
            }
        }
    }

    std::shared_ptr<NodeState> node(std::uint8_t id)
    {
        std::lock_guard<std::mutex> lock(nodes_mutex_);
        auto it = nodes_.find(id);
        if (it != nodes_.end()) {
            if (auto existing = it->second.lock()) {
                return existing;
            }
        }
        auto created = std::make_shared<NodeState>(weak_from_this(), id);
        nodes_[id] = created;
        return created;
    }

    Result<void> send_frame(const CanFrame& frame)
    {
        if (stopping_.load()) {
            return Result<void>::failure(ErrorCode::transport_closed,
                                         "CANopen master is stopped");
        }
        try {
            auto result = transport_->send(frame);
            if (result) {
                std::lock_guard<std::mutex> lock(statistics_mutex_);
                ++statistics_.transmitted_frames;
            }
            return result;
        } catch (const std::exception& error) {
            return Result<void>::failure(ErrorCode::transport_error, error.what());
        } catch (...) {
            return Result<void>::failure(ErrorCode::transport_error,
                                         "transport send threw an unknown exception");
        }
    }

    void enqueue_upload(std::uint8_t node_id,
                        ObjectAddress object,
                        SdoOptions options,
                        std::function<void(Result<SdoResponse>)> completion)
    {
        if (options.timeout.count() <= 0) {
            enqueue_event(true, [completion = std::move(completion)]() mutable {
                completion(Result<SdoResponse>::failure(
                    ErrorCode::invalid_argument, "SDO timeout must be positive"));
            });
            return;
        }

        auto transaction = std::make_shared<SdoTransaction>();
        transaction->sequence = next_sdo_sequence_.fetch_add(1);
        transaction->node_id = node_id;
        transaction->object = object;
        transaction->kind = SdoKind::upload;
        transaction->timeout = options.timeout;
        transaction->completion = std::move(completion);
        transaction->request.id = 0x600U + node_id;
        transaction->request.size = 8;
        transaction->request.data[0] = 0x40;
        transaction->request.data[1] = static_cast<std::uint8_t>(object.index & 0xFFU);
        transaction->request.data[2] = static_cast<std::uint8_t>(object.index >> 8U);
        transaction->request.data[3] = object.subindex;
        enqueue_sdo(std::move(transaction));
    }

    void enqueue_download(std::uint8_t node_id,
                          ObjectAddress object,
                          const std::uint8_t* data,
                          std::size_t size,
                          SdoOptions options,
                          std::function<void(Result<SdoResponse>)> completion)
    {
        if ((size != 1 && size != 2 && size != 4) || data == nullptr) {
            enqueue_event(true, [completion = std::move(completion)]() mutable {
                completion(Result<SdoResponse>::failure(
                    ErrorCode::invalid_argument,
                    "expedited SDO download size must be 1, 2, or 4 bytes"));
            });
            return;
        }
        if (options.timeout.count() <= 0) {
            enqueue_event(true, [completion = std::move(completion)]() mutable {
                completion(Result<SdoResponse>::failure(
                    ErrorCode::invalid_argument, "SDO timeout must be positive"));
            });
            return;
        }

        auto transaction = std::make_shared<SdoTransaction>();
        transaction->sequence = next_sdo_sequence_.fetch_add(1);
        transaction->node_id = node_id;
        transaction->object = object;
        transaction->kind = SdoKind::download;
        transaction->timeout = options.timeout;
        transaction->completion = std::move(completion);
        transaction->request.id = 0x600U + node_id;
        transaction->request.size = 8;
        transaction->request.data[0] = size == 1 ? 0x2F : (size == 2 ? 0x2B : 0x23);
        transaction->request.data[1] = static_cast<std::uint8_t>(object.index & 0xFFU);
        transaction->request.data[2] = static_cast<std::uint8_t>(object.index >> 8U);
        transaction->request.data[3] = object.subindex;
        std::copy_n(data, size, transaction->request.data.begin() + 4);
        enqueue_sdo(std::move(transaction));
    }

    bool is_internal_thread() const noexcept
    {
        return current_internal_master == this;
    }

    Subscription add_raw_callback(std::function<void(const CanFrame&)> callback)
    {
        if (!callback) {
            return {};
        }
        const auto token = add_callback(raw_callbacks_, std::move(callback));
        return cancellation(token, CallbackType::raw);
    }

    Subscription add_error_callback(std::function<void(const Error&)> callback)
    {
        if (!callback) {
            return {};
        }
        const auto token = add_callback(error_callbacks_, std::move(callback));
        return cancellation(token, CallbackType::transport_error);
    }

    Subscription add_heartbeat_callback(
        std::uint8_t node_id, std::function<void(const HeartbeatEvent&)> callback)
    {
        return add_node_callback(heartbeat_callbacks_, node_id, 0, std::move(callback),
                                 CallbackType::heartbeat);
    }

    Subscription add_timeout_callback(
        std::uint8_t node_id, std::function<void(const NodeTimeoutEvent&)> callback)
    {
        return add_node_callback(timeout_callbacks_, node_id, 0, std::move(callback),
                                 CallbackType::timeout);
    }

    Subscription add_emergency_callback(
        std::uint8_t node_id, std::function<void(const EmergencyEvent&)> callback)
    {
        return add_node_callback(emergency_callbacks_, node_id, 0, std::move(callback),
                                 CallbackType::emergency);
    }

    Subscription add_pdo_callback(std::uint8_t node_id,
                                  std::uint8_t number,
                                  std::function<void(const PdoEvent&)> callback)
    {
        return add_node_callback(pdo_callbacks_, node_id, number, std::move(callback),
                                 CallbackType::pdo);
    }

    void register_tpdo(std::uint8_t node_id, const PdoConfiguration& configuration)
    {
        std::lock_guard<std::mutex> lock(routes_mutex_);
        for (auto it = custom_tpdo_routes_.begin(); it != custom_tpdo_routes_.end();) {
            if (it->second.first == node_id &&
                it->second.second == configuration.number) {
                it = custom_tpdo_routes_.erase(it);
            } else {
                ++it;
            }
        }
        if (configuration.enabled) {
            custom_tpdo_routes_[configuration.cob_id] =
                std::make_pair(node_id, configuration.number);
        }
    }

    MasterStatistics statistics() const
    {
        std::lock_guard<std::mutex> lock(statistics_mutex_);
        return statistics_;
    }

    void reset_statistics()
    {
        std::lock_guard<std::mutex> lock(statistics_mutex_);
        statistics_ = {};
    }

    std::shared_ptr<ICanTransport> transport() const { return transport_; }

    Result<void> start_periodic_sync(std::chrono::microseconds period)
    {
        if (period.count() <= 0) {
            return Result<void>::failure(ErrorCode::invalid_argument,
                                         "SYNC period must be positive");
        }
        stop_periodic_sync();
        {
            std::lock_guard<std::mutex> lock(sync_mutex_);
            sync_stop_ = false;
            sync_statistics_ = {};
            sync_statistics_.running = true;
            sync_statistics_.period = period;
        }
        sync_thread_ = std::thread([self = shared_from_this(), period] {
            auto deadline = std::chrono::steady_clock::now() + period;
            while (true) {
                {
                    std::unique_lock<std::mutex> lock(self->sync_mutex_);
                    if (self->sync_cv_.wait_until(lock, deadline,
                                                  [self] { return self->sync_stop_; })) {
                        break;
                    }
                }
                const auto now = std::chrono::steady_clock::now();
                const auto lateness = now > deadline
                                          ? std::chrono::duration_cast<std::chrono::microseconds>(
                                                now - deadline)
                                          : std::chrono::microseconds{0};
                CanFrame frame;
                frame.id = 0x080;
                auto result = self->send_frame(frame);
                {
                    std::lock_guard<std::mutex> lock(self->sync_mutex_);
                    if (result) {
                        ++self->sync_statistics_.frames_sent;
                    } else {
                        ++self->sync_statistics_.send_errors;
                    }
                    if (lateness >= period) {
                        ++self->sync_statistics_.deadline_misses;
                    }
                    self->sync_statistics_.maximum_lateness =
                        std::max(self->sync_statistics_.maximum_lateness, lateness);
                }
                do {
                    deadline += period;
                } while (deadline <= now);
            }
            std::lock_guard<std::mutex> lock(self->sync_mutex_);
            self->sync_statistics_.running = false;
        });
        return Result<void>::success();
    }

    void stop_periodic_sync() noexcept
    {
        {
            std::lock_guard<std::mutex> lock(sync_mutex_);
            sync_stop_ = true;
        }
        sync_cv_.notify_all();
        if (sync_thread_.joinable() && sync_thread_.get_id() != std::this_thread::get_id()) {
            sync_thread_.join();
        } else if (sync_thread_.joinable()) {
            sync_thread_.detach();
        }
        std::lock_guard<std::mutex> lock(sync_mutex_);
        sync_statistics_.running = false;
    }

    SyncStatistics sync_statistics() const
    {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        return sync_statistics_;
    }

    void mark_node_guard_request(std::uint8_t node_id)
    {
        std::lock_guard<std::mutex> lock(guard_mutex_);
        ++pending_guard_requests_[node_id];
    }

    void unmark_node_guard_request(std::uint8_t node_id)
    {
        std::lock_guard<std::mutex> lock(guard_mutex_);
        auto it = pending_guard_requests_.find(node_id);
        if (it == pending_guard_requests_.end()) {
            return;
        }
        if (--it->second == 0) {
            pending_guard_requests_.erase(it);
        }
    }

    void clear_node_guard_requests(std::uint8_t node_id)
    {
        std::lock_guard<std::mutex> lock(guard_mutex_);
        pending_guard_requests_.erase(node_id);
    }

private:
    enum class CallbackType : std::uint8_t {
        raw,
        transport_error,
        heartbeat,
        timeout,
        emergency,
        pdo
    };

    template <typename Callback>
    std::uint64_t add_callback(std::unordered_map<std::uint64_t, Callback>& callbacks,
                               Callback callback)
    {
        const auto token = next_callback_token_.fetch_add(1);
        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        callbacks.emplace(token, std::move(callback));
        return token;
    }

    template <typename Callback>
    Subscription add_node_callback(
        std::unordered_map<std::uint64_t, NodeCallback<Callback>>& callbacks,
        std::uint8_t node_id,
        std::uint8_t pdo_number,
        Callback callback,
        CallbackType type)
    {
        if (!callback) {
            return {};
        }
        const auto token = next_callback_token_.fetch_add(1);
        {
            std::lock_guard<std::mutex> lock(callbacks_mutex_);
            callbacks.emplace(token,
                              NodeCallback<Callback>{node_id, pdo_number,
                                                     std::move(callback)});
        }
        return cancellation(token, type);
    }

    Subscription cancellation(std::uint64_t token, CallbackType type)
    {
        std::weak_ptr<MasterState> weak = shared_from_this();
        return Subscription([weak, token, type] {
            if (auto state = weak.lock()) {
                state->remove_callback(token, type);
            }
        });
    }

    void remove_callback(std::uint64_t token, CallbackType type)
    {
        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        switch (type) {
        case CallbackType::raw:
            raw_callbacks_.erase(token);
            break;
        case CallbackType::transport_error:
            error_callbacks_.erase(token);
            break;
        case CallbackType::heartbeat:
            heartbeat_callbacks_.erase(token);
            break;
        case CallbackType::timeout:
            timeout_callbacks_.erase(token);
            break;
        case CallbackType::emergency:
            emergency_callbacks_.erase(token);
            break;
        case CallbackType::pdo:
            pdo_callbacks_.erase(token);
            break;
        }
    }

    void receive(const CanFrame& input)
    {
        auto frame = input;
        if (frame.received_at == std::chrono::steady_clock::time_point{}) {
            frame.received_at = std::chrono::steady_clock::now();
        }
        {
            std::lock_guard<std::mutex> lock(receive_mutex_);
            if (receive_queue_.size() >= options_.receive_queue_limit) {
                receive_queue_.pop_front();
                std::lock_guard<std::mutex> stats_lock(statistics_mutex_);
                ++statistics_.receive_queue_drops;
            }
            receive_queue_.push_back(frame);
        }
        receive_cv_.notify_one();
    }

    void transport_error(Error error)
    {
        std::vector<std::function<void(const Error&)>> callbacks;
        {
            std::lock_guard<std::mutex> lock(callbacks_mutex_);
            for (const auto& entry : error_callbacks_) {
                callbacks.push_back(entry.second);
            }
        }
        enqueue_event(false, [error = std::move(error), callbacks = std::move(callbacks)] {
            invoke_callbacks(callbacks, error);
        });
    }

    void protocol_loop()
    {
        InternalThreadScope scope(this);
        while (!stopping_.load()) {
            std::deque<CanFrame> frames;
            {
                std::unique_lock<std::mutex> lock(receive_mutex_);
                receive_cv_.wait_for(lock, options_.timer_resolution, [this] {
                    return stopping_.load() || !receive_queue_.empty();
                });
                frames.swap(receive_queue_);
            }
            for (const auto& frame : frames) {
                process_frame(frame);
            }
            check_sdo_timeouts();
            check_node_guards();
            check_node_timeouts();
        }
    }

    void event_loop()
    {
        InternalThreadScope scope(this);
        while (true) {
            EventJob job;
            {
                std::unique_lock<std::mutex> lock(event_mutex_);
                event_cv_.wait(lock, [this] {
                    return event_stopping_ || !event_queue_.empty();
                });
                if (event_queue_.empty() && event_stopping_) {
                    return;
                }
                job = std::move(event_queue_.front());
                event_queue_.pop_front();
            }
            try {
                if (job.function) {
                    job.function();
                }
            } catch (...) {
                // User callbacks must not terminate the dispatcher.
            }
        }
    }

    void enqueue_event(bool critical, std::function<void()> function)
    {
        bool run_inline = false;
        {
            std::lock_guard<std::mutex> lock(event_mutex_);
            if (event_stopping_) {
                run_inline = critical && static_cast<bool>(function);
            } else {
                if (event_queue_.size() >= options_.event_queue_limit) {
                    auto discard = std::find_if(event_queue_.begin(), event_queue_.end(),
                                                [](const EventJob& job) {
                                                    return !job.critical;
                                                });
                    if (discard != event_queue_.end()) {
                        event_queue_.erase(discard);
                        std::lock_guard<std::mutex> stats_lock(statistics_mutex_);
                        ++statistics_.event_queue_drops;
                    } else if (!critical) {
                        std::lock_guard<std::mutex> stats_lock(statistics_mutex_);
                        ++statistics_.event_queue_drops;
                        return;
                    }
                }
                event_queue_.push_back(EventJob{critical, std::move(function)});
            }
        }
        if (run_inline) {
            try {
                function();
            } catch (...) {
                // Critical completions may run inline during shutdown. User code must
                // not escape a noexcept stop/destructor path.
            }
            return;
        }
        event_cv_.notify_one();
    }

    void process_frame(const CanFrame& frame)
    {
        {
            std::lock_guard<std::mutex> lock(statistics_mutex_);
            ++statistics_.received_frames;
        }
        if (!valid_frame(frame)) {
            std::lock_guard<std::mutex> lock(statistics_mutex_);
            ++statistics_.invalid_frames;
            return;
        }
        if (frame.fd || frame.error) {
            // CANopen uses classic data/remote frames. Preserve other valid bus
            // traffic for raw observers without feeding it into the protocol.
            emit_raw(frame);
            return;
        }

        bool handled = false;
        if (!frame.extended && !frame.remote && frame.id >= 0x580U &&
            frame.id <= 0x5FFU) {
            handled = process_sdo(frame);
        } else if (!frame.extended && !frame.remote && frame.id >= 0x700U &&
                   frame.id <= 0x77FU && frame.size >= 1) {
            bool node_guard = false;
            {
                std::lock_guard<std::mutex> lock(guard_mutex_);
                const auto node_id = static_cast<std::uint8_t>(frame.id - 0x700U);
                auto it = pending_guard_requests_.find(node_id);
                if (it != pending_guard_requests_.end()) {
                    node_guard = true;
                    if (--it->second == 0) {
                        pending_guard_requests_.erase(it);
                    }
                }
            }
            process_heartbeat(frame, node_guard);
            handled = true;
        } else if (!frame.extended && !frame.remote && frame.id >= 0x081U &&
                   frame.id <= 0x0FFU && frame.size == 8) {
            process_emergency(frame);
            handled = true;
        }

        if (!handled) {
            handled = process_tpdo(frame);
        }
        (void)handled;
        emit_raw(frame);
    }

    bool process_sdo(const CanFrame& frame)
    {
        if (frame.size != 8) {
            return false;
        }
        const auto node_id = static_cast<std::uint8_t>(frame.id - 0x580U);
        std::shared_ptr<SdoTransaction> transaction;
        {
            std::lock_guard<std::mutex> lock(sdo_mutex_);
            auto it = sdo_queues_.find(node_id);
            if (it == sdo_queues_.end() || it->second.empty() ||
                !it->second.front()->active) {
                return false;
            }
            transaction = it->second.front();
        }

        const ObjectAddress object{
            static_cast<std::uint16_t>(frame.data[1] |
                                       (static_cast<std::uint16_t>(frame.data[2]) << 8U)),
            frame.data[3]};
        if (object != transaction->object) {
            return false;
        }

        Result<SdoResponse> result = Result<SdoResponse>::failure(
            ErrorCode::protocol_error, "unrecognized SDO response");
        if (frame.data[0] == 0x80) {
            const std::uint32_t abort =
                static_cast<std::uint32_t>(frame.data[4]) |
                (static_cast<std::uint32_t>(frame.data[5]) << 8U) |
                (static_cast<std::uint32_t>(frame.data[6]) << 16U) |
                (static_cast<std::uint32_t>(frame.data[7]) << 24U);
            result = Result<SdoResponse>::failure(
                make_sdo_abort(abort, "SDO abort for " + object_name(object)));
            std::lock_guard<std::mutex> lock(statistics_mutex_);
            ++statistics_.sdo_aborts;
        } else if (transaction->kind == SdoKind::download && frame.data[0] == 0x60) {
            SdoResponse response;
            response.object = object;
            result = Result<SdoResponse>::success(response);
        } else if (transaction->kind == SdoKind::upload) {
            std::uint8_t size = 0;
            if (frame.data[0] == 0x4F) {
                size = 1;
            } else if (frame.data[0] == 0x4B) {
                size = 2;
            } else if (frame.data[0] == 0x43) {
                size = 4;
            }
            if (size != 0) {
                SdoResponse response;
                response.object = object;
                response.size = size;
                std::copy_n(frame.data.begin() + 4, size, response.data.begin());
                result = Result<SdoResponse>::success(response);
            }
        }

        pop_sdo(transaction);
        complete_sdo(transaction, std::move(result));
        activate_next_sdo(node_id);
        return true;
    }

    void process_heartbeat(const CanFrame& frame, bool forced_guard)
    {
        const auto node_id = static_cast<std::uint8_t>(frame.id - 0x700U);
        const bool toggle = (frame.data[0] & 0x80U) != 0;
        const auto raw_state = static_cast<std::uint8_t>(frame.data[0] & 0x7FU);
        HeartbeatEvent event;
        event.node_id = node_id;
        event.state = decode_nmt_state(raw_state);
        event.boot_up = raw_state == 0;
        event.node_guard = forced_guard || toggle;
        event.toggle = toggle;
        event.received_at = frame.received_at;

        auto node_state = node(node_id);
        {
            std::lock_guard<std::mutex> lock(node_state->mutex);
            node_state->network.nmt_state = event.state;
            node_state->network.online = true;
            node_state->network.last_seen = event.received_at;
            node_state->timeout_reported = false;
            node_state->supervision_armed = node_state->heartbeat_timeout.count() > 0;
        }
        {
            std::lock_guard<std::mutex> lock(statistics_mutex_);
            ++statistics_.heartbeat_frames;
        }

        std::vector<std::function<void(const HeartbeatEvent&)>> callbacks;
        {
            std::lock_guard<std::mutex> lock(callbacks_mutex_);
            for (const auto& entry : heartbeat_callbacks_) {
                if (entry.second.node_id == node_id) {
                    callbacks.push_back(entry.second.callback);
                }
            }
        }
        enqueue_event(false, [event, callbacks = std::move(callbacks)] {
            invoke_callbacks(callbacks, event);
        });
    }

    void process_emergency(const CanFrame& frame)
    {
        EmergencyEvent event;
        event.node_id = static_cast<std::uint8_t>(frame.id - 0x080U);
        event.error_code = static_cast<std::uint16_t>(
            frame.data[0] | (static_cast<std::uint16_t>(frame.data[1]) << 8U));
        event.error_register = frame.data[2];
        std::copy_n(frame.data.begin() + 3, 5, event.manufacturer_data.begin());
        event.received_at = frame.received_at;
        {
            std::lock_guard<std::mutex> lock(statistics_mutex_);
            ++statistics_.emergency_frames;
        }

        std::vector<std::function<void(const EmergencyEvent&)>> callbacks;
        {
            std::lock_guard<std::mutex> lock(callbacks_mutex_);
            for (const auto& entry : emergency_callbacks_) {
                if (entry.second.node_id == event.node_id) {
                    callbacks.push_back(entry.second.callback);
                }
            }
        }
        enqueue_event(false, [event, callbacks = std::move(callbacks)] {
            invoke_callbacks(callbacks, event);
        });
    }

    bool process_tpdo(const CanFrame& frame)
    {
        if (frame.remote || frame.size > 8) {
            return false;
        }
        std::uint8_t node_id = 0;
        std::uint8_t number = 0;
        {
            std::lock_guard<std::mutex> lock(routes_mutex_);
            auto custom = custom_tpdo_routes_.find(frame.id);
            if (custom != custom_tpdo_routes_.end()) {
                node_id = custom->second.first;
                number = custom->second.second;
            }
        }
        if (number == 0 && !frame.extended) {
            constexpr std::array<std::uint32_t, 4> bases{0x180U, 0x280U, 0x380U,
                                                        0x480U};
            for (std::size_t i = 0; i < bases.size(); ++i) {
                if (frame.id > bases[i] && frame.id <= bases[i] + 0x7FU) {
                    node_id = static_cast<std::uint8_t>(frame.id - bases[i]);
                    number = static_cast<std::uint8_t>(i + 1U);
                    break;
                }
            }
        }
        if (number == 0) {
            return false;
        }

        PdoEvent event;
        event.node_id = node_id;
        event.number = number;
        event.cob_id = frame.id;
        event.size = frame.size;
        std::copy_n(frame.data.begin(), frame.size, event.data.begin());
        event.received_at = frame.received_at;
        {
            std::lock_guard<std::mutex> lock(statistics_mutex_);
            ++statistics_.pdo_frames;
        }

        std::vector<std::function<void(const PdoEvent&)>> callbacks;
        {
            std::lock_guard<std::mutex> lock(callbacks_mutex_);
            for (const auto& entry : pdo_callbacks_) {
                if (entry.second.node_id == node_id &&
                    entry.second.pdo_number == number) {
                    callbacks.push_back(entry.second.callback);
                }
            }
        }
        enqueue_event(false, [event, callbacks = std::move(callbacks)] {
            invoke_callbacks(callbacks, event);
        });
        return true;
    }

    void emit_raw(const CanFrame& frame)
    {
        std::vector<std::function<void(const CanFrame&)>> callbacks;
        {
            std::lock_guard<std::mutex> lock(callbacks_mutex_);
            for (const auto& entry : raw_callbacks_) {
                callbacks.push_back(entry.second);
            }
        }
        if (!callbacks.empty()) {
            enqueue_event(false, [frame, callbacks = std::move(callbacks)] {
                invoke_callbacks(callbacks, frame);
            });
        }
    }

    void enqueue_sdo(std::shared_ptr<SdoTransaction> transaction)
    {
        const auto node_id = transaction->node_id;
        bool activate = false;
        bool overflow = false;
        {
            std::lock_guard<std::mutex> lock(sdo_mutex_);
            auto& queue = sdo_queues_[node_id];
            if (queue.size() >= options_.sdo_queue_limit_per_node) {
                overflow = true;
            } else {
                activate = queue.empty();
                queue.push_back(transaction);
            }
        }
        if (overflow) {
            complete_sdo(transaction,
                         Result<SdoResponse>::failure(
                             ErrorCode::queue_overflow,
                             "per-node SDO transaction queue is full"));
            return;
        }
        if (activate) {
            activate_next_sdo(node_id);
        }
    }

    void activate_next_sdo(std::uint8_t node_id)
    {
        while (true) {
            std::shared_ptr<SdoTransaction> transaction;
            {
                std::lock_guard<std::mutex> lock(sdo_mutex_);
                auto it = sdo_queues_.find(node_id);
                if (it == sdo_queues_.end() || it->second.empty() ||
                    it->second.front()->active) {
                    return;
                }
                transaction = it->second.front();
                transaction->active = true;
                transaction->deadline = std::chrono::steady_clock::now() +
                                        transaction->timeout;
            }
            receive_cv_.notify_one();
            auto sent = send_frame(transaction->request);
            if (sent) {
                return;
            }
            pop_sdo(transaction);
            complete_sdo(transaction,
                         Result<SdoResponse>::failure(sent.error()));
        }
    }

    void pop_sdo(const std::shared_ptr<SdoTransaction>& transaction)
    {
        std::lock_guard<std::mutex> lock(sdo_mutex_);
        auto it = sdo_queues_.find(transaction->node_id);
        if (it == sdo_queues_.end() || it->second.empty()) {
            return;
        }
        if (it->second.front()->sequence == transaction->sequence) {
            it->second.pop_front();
        }
        if (it->second.empty()) {
            sdo_queues_.erase(it);
        }
    }

    void complete_sdo(const std::shared_ptr<SdoTransaction>& transaction,
                      Result<SdoResponse> result)
    {
        bool expected = false;
        if (!transaction->completed.compare_exchange_strong(expected, true)) {
            return;
        }
        enqueue_event(true,
                      [transaction, result = std::move(result)]() mutable {
                          transaction->completion(std::move(result));
                      });
    }

    void check_sdo_timeouts()
    {
        const auto now = std::chrono::steady_clock::now();
        std::vector<std::shared_ptr<SdoTransaction>> expired;
        std::vector<std::uint8_t> nodes;
        {
            std::lock_guard<std::mutex> lock(sdo_mutex_);
            for (auto it = sdo_queues_.begin(); it != sdo_queues_.end();) {
                if (!it->second.empty() && it->second.front()->active &&
                    it->second.front()->deadline <= now) {
                    expired.push_back(it->second.front());
                    nodes.push_back(it->first);
                    it->second.pop_front();
                }
                if (it->second.empty()) {
                    it = sdo_queues_.erase(it);
                } else {
                    ++it;
                }
            }
        }
        for (const auto& transaction : expired) {
            {
                std::lock_guard<std::mutex> lock(statistics_mutex_);
                ++statistics_.sdo_timeouts;
            }
            complete_sdo(transaction,
                         Result<SdoResponse>::failure(
                             ErrorCode::timeout,
                             "SDO timeout for " + object_name(transaction->object)));
        }
        for (const auto node_id : nodes) {
            activate_next_sdo(node_id);
        }
    }

    void check_node_timeouts()
    {
        const auto now = std::chrono::steady_clock::now();
        std::vector<std::shared_ptr<NodeState>> nodes;
        {
            std::lock_guard<std::mutex> lock(nodes_mutex_);
            for (auto it = nodes_.begin(); it != nodes_.end();) {
                if (auto state = it->second.lock()) {
                    nodes.push_back(std::move(state));
                    ++it;
                } else {
                    it = nodes_.erase(it);
                }
            }
        }
        for (const auto& node_state : nodes) {
            NodeTimeoutEvent event;
            bool timed_out = false;
            {
                std::lock_guard<std::mutex> lock(node_state->mutex);
                if (node_state->heartbeat_timeout.count() > 0 &&
                    node_state->supervision_armed &&
                    now - node_state->network.last_seen > node_state->heartbeat_timeout &&
                    !node_state->timeout_reported) {
                    node_state->network.online = false;
                    node_state->timeout_reported = true;
                    event.node_id = node_state->id;
                    event.timeout = node_state->heartbeat_timeout;
                    event.last_seen = node_state->network.last_seen;
                    timed_out = true;
                }
            }
            if (!timed_out) {
                continue;
            }

            std::vector<std::function<void(const NodeTimeoutEvent&)>> callbacks;
            {
                std::lock_guard<std::mutex> lock(callbacks_mutex_);
                for (const auto& entry : timeout_callbacks_) {
                    if (entry.second.node_id == event.node_id) {
                        callbacks.push_back(entry.second.callback);
                    }
                }
            }
            enqueue_event(false, [event, callbacks = std::move(callbacks)] {
                invoke_callbacks(callbacks, event);
            });
        }
    }

    void check_node_guards()
    {
        const auto now = std::chrono::steady_clock::now();
        std::vector<std::shared_ptr<NodeState>> due;
        {
            std::lock_guard<std::mutex> lock(nodes_mutex_);
            for (auto it = nodes_.begin(); it != nodes_.end();) {
                auto state = it->second.lock();
                if (!state) {
                    it = nodes_.erase(it);
                    continue;
                }
                {
                    std::lock_guard<std::mutex> node_lock(state->mutex);
                    if (state->node_guard_active && state->node_guard_period.count() > 0 &&
                        state->next_node_guard <= now) {
                        due.push_back(state);
                        do {
                            state->next_node_guard += state->node_guard_period;
                        } while (state->next_node_guard <= now);
                    }
                }
                ++it;
            }
        }
        for (const auto& state : due) {
            const auto node_id = state->id;
            {
                // Pair the active-state check with the pending marker. A concurrent
                // stop/switch to Heartbeat then clears a marker that was already made,
                // and no stale marker can be created after that clear.
                std::lock_guard<std::mutex> node_lock(state->mutex);
                if (!state->node_guard_active) {
                    continue;
                }
                mark_node_guard_request(node_id);
            }
            CanFrame request;
            request.id = 0x700U + node_id;
            request.remote = true;
            auto sent = send_frame(request);
            if (!sent) {
                unmark_node_guard_request(node_id);
                transport_error(sent.error());
            }
        }
    }

    std::shared_ptr<ICanTransport> transport_;
    MasterOptions options_;
    std::atomic<bool> stopping_{false};
    std::thread protocol_thread_;
    std::thread event_thread_;

    std::mutex receive_mutex_;
    std::condition_variable receive_cv_;
    std::deque<CanFrame> receive_queue_;

    std::mutex event_mutex_;
    std::condition_variable event_cv_;
    std::deque<EventJob> event_queue_;
    bool event_stopping_{false};

    std::mutex sdo_mutex_;
    std::map<std::uint8_t, std::deque<std::shared_ptr<SdoTransaction>>> sdo_queues_;
    std::atomic<std::uint64_t> next_sdo_sequence_{1};

    std::mutex nodes_mutex_;
    std::map<std::uint8_t, std::weak_ptr<NodeState>> nodes_;

    std::mutex routes_mutex_;
    std::unordered_map<std::uint32_t, std::pair<std::uint8_t, std::uint8_t>>
        custom_tpdo_routes_;

    std::mutex guard_mutex_;
    std::unordered_map<std::uint8_t, std::size_t> pending_guard_requests_;

    mutable std::mutex sync_mutex_;
    std::condition_variable sync_cv_;
    std::thread sync_thread_;
    bool sync_stop_{true};
    SyncStatistics sync_statistics_;

    std::mutex callbacks_mutex_;
    std::atomic<std::uint64_t> next_callback_token_{1};
    std::unordered_map<std::uint64_t, std::function<void(const CanFrame&)>> raw_callbacks_;
    std::unordered_map<std::uint64_t, std::function<void(const Error&)>> error_callbacks_;
    std::unordered_map<std::uint64_t,
                       NodeCallback<std::function<void(const HeartbeatEvent&)>>>
        heartbeat_callbacks_;
    std::unordered_map<std::uint64_t,
                       NodeCallback<std::function<void(const NodeTimeoutEvent&)>>>
        timeout_callbacks_;
    std::unordered_map<std::uint64_t,
                       NodeCallback<std::function<void(const EmergencyEvent&)>>>
        emergency_callbacks_;
    std::unordered_map<std::uint64_t,
                       NodeCallback<std::function<void(const PdoEvent&)>>>
        pdo_callbacks_;

    mutable std::mutex statistics_mutex_;
    MasterStatistics statistics_;
};

}  // namespace detail

CanopenMaster::CanopenMaster(std::shared_ptr<ICanTransport> transport,
                             MasterOptions options)
    : state_(std::make_shared<detail::MasterState>(std::move(transport), options))
{
    state_->start();
}

CanopenMaster::~CanopenMaster()
{
    if (state_) {
        state_->stop();
    }
}

CanopenMaster::CanopenMaster(CanopenMaster&& other) noexcept
    : state_(std::move(other.state_))
{
}

CanopenMaster& CanopenMaster::operator=(CanopenMaster&& other) noexcept
{
    if (this != &other) {
        if (state_) {
            state_->stop();
        }
        state_ = std::move(other.state_);
    }
    return *this;
}

std::shared_ptr<CanopenNode> CanopenMaster::node(std::uint8_t node_id)
{
    if (node_id == 0 || node_id > 127) {
        throw std::invalid_argument("CANopen Node-ID must be in the range 1..127");
    }
    return std::shared_ptr<CanopenNode>(new CanopenNode(state_->node(node_id)));
}

Result<void> CanopenMaster::send_nmt(NmtCommand command, std::uint8_t node_id)
{
    if (node_id > 127) {
        return Result<void>::failure(ErrorCode::invalid_argument,
                                     "NMT Node-ID must be in the range 0..127");
    }
    CanFrame frame;
    frame.id = 0;
    frame.size = 2;
    frame.data[0] = static_cast<std::uint8_t>(command);
    frame.data[1] = node_id;
    return state_->send_frame(frame);
}

Result<void> CanopenMaster::send_sync()
{
    CanFrame frame;
    frame.id = 0x080;
    return state_->send_frame(frame);
}

Result<void> CanopenMaster::start_periodic_sync(std::chrono::microseconds period)
{
    return state_->start_periodic_sync(period);
}

void CanopenMaster::stop_periodic_sync() noexcept
{
    if (state_) {
        state_->stop_periodic_sync();
    }
}

SyncStatistics CanopenMaster::sync_statistics() const
{
    return state_->sync_statistics();
}

Result<void> CanopenMaster::request_node_guard(std::uint8_t node_id)
{
    if (node_id == 0 || node_id > 127) {
        return Result<void>::failure(ErrorCode::invalid_argument,
                                     "Node Guard Node-ID must be in the range 1..127");
    }
    CanFrame frame;
    frame.id = 0x700U + node_id;
    frame.remote = true;
    state_->mark_node_guard_request(node_id);
    auto sent = state_->send_frame(frame);
    if (!sent) {
        state_->unmark_node_guard_request(node_id);
    }
    return sent;
}

Subscription CanopenMaster::on_raw_frame(std::function<void(const CanFrame&)> callback)
{
    return state_->add_raw_callback(std::move(callback));
}

Subscription CanopenMaster::on_transport_error(std::function<void(const Error&)> callback)
{
    return state_->add_error_callback(std::move(callback));
}

MasterStatistics CanopenMaster::statistics() const
{
    return state_->statistics();
}

void CanopenMaster::reset_statistics()
{
    state_->reset_statistics();
}

bool CanopenMaster::is_internal_thread() const noexcept
{
    return state_ && state_->is_internal_thread();
}

std::shared_ptr<ICanTransport> CanopenMaster::transport() const
{
    return state_->transport();
}

CanopenNode::CanopenNode(std::shared_ptr<detail::NodeState> state)
    : state_(std::move(state))
{
}

CanopenNode::~CanopenNode() = default;

std::uint8_t CanopenNode::id() const noexcept
{
    return state_->id;
}

void CanopenNode::upload_async(ObjectAddress object,
                               SdoOptions options,
                               UploadCallback callback)
{
    if (!callback) {
        return;
    }
    if (auto master = state_->master.lock()) {
        master->enqueue_upload(id(), object, options, std::move(callback));
    } else {
        callback(Result<SdoResponse>::failure(ErrorCode::transport_closed,
                                              "CANopen master no longer exists"));
    }
}

void CanopenNode::download_async(ObjectAddress object,
                                 const std::uint8_t* data,
                                 std::size_t size,
                                 SdoOptions options,
                                 DownloadCallback callback)
{
    if (!callback) {
        return;
    }
    if (auto master = state_->master.lock()) {
        master->enqueue_download(
            id(), object, data, size, options,
            [callback = std::move(callback)](Result<SdoResponse> response) mutable {
                if (!response) {
                    callback(Result<void>::failure(response.error()));
                } else {
                    callback(Result<void>::success());
                }
            });
    } else {
        callback(Result<void>::failure(ErrorCode::transport_closed,
                                       "CANopen master no longer exists"));
    }
}

std::future<Result<SdoResponse>> CanopenNode::upload(ObjectAddress object,
                                                     SdoOptions options)
{
    auto promise = std::make_shared<std::promise<Result<SdoResponse>>>();
    auto future = promise->get_future();
    upload_async(object, options, [promise](Result<SdoResponse> result) mutable {
        promise->set_value(std::move(result));
    });
    return future;
}

std::future<Result<void>> CanopenNode::download(ObjectAddress object,
                                                const std::uint8_t* data,
                                                std::size_t size,
                                                SdoOptions options)
{
    auto promise = std::make_shared<std::promise<Result<void>>>();
    auto future = promise->get_future();
    download_async(object, data, size, options,
                   [promise](Result<void> result) mutable {
                       promise->set_value(std::move(result));
                   });
    return future;
}

Result<SdoResponse> CanopenNode::upload_blocking(ObjectAddress object,
                                                 SdoOptions options)
{
    auto master = state_->master.lock();
    if (!master) {
        return Result<SdoResponse>::failure(ErrorCode::transport_closed,
                                            "CANopen master no longer exists");
    }
    if (master->is_internal_thread()) {
        return Result<SdoResponse>::failure(
            ErrorCode::would_deadlock,
            "blocking SDO cannot be called from a CANopen worker thread");
    }
    return upload(object, options).get();
}

Result<void> CanopenNode::download_blocking(ObjectAddress object,
                                            const std::uint8_t* data,
                                            std::size_t size,
                                            SdoOptions options)
{
    auto master = state_->master.lock();
    if (!master) {
        return Result<void>::failure(ErrorCode::transport_closed,
                                     "CANopen master no longer exists");
    }
    if (master->is_internal_thread()) {
        return Result<void>::failure(
            ErrorCode::would_deadlock,
            "blocking SDO cannot be called from a CANopen worker thread");
    }
    return download(object, data, size, options).get();
}

Result<void> CanopenNode::send_nmt(NmtCommand command)
{
    auto master = state_->master.lock();
    if (!master) {
        return Result<void>::failure(ErrorCode::transport_closed,
                                     "CANopen master no longer exists");
    }
    CanFrame frame;
    frame.id = 0;
    frame.size = 2;
    frame.data[0] = static_cast<std::uint8_t>(command);
    frame.data[1] = id();
    return master->send_frame(frame);
}

Result<void> CanopenNode::configure_pdo(const PdoConfiguration& input,
                                        SdoOptions options)
{
    auto validation = PdoCodec::validate(input);
    if (!validation) {
        return validation;
    }
    PdoConfiguration configuration = input;
    if (configuration.cob_id == 0) {
        configuration.cob_id = PdoCodec::default_cob_id(
            configuration.direction, configuration.number, id());
    }

    const auto communication =
        PdoCodec::communication_index(configuration.direction, configuration.number);
    const auto mapping =
        PdoCodec::mapping_index(configuration.direction, configuration.number);
    std::uint32_t cob_value = configuration.cob_id;
    if (configuration.cob_id > 0x7FFU) {
        cob_value |= (1UL << 29U);
    }

    auto result = write(Object<std::uint32_t>{{communication, 1}, "PDO COB-ID"},
                        cob_value | (1UL << 31U), options);
    if (!result) {
        return result;
    }
    result = write(Object<std::uint8_t>{{mapping, 0}, "PDO map count"}, 0, options);
    if (!result) {
        return result;
    }
    for (std::size_t i = 0; i < configuration.mapping.size(); ++i) {
        result = write(Object<std::uint32_t>{
                           {mapping, static_cast<std::uint8_t>(i + 1U)}, "PDO mapping"},
                       PdoCodec::mapping_value(configuration.mapping[i]), options);
        if (!result) {
            return result;
        }
    }
    result = write(Object<std::uint8_t>{{mapping, 0}, "PDO map count"},
                   static_cast<std::uint8_t>(configuration.mapping.size()), options);
    if (!result) {
        return result;
    }
    result = write(Object<std::uint8_t>{{communication, 2}, "PDO transmission type"},
                   configuration.transmission_type, options);
    if (!result) {
        return result;
    }
    if (configuration.direction == PdoDirection::transmit) {
        result = write(Object<std::uint16_t>{{communication, 3}, "TPDO inhibit time"},
                       configuration.inhibit_time, options);
        if (!result) {
            return result;
        }
        result = write(Object<std::uint16_t>{{communication, 5}, "TPDO event timer"},
                       configuration.event_timer_ms, options);
        if (!result) {
            return result;
        }
    }
    if (configuration.enabled) {
        result = write(Object<std::uint32_t>{{communication, 1}, "PDO COB-ID"},
                       cob_value, options);
        if (!result) {
            return result;
        }
    }

    auto master = state_->master.lock();
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        const auto index = static_cast<std::size_t>(configuration.number - 1U);
        if (configuration.direction == PdoDirection::receive) {
            state_->rpdo[index] = configuration;
        } else {
            state_->tpdo[index] = configuration;
        }
    }
    if (master && configuration.direction == PdoDirection::transmit) {
        master->register_tpdo(id(), configuration);
    }
    return Result<void>::success();
}

Result<void> CanopenNode::disable_pdo(PdoDirection direction,
                                      std::uint8_t number,
                                      SdoOptions options)
{
    if (number < 1 || number > 4) {
        return Result<void>::failure(ErrorCode::invalid_argument,
                                     "PDO number must be in the range 1..4");
    }
    PdoConfiguration configuration;
    configuration.direction = direction;
    configuration.number = number;
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        const auto& stored = direction == PdoDirection::receive
                                 ? state_->rpdo[static_cast<std::size_t>(number - 1U)]
                                 : state_->tpdo[static_cast<std::size_t>(number - 1U)];
        if (stored) {
            configuration = *stored;
        } else {
            configuration.cob_id = PdoCodec::default_cob_id(direction, number, id());
        }
    }
    std::uint32_t cob_value = configuration.cob_id;
    if (configuration.cob_id > 0x7FFU) {
        cob_value |= 1UL << 29U;
    }
    auto result = write(
        Object<std::uint32_t>{{PdoCodec::communication_index(direction, number), 1},
                              "PDO COB-ID"},
        cob_value | (1UL << 31U), options);
    if (!result) {
        return result;
    }
    configuration.enabled = false;
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        auto& stored = direction == PdoDirection::receive
                           ? state_->rpdo[static_cast<std::size_t>(number - 1U)]
                           : state_->tpdo[static_cast<std::size_t>(number - 1U)];
        stored = configuration;
    }
    if (auto master = state_->master.lock();
        master && direction == PdoDirection::transmit) {
        master->register_tpdo(id(), configuration);
    }
    return Result<void>::success();
}

Result<void> CanopenNode::send_rpdo(std::uint8_t number,
                                    const std::uint8_t* data,
                                    std::size_t size)
{
    if (number < 1 || number > 4 || size > 8 || (data == nullptr && size != 0)) {
        return Result<void>::failure(ErrorCode::invalid_argument,
                                     "invalid RPDO number or payload");
    }
    std::uint32_t cob_id = PdoCodec::default_cob_id(PdoDirection::receive, number, id());
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        const auto& stored = state_->rpdo[static_cast<std::size_t>(number - 1U)];
        if (stored) {
            if (!stored->enabled) {
                return Result<void>::failure(ErrorCode::illegal_state,
                                             "RPDO is disabled");
            }
            cob_id = stored->cob_id;
        }
    }
    auto master = state_->master.lock();
    if (!master) {
        return Result<void>::failure(ErrorCode::transport_closed,
                                     "CANopen master no longer exists");
    }
    CanFrame frame;
    frame.id = cob_id;
    frame.extended = cob_id > 0x7FFU;
    frame.size = static_cast<std::uint8_t>(size);
    if (size != 0) {
        std::copy_n(data, size, frame.data.begin());
    }
    return master->send_frame(frame);
}

Result<void> CanopenNode::send_mapped_rpdo(
    std::uint8_t number,
    const std::function<Result<MappedValue>(ObjectAddress)>& value_provider)
{
    std::vector<PdoMappingEntry> mapping;
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (number < 1 || number > 4 ||
            !state_->rpdo[static_cast<std::size_t>(number - 1U)]) {
            return Result<void>::failure(ErrorCode::not_found,
                                         "RPDO mapping is not configured locally");
        }
        mapping = state_->rpdo[static_cast<std::size_t>(number - 1U)]->mapping;
    }
    auto payload = PdoCodec::encode(mapping, value_provider);
    if (!payload) {
        return Result<void>::failure(payload.error());
    }
    return send_rpdo(number, payload.value().data(), payload.value().size());
}

Result<std::vector<MappedValue>> CanopenNode::decode_tpdo(const PdoEvent& event) const
{
    if (event.node_id != id() || event.number < 1 || event.number > 4) {
        return Result<std::vector<MappedValue>>::failure(
            ErrorCode::invalid_argument, "TPDO event does not belong to this node");
    }
    std::vector<PdoMappingEntry> mapping;
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        const auto& stored = state_->tpdo[static_cast<std::size_t>(event.number - 1U)];
        if (!stored) {
            return Result<std::vector<MappedValue>>::failure(
                ErrorCode::not_found, "TPDO mapping is not configured locally");
        }
        mapping = stored->mapping;
    }
    return PdoCodec::decode(mapping, event.data.data(), event.size);
}

Result<void> CanopenNode::configure_heartbeat(std::chrono::milliseconds producer_time,
                                              std::chrono::milliseconds local_timeout,
                                              bool write_producer_to_device,
                                              SdoOptions options)
{
    if (producer_time.count() < 0 || producer_time.count() > 0xFFFF ||
        local_timeout.count() < 0) {
        return Result<void>::failure(ErrorCode::invalid_argument,
                                     "invalid heartbeat producer or timeout value");
    }
    if (write_producer_to_device) {
        auto written = write(Object<std::uint16_t>{{0x1017, 0x00},
                                                   "Heartbeat producer time"},
                             producer_time.count(), options);
        if (!written) {
            return written;
        }
    }
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->node_guard_active = false;
        state_->heartbeat_timeout = local_timeout;
        state_->timeout_reported = false;
        state_->supervision_armed = local_timeout.count() > 0;
        if (state_->supervision_armed) {
            state_->network.last_seen = std::chrono::steady_clock::now();
        }
    }
    if (auto master = state_->master.lock()) {
        master->clear_node_guard_requests(id());
    }
    return Result<void>::success();
}

Result<void> CanopenNode::configure_node_guard(
    std::chrono::milliseconds guard_time,
    std::uint8_t life_time_factor,
    bool write_configuration_to_device,
    SdoOptions options)
{
    if (guard_time.count() <= 0 || guard_time.count() > 0xFFFF ||
        life_time_factor == 0) {
        return Result<void>::failure(ErrorCode::invalid_argument,
                                     "invalid Node Guard time or life-time factor");
    }
    if (write_configuration_to_device) {
        auto written = write(Object<std::uint16_t>{{0x100C, 0x00}, "Guard time"},
                             guard_time.count(), options);
        if (!written) {
            return written;
        }
        written = write(Object<std::uint8_t>{{0x100D, 0x00}, "Life-time factor"},
                        life_time_factor, options);
        if (!written) {
            return written;
        }
    }
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->node_guard_active = true;
        state_->node_guard_period = guard_time;
        state_->next_node_guard = std::chrono::steady_clock::now();
        state_->heartbeat_timeout = guard_time * life_time_factor;
        state_->timeout_reported = false;
        state_->supervision_armed = true;
        state_->network.last_seen = std::chrono::steady_clock::now();
    }
    return Result<void>::success();
}

void CanopenNode::stop_node_guard()
{
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->node_guard_active = false;
        state_->node_guard_period = std::chrono::milliseconds{0};
        state_->heartbeat_timeout = std::chrono::milliseconds{0};
        state_->supervision_armed = false;
        state_->timeout_reported = false;
    }
    if (auto master = state_->master.lock()) {
        master->clear_node_guard_requests(id());
    }
}

void CanopenNode::set_local_heartbeat_timeout(std::chrono::milliseconds timeout)
{
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->heartbeat_timeout = timeout;
    state_->timeout_reported = false;
    state_->supervision_armed = timeout.count() > 0;
    if (state_->supervision_armed) {
        state_->network.last_seen = std::chrono::steady_clock::now();
    }
}

NodeNetworkState CanopenNode::network_state() const
{
    std::lock_guard<std::mutex> lock(state_->mutex);
    return state_->network;
}

Subscription CanopenNode::on_heartbeat(
    std::function<void(const HeartbeatEvent&)> callback)
{
    if (auto master = state_->master.lock()) {
        return master->add_heartbeat_callback(id(), std::move(callback));
    }
    return {};
}

Subscription CanopenNode::on_timeout(
    std::function<void(const NodeTimeoutEvent&)> callback)
{
    if (auto master = state_->master.lock()) {
        return master->add_timeout_callback(id(), std::move(callback));
    }
    return {};
}

Subscription CanopenNode::on_emergency(
    std::function<void(const EmergencyEvent&)> callback)
{
    if (auto master = state_->master.lock()) {
        return master->add_emergency_callback(id(), std::move(callback));
    }
    return {};
}

Subscription CanopenNode::on_tpdo(std::uint8_t number,
                                  std::function<void(const PdoEvent&)> callback)
{
    if (number < 1 || number > 4) {
        return {};
    }
    if (auto master = state_->master.lock()) {
        return master->add_pdo_callback(id(), number, std::move(callback));
    }
    return {};
}

}  // namespace iswv::canopen
