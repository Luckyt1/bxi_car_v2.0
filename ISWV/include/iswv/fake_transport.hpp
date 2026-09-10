#pragma once

#include "iswv/transport.hpp"

#include <chrono>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <vector>

namespace iswv {

class FakeTransport final : public ICanTransport {
public:
    using SendHook = std::function<void(const CanFrame&)>;

    Result<void> send(const CanFrame& frame) override;
    void set_receive_handler(ReceiveHandler handler) override;
    void set_error_handler(TransportErrorHandler handler) override;
    bool is_open() const noexcept override;
    std::string name() const override;

    void inject(const CanFrame& frame);
    void inject_error(Error error);
    void set_send_hook(SendHook hook);
    void close();
    void open();

    bool wait_for_sent(CanFrame& frame, std::chrono::milliseconds timeout);
    std::vector<CanFrame> sent_frames() const;
    void clear_sent();

private:
    mutable std::mutex mutex_;
    std::condition_variable sent_cv_;
    std::deque<CanFrame> sent_;
    ReceiveHandler receive_handler_;
    TransportErrorHandler error_handler_;
    SendHook send_hook_;
    bool open_{true};
};

}  // namespace iswv
