#pragma once

#include <functional>
#include <utility>

namespace iswv {

class Subscription {
public:
    Subscription() = default;
    explicit Subscription(std::function<void()> cancel) : cancel_(std::move(cancel)) {}
    ~Subscription() { reset(); }

    Subscription(const Subscription&) = delete;
    Subscription& operator=(const Subscription&) = delete;

    Subscription(Subscription&& other) noexcept : cancel_(std::move(other.cancel_))
    {
        other.cancel_ = {};
    }

    Subscription& operator=(Subscription&& other) noexcept
    {
        if (this != &other) {
            reset();
            cancel_ = std::move(other.cancel_);
            other.cancel_ = {};
        }
        return *this;
    }

    void reset() noexcept
    {
        if (cancel_) {
            auto cancel = std::move(cancel_);
            cancel_ = {};
            cancel();
        }
    }

    explicit operator bool() const noexcept { return static_cast<bool>(cancel_); }

private:
    std::function<void()> cancel_;
};

}  // namespace iswv
