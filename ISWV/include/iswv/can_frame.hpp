#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace iswv {

inline constexpr std::size_t kClassicCanPayload = 8;
inline constexpr std::size_t kCanFdPayload = 64;

struct CanFrame {
    std::uint32_t id{0};
    bool extended{false};
    bool remote{false};
    bool error{false};
    bool fd{false};
    bool bitrate_switch{false};
    bool error_state_indicator{false};
    std::uint8_t size{0};
    std::array<std::uint8_t, kCanFdPayload> data{};
    std::chrono::steady_clock::time_point received_at{};
};

inline bool valid_frame(const CanFrame& frame) noexcept
{
    // SocketCAN error frames use the 29-bit CAN_ERR_MASK namespace without
    // being extended data frames.
    const auto max_id = (frame.extended || frame.error) ? 0x1FFFFFFFU : 0x7FFU;
    const auto max_size = frame.fd ? kCanFdPayload : kClassicCanPayload;
    return frame.id <= max_id && frame.size <= max_size &&
           (!frame.remote || !frame.fd) &&
           (frame.fd || (!frame.bitrate_switch && !frame.error_state_indicator)) &&
           (!frame.error || (!frame.extended && !frame.remote && !frame.fd));
}

}  // namespace iswv
