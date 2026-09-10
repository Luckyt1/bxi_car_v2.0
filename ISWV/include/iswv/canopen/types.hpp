#pragma once

#include "iswv/can_frame.hpp"
#include "iswv/result.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>
#include <vector>

namespace iswv::canopen {

struct ObjectAddress {
    std::uint16_t index{0};
    std::uint8_t subindex{0};

    friend bool operator==(ObjectAddress lhs, ObjectAddress rhs) noexcept
    {
        return lhs.index == rhs.index && lhs.subindex == rhs.subindex;
    }

    friend bool operator!=(ObjectAddress lhs, ObjectAddress rhs) noexcept
    {
        return !(lhs == rhs);
    }

    friend bool operator<(ObjectAddress lhs, ObjectAddress rhs) noexcept
    {
        return lhs.index < rhs.index ||
               (lhs.index == rhs.index && lhs.subindex < rhs.subindex);
    }
};

template <typename T>
struct Object {
    ObjectAddress address;
    const char* name;
};

template <typename T>
using UnsignedEquivalent = std::make_unsigned_t<std::remove_cv_t<T>>;

template <typename T>
std::array<std::uint8_t, sizeof(T)> encode_little_endian(T value)
{
    static_assert(std::is_integral_v<T>, "CANopen scalar must be integral");
    using U = UnsignedEquivalent<T>;
    U raw = static_cast<U>(value);
    std::array<std::uint8_t, sizeof(T)> bytes{};
    for (std::size_t i = 0; i < bytes.size(); ++i) {
        bytes[i] = static_cast<std::uint8_t>((raw >> (i * 8U)) & 0xFFU);
    }
    return bytes;
}

template <typename T>
Result<T> decode_little_endian(const std::uint8_t* data, std::size_t size)
{
    static_assert(std::is_integral_v<T>, "CANopen scalar must be integral");
    if (data == nullptr || size != sizeof(T)) {
        return Result<T>::failure(ErrorCode::protocol_error,
                                  "CANopen scalar has the wrong size");
    }
    using U = UnsignedEquivalent<T>;
    std::uint64_t raw_wide = 0;
    for (std::size_t i = 0; i < size; ++i) {
        raw_wide |= static_cast<std::uint64_t>(data[i]) << (i * 8U);
    }
    const U raw = static_cast<U>(raw_wide);
    T value{};
    static_assert(sizeof(value) == sizeof(raw), "unexpected integer representation");
    std::memcpy(&value, &raw, sizeof(value));
    return Result<T>::success(value);
}

enum class NmtCommand : std::uint8_t {
    start = 0x01,
    stop = 0x02,
    enter_pre_operational = 0x80,
    reset_node = 0x81,
    reset_communication = 0x82
};

enum class NmtState : std::uint8_t {
    initializing = 0x00,
    stopped = 0x04,
    operational = 0x05,
    pre_operational = 0x7F,
    unknown = 0xFF
};

struct SdoOptions {
    std::chrono::milliseconds timeout{250};
};

struct SdoResponse {
    ObjectAddress object;
    std::array<std::uint8_t, 4> data{};
    std::uint8_t size{0};
};

struct HeartbeatEvent {
    std::uint8_t node_id{0};
    NmtState state{NmtState::unknown};
    bool boot_up{false};
    bool node_guard{false};
    bool toggle{false};
    std::chrono::steady_clock::time_point received_at{};
};

struct NodeTimeoutEvent {
    std::uint8_t node_id{0};
    std::chrono::milliseconds timeout{0};
    std::chrono::steady_clock::time_point last_seen{};
};

struct EmergencyEvent {
    std::uint8_t node_id{0};
    std::uint16_t error_code{0};
    std::uint8_t error_register{0};
    std::array<std::uint8_t, 5> manufacturer_data{};
    std::chrono::steady_clock::time_point received_at{};
};

enum class PdoDirection : std::uint8_t { receive, transmit };

struct PdoMappingEntry {
    ObjectAddress object;
    std::uint8_t bit_length{0};
};

struct PdoConfiguration {
    PdoDirection direction{PdoDirection::transmit};
    std::uint8_t number{1};
    std::uint32_t cob_id{0};
    std::uint8_t transmission_type{254};
    std::uint16_t inhibit_time{0};
    std::uint16_t event_timer_ms{0};
    bool enabled{true};
    std::vector<PdoMappingEntry> mapping;
};

struct PdoEvent {
    std::uint8_t node_id{0};
    std::uint8_t number{0};
    std::uint32_t cob_id{0};
    std::array<std::uint8_t, 8> data{};
    std::uint8_t size{0};
    std::chrono::steady_clock::time_point received_at{};
};

struct MappedValue {
    ObjectAddress object;
    std::array<std::uint8_t, 8> data{};
    std::uint8_t size{0};
};

struct NodeNetworkState {
    std::uint8_t node_id{0};
    NmtState nmt_state{NmtState::unknown};
    bool online{false};
    std::chrono::steady_clock::time_point last_seen{};
};

struct MasterStatistics {
    std::uint64_t received_frames{0};
    std::uint64_t transmitted_frames{0};
    std::uint64_t invalid_frames{0};
    std::uint64_t receive_queue_drops{0};
    std::uint64_t event_queue_drops{0};
    std::uint64_t sdo_timeouts{0};
    std::uint64_t sdo_aborts{0};
    std::uint64_t emergency_frames{0};
    std::uint64_t heartbeat_frames{0};
    std::uint64_t pdo_frames{0};
};

struct SyncStatistics {
    bool running{false};
    std::chrono::microseconds period{0};
    std::uint64_t frames_sent{0};
    std::uint64_t send_errors{0};
    std::uint64_t deadline_misses{0};
    std::chrono::microseconds maximum_lateness{0};
};

inline std::string object_name(ObjectAddress object)
{
    constexpr char hex[] = "0123456789ABCDEF";
    std::string result = "0x0000:00";
    for (int i = 0; i < 4; ++i) {
        result[5 - i] = hex[(object.index >> (i * 4)) & 0x0F];
    }
    result[7] = hex[(object.subindex >> 4) & 0x0F];
    result[8] = hex[object.subindex & 0x0F];
    return result;
}

}  // namespace iswv::canopen
