#pragma once

#include "iswv/canopen/master.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>

namespace iswv::cia402 {

inline constexpr canopen::Object<std::uint16_t> control_word{{0x6040, 0x00},
                                                              "Control word"};
inline constexpr canopen::Object<std::uint16_t> status_word{{0x6041, 0x00},
                                                             "Status word"};
inline constexpr canopen::Object<std::int8_t> modes_of_operation{{0x6060, 0x00},
                                                                  "Mode"};

enum class OperationMode : std::int8_t {
    pulse = -4,
    immediate_velocity = -3,
    profile_position = 1,
    profile_velocity = 3,
    torque = 4,
    homing = 6,
    interpolation = 7
};

enum class DriveState : std::uint8_t {
    not_ready_to_switch_on,
    switch_on_disabled,
    ready_to_switch_on,
    switched_on,
    operation_enabled,
    quick_stop_active,
    fault_reaction_active,
    fault,
    unknown
};

struct TransitionOptions {
    std::chrono::milliseconds timeout{1000};
    std::chrono::milliseconds poll_interval{10};
    canopen::SdoOptions sdo{};
};

DriveState decode_state(std::uint16_t status_word) noexcept;
const char* state_name(DriveState state) noexcept;

class Drive {
public:
    explicit Drive(std::shared_ptr<canopen::CanopenNode> node);

    std::shared_ptr<canopen::CanopenNode> node() const noexcept;
    Result<std::uint16_t> read_status(canopen::SdoOptions options = {}) const;
    Result<DriveState> state(canopen::SdoOptions options = {}) const;
    Result<void> write_control(std::uint16_t value,
                               canopen::SdoOptions options = {}) const;
    Result<void> set_mode(OperationMode mode,
                          canopen::SdoOptions options = {}) const;

    Result<void> transition_to(DriveState target,
                               TransitionOptions options = {}) const;
    Result<void> enable(TransitionOptions options = {}) const;
    Result<void> disable(canopen::SdoOptions options = {}) const;
    Result<void> quick_stop(canopen::SdoOptions options = {}) const;
    Result<void> fault_reset(canopen::SdoOptions options = {}) const;

    Result<std::uint16_t> wait_for_status(
        const std::function<bool(std::uint16_t)>& predicate,
        std::chrono::milliseconds timeout,
        std::chrono::milliseconds poll_interval = std::chrono::milliseconds{10},
        canopen::SdoOptions sdo = {}) const;

private:
    std::shared_ptr<canopen::CanopenNode> node_;
};

namespace control {
inline constexpr std::uint16_t disable_voltage = 0x0000;
inline constexpr std::uint16_t shutdown = 0x0006;
inline constexpr std::uint16_t switch_on = 0x0007;
inline constexpr std::uint16_t enable_operation = 0x000F;
inline constexpr std::uint16_t quick_stop = 0x000B;
inline constexpr std::uint16_t fault_reset = 0x0086;
inline constexpr std::uint16_t absolute_prepare = 0x002F;
inline constexpr std::uint16_t absolute_start = 0x003F;
inline constexpr std::uint16_t relative_prepare = 0x004F;
inline constexpr std::uint16_t relative_start = 0x005F;
inline constexpr std::uint16_t immediate_absolute = 0x103F;
inline constexpr std::uint16_t homing_start = 0x001F;
inline constexpr std::uint16_t interpolation_start = 0x001F;
}  // namespace control

namespace status {
inline constexpr std::uint16_t ready_to_switch_on = 1U << 0U;
inline constexpr std::uint16_t switched_on = 1U << 1U;
inline constexpr std::uint16_t operation_enabled = 1U << 2U;
inline constexpr std::uint16_t fault = 1U << 3U;
inline constexpr std::uint16_t voltage_enabled = 1U << 4U;
inline constexpr std::uint16_t quick_stop = 1U << 5U;
inline constexpr std::uint16_t switch_on_disabled = 1U << 6U;
inline constexpr std::uint16_t warning = 1U << 7U;
inline constexpr std::uint16_t remote = 1U << 9U;
inline constexpr std::uint16_t target_reached = 1U << 10U;
inline constexpr std::uint16_t internal_limit = 1U << 11U;
inline constexpr std::uint16_t mode_specific_12 = 1U << 12U;
inline constexpr std::uint16_t following_error = 1U << 13U;
inline constexpr std::uint16_t commutation_found = 1U << 14U;
inline constexpr std::uint16_t reference_found = 1U << 15U;
}  // namespace status

}  // namespace iswv::cia402
