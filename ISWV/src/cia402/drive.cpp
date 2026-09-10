#include "iswv/cia402/drive.hpp"

#include <stdexcept>
#include <thread>

namespace iswv::cia402 {

DriveState decode_state(std::uint16_t word) noexcept
{
    switch (word & 0x004FU) {
    case 0x0000:
        return DriveState::not_ready_to_switch_on;
    case 0x0040:
        return DriveState::switch_on_disabled;
    case 0x000F:
        return DriveState::fault_reaction_active;
    case 0x0008:
        return DriveState::fault;
    default:
        break;
    }

    switch (word & 0x006FU) {
    case 0x0021:
        return DriveState::ready_to_switch_on;
    case 0x0023:
        return DriveState::switched_on;
    case 0x0027:
        return DriveState::operation_enabled;
    case 0x0007:
        return DriveState::quick_stop_active;
    default:
        return DriveState::unknown;
    }
}

const char* state_name(DriveState state) noexcept
{
    switch (state) {
    case DriveState::not_ready_to_switch_on:
        return "not ready to switch on";
    case DriveState::switch_on_disabled:
        return "switch on disabled";
    case DriveState::ready_to_switch_on:
        return "ready to switch on";
    case DriveState::switched_on:
        return "switched on";
    case DriveState::operation_enabled:
        return "operation enabled";
    case DriveState::quick_stop_active:
        return "quick stop active";
    case DriveState::fault_reaction_active:
        return "fault reaction active";
    case DriveState::fault:
        return "fault";
    case DriveState::unknown:
        return "unknown";
    }
    return "unknown";
}

Drive::Drive(std::shared_ptr<canopen::CanopenNode> node) : node_(std::move(node))
{
    if (!node_) {
        throw std::invalid_argument("CiA 402 drive requires a CANopen node");
    }
}

std::shared_ptr<canopen::CanopenNode> Drive::node() const noexcept
{
    return node_;
}

Result<std::uint16_t> Drive::read_status(canopen::SdoOptions options) const
{
    return node_->read(status_word, options);
}

Result<DriveState> Drive::state(canopen::SdoOptions options) const
{
    auto word = read_status(options);
    if (!word) {
        return Result<DriveState>::failure(word.error());
    }
    return Result<DriveState>::success(decode_state(word.value()));
}

Result<void> Drive::write_control(std::uint16_t value,
                                  canopen::SdoOptions options) const
{
    return node_->write(control_word, value, options);
}

Result<void> Drive::set_mode(OperationMode mode, canopen::SdoOptions options) const
{
    return node_->write(modes_of_operation, static_cast<std::int8_t>(mode), options);
}

Result<void> Drive::transition_to(DriveState target, TransitionOptions options) const
{
    if (options.timeout.count() <= 0 || options.poll_interval.count() <= 0) {
        return Result<void>::failure(ErrorCode::invalid_argument,
                                     "transition timeout and poll interval must be positive");
    }
    const auto deadline = std::chrono::steady_clock::now() + options.timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        auto current_result = state(options.sdo);
        if (!current_result) {
            return Result<void>::failure(current_result.error());
        }
        const auto current = current_result.value();
        if (current == target) {
            return Result<void>::success();
        }
        if (current == DriveState::fault ||
            current == DriveState::fault_reaction_active) {
            return Result<void>::failure(ErrorCode::illegal_state,
                                         "drive is in a fault state");
        }

        std::uint16_t command = 0;
        bool command_needed = true;
        if (target == DriveState::operation_enabled) {
            switch (current) {
            case DriveState::switch_on_disabled:
            case DriveState::not_ready_to_switch_on:
            case DriveState::unknown:
                command = control::shutdown;
                break;
            case DriveState::ready_to_switch_on:
                command = control::switch_on;
                break;
            case DriveState::switched_on:
            case DriveState::quick_stop_active:
                command = control::enable_operation;
                break;
            case DriveState::operation_enabled:
            case DriveState::fault_reaction_active:
            case DriveState::fault:
                command_needed = false;
                break;
            }
        } else if (target == DriveState::switched_on) {
            command = current == DriveState::switch_on_disabled
                          ? control::shutdown
                          : control::switch_on;
        } else if (target == DriveState::ready_to_switch_on) {
            command = control::shutdown;
        } else if (target == DriveState::switch_on_disabled) {
            command = control::disable_voltage;
        } else {
            return Result<void>::failure(ErrorCode::unsupported,
                                         "requested CiA 402 transition is not implemented");
        }

        if (command_needed) {
            auto written = write_control(command, options.sdo);
            if (!written) {
                return written;
            }
        }
        std::this_thread::sleep_for(options.poll_interval);
    }
    return Result<void>::failure(ErrorCode::timeout,
                                 std::string("timeout entering state ") +
                                     state_name(target));
}

Result<void> Drive::enable(TransitionOptions options) const
{
    return transition_to(DriveState::operation_enabled, options);
}

Result<void> Drive::disable(canopen::SdoOptions options) const
{
    return write_control(control::shutdown, options);
}

Result<void> Drive::quick_stop(canopen::SdoOptions options) const
{
    return write_control(control::quick_stop, options);
}

Result<void> Drive::fault_reset(canopen::SdoOptions options) const
{
    auto result = write_control(control::fault_reset, options);
    if (!result) {
        return result;
    }
    return write_control(control::shutdown, options);
}

Result<std::uint16_t> Drive::wait_for_status(
    const std::function<bool(std::uint16_t)>& predicate,
    std::chrono::milliseconds timeout,
    std::chrono::milliseconds poll_interval,
    canopen::SdoOptions sdo) const
{
    if (!predicate || timeout.count() <= 0 || poll_interval.count() <= 0) {
        return Result<std::uint16_t>::failure(ErrorCode::invalid_argument,
                                              "invalid status wait arguments");
    }
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        auto word = read_status(sdo);
        if (!word) {
            return word;
        }
        if (predicate(word.value())) {
            return word;
        }
        std::this_thread::sleep_for(poll_interval);
    }
    return Result<std::uint16_t>::failure(ErrorCode::timeout,
                                          "status condition timed out");
}

}  // namespace iswv::cia402
