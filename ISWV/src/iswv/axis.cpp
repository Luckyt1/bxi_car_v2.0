#include "iswv/iswv/axis.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <limits>
#include <map>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace iswv {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

template <typename T>
Result<T> checked_round(double value, const char* description)
{
    if (!std::isfinite(value) ||
        value < static_cast<double>(std::numeric_limits<T>::lowest()) ||
        value > static_cast<double>(std::numeric_limits<T>::max())) {
        return Result<T>::failure(ErrorCode::invalid_argument,
                                  std::string(description) + " is outside the object range");
    }
    return Result<T>::success(static_cast<T>(std::llround(value)));
}

}  // namespace

namespace detail {

class AxisStateImpl : public std::enable_shared_from_this<AxisStateImpl> {
public:
    explicit AxisStateImpl(std::uint8_t node_id)
    {
        state.node_id = node_id;
    }

    void mutate(const std::function<void(AxisState&)>& mutation)
    {
        AxisState snapshot;
        std::vector<std::function<void(const AxisState&)>> listeners;
        {
            std::lock_guard<std::mutex> lock(mutex);
            mutation(state);
            snapshot = state;
            for (const auto& entry : callbacks) {
                listeners.push_back(entry.second);
            }
        }
        for (const auto& listener : listeners) {
            try {
                listener(snapshot);
            } catch (...) {
                // State observers are isolated from internal safety/event handling.
            }
        }
    }

    AxisState snapshot() const
    {
        std::lock_guard<std::mutex> lock(mutex);
        return state;
    }

    Subscription subscribe(std::function<void(const AxisState&)> callback)
    {
        if (!callback) {
            return {};
        }
        std::uint64_t token = 0;
        {
            std::lock_guard<std::mutex> lock(mutex);
            token = next_token++;
            callbacks.emplace(token, std::move(callback));
        }
        std::weak_ptr<AxisStateImpl> weak = shared_from_this();
        return Subscription([weak, token] {
            if (auto self = weak.lock()) {
                std::lock_guard<std::mutex> lock(self->mutex);
                self->callbacks.erase(token);
            }
        });
    }

    mutable std::mutex mutex;
    AxisState state;
    std::map<std::uint64_t, std::function<void(const AxisState&)>> callbacks;
    std::uint64_t next_token{1};
    SafetyPolicy safety;
};

}  // namespace detail

AxisConfiguration AxisConfiguration::manual_travel(std::uint8_t node_id)
{
    AxisConfiguration configuration;
    configuration.kind = AxisKind::travel;
    configuration.node_id = node_id;
    configuration.encoder_resolution = 65536;
    configuration.gear_ratio = 10.0;  // Manual: 2800 rpm motor / 280 rpm wheel.
    configuration.wheel_diameter_m = 0.130;
    return configuration;
}

AxisConfiguration AxisConfiguration::manual_steering(std::uint8_t node_id)
{
    AxisConfiguration configuration;
    configuration.kind = AxisKind::steering;
    configuration.node_id = node_id;
    configuration.encoder_resolution = 65536;
    configuration.gear_ratio = 2500.0 / 277.0;  // Nominal ratio derived from the manual.
    configuration.wheel_diameter_m = 0.130;
    return configuration;
}

class Axis::Impl {
public:
    Impl(std::shared_ptr<canopen::CanopenNode> node_in,
         AxisConfiguration configuration_in)
        : configuration(std::move(configuration_in)),
          node(std::move(node_in)),
          drive(node),
          state(std::make_shared<detail::AxisStateImpl>(configuration.node_id))
    {
        if (!node) {
            throw std::invalid_argument("iSWV axis requires a CANopen node");
        }
        if (node->id() != configuration.node_id || configuration.encoder_resolution == 0 ||
            !std::isfinite(configuration.gear_ratio) || configuration.gear_ratio <= 0.0 ||
            !std::isfinite(configuration.wheel_diameter_m) ||
            configuration.wheel_diameter_m <= 0.0) {
            throw std::invalid_argument("invalid iSWV axis configuration");
        }
        install_event_handlers();
    }

    void install_event_handlers()
    {
        std::weak_ptr<detail::AxisStateImpl> weak_state = state;
        std::weak_ptr<canopen::CanopenNode> weak_node = node;

        subscriptions.push_back(node->on_heartbeat(
            [weak_state](const canopen::HeartbeatEvent& event) {
                if (auto current = weak_state.lock()) {
                    current->mutate([&event](AxisState& axis) {
                        axis.online = true;
                        axis.nmt_state = event.state;
                        axis.updated_at = event.received_at;
                    });
                }
            }));
        subscriptions.push_back(node->on_timeout(
            [weak_state, weak_node](const canopen::NodeTimeoutEvent&) {
                SafetyPolicy policy;
                if (auto current = weak_state.lock()) {
                    {
                        std::lock_guard<std::mutex> lock(current->mutex);
                        policy = current->safety;
                    }
                    current->mutate([](AxisState& axis) {
                        axis.online = false;
                        axis.updated_at = std::chrono::steady_clock::now();
                    });
                } else {
                    return;
                }
                auto current_node = weak_node.lock();
                if (!current_node || !policy.enabled) {
                    return;
                }
                std::uint16_t command = 0;
                if (policy.timeout_action == TimeoutAction::quick_stop) {
                    command = cia402::control::quick_stop;
                } else if (policy.timeout_action == TimeoutAction::disable) {
                    command = cia402::control::disable_voltage;
                } else {
                    return;
                }
                current_node->write_async(objects::control_word, command, {}, [](Result<void>) {});
            }));
        subscriptions.push_back(node->on_emergency(
            [weak_state](const canopen::EmergencyEvent& event) {
                if (auto current = weak_state.lock()) {
                    current->mutate([&event](AxisState& axis) {
                        axis.last_emergency_code = event.error_code;
                        axis.last_emergency_register = event.error_register;
                        // The iSWV manual leaves manufacturer_data[0] (EMCY byte 3) unused.
                        axis.error_status = static_cast<std::uint16_t>(
                            event.manufacturer_data[1] |
                            (static_cast<std::uint16_t>(event.manufacturer_data[2]) << 8U));
                        axis.error_status_2 = static_cast<std::uint16_t>(
                            event.manufacturer_data[3] |
                            (static_cast<std::uint16_t>(event.manufacturer_data[4]) << 8U));
                        axis.updated_at = event.received_at;
                    });
                }
            }));

        for (std::uint8_t number = 1; number <= 3; ++number) {
            subscriptions.push_back(node->on_tpdo(
                number, [weak_state, weak_node](const canopen::PdoEvent& event) {
                    auto current = weak_state.lock();
                    auto current_node = weak_node.lock();
                    if (!current || !current_node) {
                        return;
                    }
                    auto decoded = current_node->decode_tpdo(event);
                    if (!decoded) {
                        return;
                    }
                    current->mutate([&decoded, &event](AxisState& axis) {
                        for (const auto& value : decoded.value()) {
                            if (value.object == objects::status_word.address) {
                                auto item = canopen::decode_little_endian<std::uint16_t>(
                                    value.data.data(), value.size);
                                if (item) {
                                    axis.status_word = item.value();
                                    axis.drive_state = cia402::decode_state(item.value());
                                }
                            } else if (value.object == objects::actual_position.address) {
                                auto item = canopen::decode_little_endian<std::int32_t>(
                                    value.data.data(), value.size);
                                if (item) {
                                    axis.position_raw = item.value();
                                }
                            } else if (value.object == objects::actual_velocity.address) {
                                auto item = canopen::decode_little_endian<std::int32_t>(
                                    value.data.data(), value.size);
                                if (item) {
                                    axis.velocity_raw = item.value();
                                }
                            } else if (value.object == objects::actual_current.address) {
                                auto item = canopen::decode_little_endian<std::int16_t>(
                                    value.data.data(), value.size);
                                if (item) {
                                    axis.current_raw = item.value();
                                }
                            } else if (value.object == objects::digital_inputs.address) {
                                auto item = canopen::decode_little_endian<std::uint32_t>(
                                    value.data.data(), value.size);
                                if (item) {
                                    axis.digital_inputs = item.value();
                                }
                            } else if (value.object == objects::error_status.address) {
                                auto item = canopen::decode_little_endian<std::uint16_t>(
                                    value.data.data(), value.size);
                                if (item) {
                                    axis.error_status = item.value();
                                }
                            } else if (value.object == objects::error_status_2.address) {
                                auto item = canopen::decode_little_endian<std::uint16_t>(
                                    value.data.data(), value.size);
                                if (item) {
                                    axis.error_status_2 = item.value();
                                }
                            }
                        }
                        axis.online = true;
                        axis.has_process_data = true;
                        axis.updated_at = event.received_at;
                    });
                }));
        }
    }

    AxisConfiguration configuration;
    std::shared_ptr<canopen::CanopenNode> node;
    cia402::Drive drive;
    std::shared_ptr<detail::AxisStateImpl> state;
    std::vector<Subscription> subscriptions;
    std::atomic<std::int8_t> velocity_mode{
        static_cast<std::int8_t>(cia402::OperationMode::profile_velocity)};
};

Axis::Axis(std::shared_ptr<canopen::CanopenNode> node,
           AxisConfiguration configuration)
    : impl_(std::make_unique<Impl>(std::move(node), std::move(configuration)))
{
}

Axis::~Axis() = default;
Axis::Axis(Axis&&) noexcept = default;
Axis& Axis::operator=(Axis&&) noexcept = default;

const AxisConfiguration& Axis::configuration() const noexcept
{
    return impl_->configuration;
}

std::shared_ptr<canopen::CanopenNode> Axis::node() const noexcept
{
    return impl_->node;
}

cia402::Drive& Axis::drive() noexcept
{
    return impl_->drive;
}

const cia402::Drive& Axis::drive() const noexcept
{
    return impl_->drive;
}

Result<void> Axis::configure_default_pdos(DefaultPdoOptions options,
                                          canopen::SdoOptions sdo)
{
    using canopen::PdoConfiguration;
    using canopen::PdoDirection;
    using canopen::PdoMappingEntry;

    const std::array<PdoConfiguration, 7> configurations{{
        {PdoDirection::receive,
         1,
         0,
         options.rpdo_transmission_type,
         0,
         0,
         true,
         {{objects::control_word.address, 16},
          {objects::mode.address, 8},
          {objects::target_velocity.address, 32}}},
        {PdoDirection::receive,
         2,
         0,
         options.rpdo_transmission_type,
         0,
         0,
         true,
         {{objects::control_word.address, 16}, {objects::target_position.address, 32}}},
        {PdoDirection::receive,
         3,
         0,
         options.rpdo_transmission_type,
         0,
         0,
         true,
         {{objects::control_word.address, 16}, {objects::target_torque.address, 16}}},
        {PdoDirection::receive,
         4,
         0,
         options.rpdo_transmission_type,
         0,
         0,
         true,
         {{objects::target_position.address, 32}}},
        {PdoDirection::transmit,
         1,
         0,
         options.tpdo_transmission_type,
         options.tpdo_inhibit_time,
         options.tpdo_event_timer_ms,
         true,
         {{objects::status_word.address, 16}, {objects::actual_position.address, 32}}},
        {PdoDirection::transmit,
         2,
         0,
         options.tpdo_transmission_type,
         options.tpdo_inhibit_time,
         options.tpdo_event_timer_ms,
         true,
         {{objects::actual_velocity.address, 32}, {objects::actual_current.address, 16}}},
        {PdoDirection::transmit,
         3,
         0,
         options.tpdo_transmission_type,
         options.tpdo_inhibit_time,
         options.tpdo_event_timer_ms,
         true,
         {{objects::digital_inputs.address, 32},
          {objects::error_status.address, 16},
          {objects::error_status_2.address, 16}}},
    }};

    for (const auto& configuration : configurations) {
        auto result = impl_->node->configure_pdo(configuration, sdo);
        if (!result) {
            return result;
        }
    }
    return Result<void>::success();
}

Result<void> Axis::apply_safety_policy(SafetyPolicy policy, canopen::SdoOptions sdo)
{
    if (policy.local_heartbeat_timeout.count() < 0 ||
        policy.heartbeat_producer_time.count() < 0) {
        return Result<void>::failure(ErrorCode::invalid_argument,
                                     "heartbeat timeout cannot be negative");
    }
    if (policy.configure_device_interruption_fault) {
        auto configured = impl_->node->write(objects::communication_interruption_mode, 1, sdo);
        if (!configured) {
            return configured;
        }
    }
    if (policy.configure_heartbeat_producer) {
        auto configured = impl_->node->configure_heartbeat(
            policy.heartbeat_producer_time,
            policy.enabled ? policy.local_heartbeat_timeout : std::chrono::milliseconds{0},
            true, sdo);
        if (!configured) {
            return configured;
        }
    }
    {
        std::lock_guard<std::mutex> lock(impl_->state->mutex);
        impl_->state->safety = policy;
    }
    if (!policy.configure_heartbeat_producer) {
        impl_->node->set_local_heartbeat_timeout(
            policy.enabled ? policy.local_heartbeat_timeout : std::chrono::milliseconds{0});
    }
    return Result<void>::success();
}

Result<void> Axis::configure_velocity_mode(bool immediate,
                                           double acceleration_rps2,
                                           double deceleration_rps2,
                                           canopen::SdoOptions sdo)
{
    auto acceleration = acceleration_to_raw(acceleration_rps2);
    auto deceleration = acceleration_to_raw(deceleration_rps2);
    if (!acceleration) {
        return Result<void>::failure(acceleration.error());
    }
    if (!deceleration) {
        return Result<void>::failure(deceleration.error());
    }
    auto result = set_direction(impl_->configuration.inverted, sdo);
    if (!result) {
        return result;
    }
    result = impl_->drive.set_mode(immediate ? cia402::OperationMode::immediate_velocity
                                             : cia402::OperationMode::profile_velocity,
                                   sdo);
    if (!result) {
        return result;
    }
    impl_->velocity_mode.store(static_cast<std::int8_t>(
        immediate ? cia402::OperationMode::immediate_velocity
                  : cia402::OperationMode::profile_velocity));
    result = impl_->node->write(objects::profile_acceleration, acceleration.value(), sdo);
    if (!result) {
        return result;
    }
    return impl_->node->write(objects::profile_deceleration, deceleration.value(), sdo);
}

Result<void> Axis::command_motor_velocity_rpm(double rpm, canopen::SdoOptions sdo)
{
    auto raw = motor_rpm_to_raw(rpm);
    if (!raw) {
        return Result<void>::failure(raw.error());
    }
    return impl_->node->write(objects::target_velocity, raw.value(), sdo);
}

Result<void> Axis::command_travel_velocity_mps(double metres_per_second,
                                               canopen::SdoOptions sdo)
{
    auto rpm = travel_mps_to_motor_rpm(metres_per_second);
    if (!rpm) {
        return Result<void>::failure(rpm.error());
    }
    return command_motor_velocity_rpm(rpm.value(), sdo);
}

Result<void> Axis::command_velocity_pdo(double motor_rpm)
{
    auto raw = motor_rpm_to_raw(motor_rpm);
    if (!raw) {
        return Result<void>::failure(raw.error());
    }
    const auto mode = impl_->velocity_mode.load();
    return impl_->node->send_mapped_rpdo(
        1, [raw = raw.value(), mode](canopen::ObjectAddress object) {
            if (object == objects::control_word.address) {
                return Result<canopen::MappedValue>::success(
                    canopen::mapped_value(objects::control_word,
                                          cia402::control::enable_operation));
            }
            if (object == objects::mode.address) {
                return Result<canopen::MappedValue>::success(
                    canopen::mapped_value(objects::mode, mode));
            }
            if (object == objects::target_velocity.address) {
                return Result<canopen::MappedValue>::success(
                    canopen::mapped_value(objects::target_velocity, raw));
            }
            return Result<canopen::MappedValue>::failure(ErrorCode::not_found,
                                                         "unexpected velocity RPDO object");
        });
}

Result<void> Axis::configure_position_mode(double profile_velocity_rpm,
                                           double acceleration_rps2,
                                           double deceleration_rps2,
                                           canopen::SdoOptions sdo)
{
    if (profile_velocity_rpm < 0.0) {
        return Result<void>::failure(ErrorCode::invalid_argument,
                                     "profile velocity must be non-negative");
    }
    auto velocity = motor_rpm_to_raw(profile_velocity_rpm);
    auto acceleration = acceleration_to_raw(acceleration_rps2);
    auto deceleration = acceleration_to_raw(deceleration_rps2);
    if (!velocity) {
        return Result<void>::failure(velocity.error());
    }
    if (!acceleration) {
        return Result<void>::failure(acceleration.error());
    }
    if (!deceleration) {
        return Result<void>::failure(deceleration.error());
    }
    auto result = set_direction(impl_->configuration.inverted, sdo);
    if (!result) {
        return result;
    }
    result = impl_->drive.set_mode(cia402::OperationMode::profile_position, sdo);
    if (!result) {
        return result;
    }
    result = impl_->node->write(objects::profile_velocity,
                                static_cast<std::uint32_t>(velocity.value()), sdo);
    if (!result) {
        return result;
    }
    result = impl_->node->write(objects::profile_acceleration, acceleration.value(), sdo);
    if (!result) {
        return result;
    }
    return impl_->node->write(objects::profile_deceleration, deceleration.value(), sdo);
}

Result<void> Axis::move_absolute_inc(std::int32_t target, canopen::SdoOptions sdo)
{
    auto result = impl_->node->write(objects::target_position, target, sdo);
    if (!result) {
        return result;
    }
    result = impl_->drive.write_control(cia402::control::absolute_prepare, sdo);
    if (!result) {
        return result;
    }
    return impl_->drive.write_control(cia402::control::absolute_start, sdo);
}

Result<void> Axis::move_relative_inc(std::int32_t delta, canopen::SdoOptions sdo)
{
    auto result = impl_->node->write(objects::target_position, delta, sdo);
    if (!result) {
        return result;
    }
    result = impl_->drive.write_control(cia402::control::relative_prepare, sdo);
    if (!result) {
        return result;
    }
    return impl_->drive.write_control(cia402::control::relative_start, sdo);
}

Result<void> Axis::move_continuous_absolute_inc(std::int32_t target,
                                                canopen::SdoOptions sdo)
{
    auto result = impl_->node->write(objects::target_position, target, sdo);
    if (!result) {
        return result;
    }
    return impl_->drive.write_control(cia402::control::immediate_absolute, sdo);
}

Result<void> Axis::move_steering_angle_rad(double radians, canopen::SdoOptions sdo)
{
    auto increments = steering_radians_to_inc(radians);
    if (!increments) {
        return Result<void>::failure(increments.error());
    }
    return move_absolute_inc(increments.value(), sdo);
}

Result<void> Axis::command_position_pdo(std::int32_t target, bool relative)
{
    const std::array<std::uint16_t, 2> controls =
        relative ? std::array<std::uint16_t, 2>{cia402::control::relative_prepare,
                                                cia402::control::relative_start}
                 : std::array<std::uint16_t, 2>{cia402::control::absolute_prepare,
                                                cia402::control::absolute_start};
    for (const auto control : controls) {
        auto result = impl_->node->send_mapped_rpdo(
            2, [target, control](canopen::ObjectAddress object) {
                if (object == objects::control_word.address) {
                    return Result<canopen::MappedValue>::success(
                        canopen::mapped_value(objects::control_word, control));
                }
                if (object == objects::target_position.address) {
                    return Result<canopen::MappedValue>::success(
                        canopen::mapped_value(objects::target_position, target));
                }
                return Result<canopen::MappedValue>::failure(
                    ErrorCode::not_found, "unexpected position RPDO object");
            });
        if (!result) {
            return result;
        }
    }
    return Result<void>::success();
}

Result<void> Axis::command_interpolated_position_pdo(std::int32_t target)
{
    return impl_->node->send_mapped_rpdo(
        4, [target](canopen::ObjectAddress object) {
            if (object == objects::target_position.address) {
                return Result<canopen::MappedValue>::success(
                    canopen::mapped_value(objects::target_position, target));
            }
            return Result<canopen::MappedValue>::failure(
                ErrorCode::not_found, "unexpected interpolation RPDO object");
        });
}

Result<void> Axis::configure_torque_mode(canopen::SdoOptions sdo)
{
    return impl_->drive.set_mode(cia402::OperationMode::torque, sdo);
}

Result<void> Axis::command_torque_percent(double percent, canopen::SdoOptions sdo)
{
    if (!std::isfinite(percent) || percent < -100.0 || percent > 100.0) {
        return Result<void>::failure(ErrorCode::invalid_argument,
                                     "target torque percentage must be within [-100, 100]");
    }
    auto raw = checked_round<std::int16_t>(percent, "target torque");
    if (!raw) {
        return Result<void>::failure(raw.error());
    }
    return impl_->node->write(objects::target_torque, raw.value(), sdo);
}

Result<void> Axis::command_torque_pdo(double percent)
{
    if (!std::isfinite(percent) || percent < -100.0 || percent > 100.0) {
        return Result<void>::failure(ErrorCode::invalid_argument,
                                     "target torque percentage must be within [-100, 100]");
    }
    auto raw = checked_round<std::int16_t>(percent, "target torque");
    if (!raw) {
        return Result<void>::failure(raw.error());
    }
    return impl_->node->send_mapped_rpdo(
        3, [raw = raw.value()](canopen::ObjectAddress object) {
            if (object == objects::control_word.address) {
                return Result<canopen::MappedValue>::success(
                    canopen::mapped_value(objects::control_word,
                                          cia402::control::enable_operation));
            }
            if (object == objects::target_torque.address) {
                return Result<canopen::MappedValue>::success(
                    canopen::mapped_value(objects::target_torque, raw));
            }
            return Result<canopen::MappedValue>::failure(ErrorCode::not_found,
                                                         "unexpected torque RPDO object");
        });
}

Result<void> Axis::configure_homing(const HomingConfiguration& configuration,
                                    canopen::SdoOptions sdo)
{
    if (configuration.switch_velocity_rpm < 0.0 ||
        configuration.zero_velocity_rpm < 0.0) {
        return Result<void>::failure(ErrorCode::invalid_argument,
                                     "homing velocities must be non-negative");
    }
    auto switch_velocity = motor_rpm_to_raw(configuration.switch_velocity_rpm);
    auto zero_velocity = motor_rpm_to_raw(configuration.zero_velocity_rpm);
    auto acceleration = acceleration_to_raw(configuration.acceleration_rps2);
    if (!switch_velocity) {
        return Result<void>::failure(switch_velocity.error());
    }
    if (!zero_velocity) {
        return Result<void>::failure(zero_velocity.error());
    }
    if (!acceleration) {
        return Result<void>::failure(acceleration.error());
    }
    auto result = impl_->drive.set_mode(cia402::OperationMode::homing, sdo);
    if (!result) {
        return result;
    }
    result = impl_->node->write(objects::homing_method, configuration.method, sdo);
    if (!result) {
        return result;
    }
    result = impl_->node->write(objects::homing_switch_velocity,
                                static_cast<std::uint32_t>(switch_velocity.value()), sdo);
    if (!result) {
        return result;
    }
    result = impl_->node->write(objects::homing_zero_velocity,
                                static_cast<std::uint32_t>(zero_velocity.value()), sdo);
    if (!result) {
        return result;
    }
    result = impl_->node->write(objects::homing_acceleration, acceleration.value(), sdo);
    if (!result) {
        return result;
    }
    result = impl_->node->write(objects::home_offset, configuration.home_offset_inc, sdo);
    if (!result) {
        return result;
    }
    return impl_->node->write(objects::homing_offset_mode, configuration.offset_mode, sdo);
}

Result<void> Axis::start_homing(canopen::SdoOptions sdo)
{
    auto result = impl_->drive.write_control(cia402::control::enable_operation, sdo);
    if (!result) {
        return result;
    }
    return impl_->drive.write_control(cia402::control::homing_start, sdo);
}

Result<void> Axis::wait_homing(std::chrono::milliseconds timeout,
                               canopen::SdoOptions sdo)
{
    auto word = impl_->drive.wait_for_status(
        [](std::uint16_t status) {
            return (status & cia402::status::reference_found) != 0U ||
                   (status & cia402::status::mode_specific_12) != 0U;
        },
        timeout, std::chrono::milliseconds{10}, sdo);
    if (!word) {
        return Result<void>::failure(word.error());
    }
    if ((word.value() & cia402::status::mode_specific_12) != 0U) {
        return Result<void>::failure(ErrorCode::illegal_state, "homing failed");
    }
    return Result<void>::success();
}

Result<void> Axis::configure_interpolation(
    const InterpolationConfiguration& configuration,
    canopen::SdoOptions sdo)
{
    std::uint8_t period_code = 0;
    if (configuration.sync_period == std::chrono::milliseconds{1}) {
        period_code = 0;
    } else if (configuration.sync_period == std::chrono::milliseconds{2}) {
        period_code = 1;
    } else if (configuration.sync_period == std::chrono::milliseconds{4}) {
        period_code = 2;
    } else if (configuration.sync_period == std::chrono::milliseconds{8}) {
        period_code = 3;
    } else {
        return Result<void>::failure(ErrorCode::invalid_argument,
                                     "iSWV interpolation period must be 1, 2, 4, or 8 ms");
    }
    auto result = impl_->drive.set_mode(cia402::OperationMode::interpolation, sdo);
    if (!result) {
        return result;
    }
    result = impl_->node->write(objects::ecan_sync_period, period_code, sdo);
    if (!result) {
        return result;
    }
    return impl_->node->write(objects::ecan_sync_clock_mode,
                              configuration.enable_sync_clock ? 1 : 0, sdo);
}

Result<void> Axis::start_interpolation(canopen::SdoOptions sdo)
{
    return impl_->drive.write_control(cia402::control::interpolation_start, sdo);
}

Result<void> Axis::stop_interpolation(canopen::SdoOptions sdo)
{
    return impl_->drive.write_control(cia402::control::enable_operation, sdo);
}

Result<std::uint16_t> Axis::read_sync_lost_count(canopen::SdoOptions sdo) const
{
    return impl_->node->read(objects::ecan_sync_lost_count, sdo);
}

Result<void> Axis::set_direction(bool inverted, canopen::SdoOptions sdo)
{
    return impl_->node->write(objects::direction, inverted ? 1 : 0, sdo);
}

Result<void> Axis::set_software_limits(std::int32_t negative,
                                       std::int32_t positive,
                                       canopen::SdoOptions sdo)
{
    if (negative >= positive) {
        return Result<void>::failure(ErrorCode::invalid_argument,
                                     "negative software limit must be below positive limit");
    }
    auto result = impl_->node->write(objects::negative_software_limit, negative, sdo);
    if (!result) {
        return result;
    }
    return impl_->node->write(objects::positive_software_limit, positive, sdo);
}

Result<void> Axis::set_stop_options(std::int16_t quick_stop,
                                    std::int16_t shutdown,
                                    std::int16_t disable,
                                    std::int16_t halt,
                                    std::int16_t fault,
                                    canopen::SdoOptions sdo)
{
    auto result = impl_->node->write(objects::quick_stop_option, quick_stop, sdo);
    if (!result) {
        return result;
    }
    result = impl_->node->write(objects::shutdown_option, shutdown, sdo);
    if (!result) {
        return result;
    }
    result = impl_->node->write(objects::disable_operation_option, disable, sdo);
    if (!result) {
        return result;
    }
    result = impl_->node->write(objects::halt_option, halt, sdo);
    if (!result) {
        return result;
    }
    return impl_->node->write(objects::fault_reaction_option, fault, sdo);
}

Result<void> Axis::store_parameters(canopen::SdoOptions sdo)
{
    return impl_->node->write(objects::store_control_parameters, 1, sdo);
}

Result<void> Axis::restore_default_parameters(canopen::SdoOptions sdo)
{
    return impl_->node->write(objects::store_control_parameters, 10, sdo);
}

Result<void> Axis::set_can_parameters(std::uint16_t new_node_id,
                                      std::uint8_t bitrate_code,
                                      bool store,
                                      canopen::SdoOptions sdo)
{
    constexpr std::array<std::uint8_t, 6> valid_bitrate_codes{1, 5, 12, 25, 50, 100};
    if (new_node_id == 0 || new_node_id > 127 ||
        std::find(valid_bitrate_codes.begin(), valid_bitrate_codes.end(), bitrate_code) ==
            valid_bitrate_codes.end()) {
        return Result<void>::failure(ErrorCode::invalid_argument,
                                     "invalid Node-ID or iSWV bitrate code");
    }
    auto result = impl_->node->write(objects::device_node_id, new_node_id, sdo);
    if (!result) {
        return result;
    }
    result = impl_->node->write(objects::can_bitrate_code, bitrate_code, sdo);
    if (!result || !store) {
        return result;
    }
    return store_parameters(sdo);
}

Result<AxisState> Axis::refresh_state(canopen::SdoOptions sdo)
{
    auto status = impl_->node->read(objects::status_word, sdo);
    if (!status) {
        return Result<AxisState>::failure(status.error());
    }
    auto position = impl_->node->read(objects::actual_position, sdo);
    if (!position) {
        return Result<AxisState>::failure(position.error());
    }
    auto velocity = impl_->node->read(objects::actual_velocity, sdo);
    if (!velocity) {
        return Result<AxisState>::failure(velocity.error());
    }
    auto current = impl_->node->read(objects::actual_current, sdo);
    if (!current) {
        return Result<AxisState>::failure(current.error());
    }
    auto inputs = impl_->node->read(objects::digital_inputs, sdo);
    if (!inputs) {
        return Result<AxisState>::failure(inputs.error());
    }
    auto error_1 = impl_->node->read(objects::error_status, sdo);
    if (!error_1) {
        return Result<AxisState>::failure(error_1.error());
    }
    auto error_2 = impl_->node->read(objects::error_status_2, sdo);
    if (!error_2) {
        return Result<AxisState>::failure(error_2.error());
    }
    const auto network = impl_->node->network_state();
    impl_->state->mutate([&](AxisState& axis) {
        axis.online = network.online;
        axis.nmt_state = network.nmt_state;
        axis.status_word = status.value();
        axis.drive_state = cia402::decode_state(status.value());
        axis.position_raw = position.value();
        axis.velocity_raw = velocity.value();
        axis.current_raw = current.value();
        axis.digital_inputs = inputs.value();
        axis.error_status = error_1.value();
        axis.error_status_2 = error_2.value();
        axis.has_process_data = true;
        axis.updated_at = std::chrono::steady_clock::now();
    });
    return Result<AxisState>::success(impl_->state->snapshot());
}

AxisState Axis::state_snapshot() const
{
    return impl_->state->snapshot();
}

std::vector<FaultInfo> Axis::faults() const
{
    const auto snapshot = state_snapshot();
    return decode_fault_status(snapshot.error_status, snapshot.error_status_2);
}

Subscription Axis::on_state(std::function<void(const AxisState&)> callback)
{
    return impl_->state->subscribe(std::move(callback));
}

Result<std::int32_t> Axis::motor_rpm_to_raw(double rpm) const
{
    if (!std::isfinite(rpm)) {
        return Result<std::int32_t>::failure(ErrorCode::invalid_argument,
                                             "motor velocity is not finite");
    }
    const double raw = rpm * 512.0 * impl_->configuration.encoder_resolution / 1875.0;
    return checked_round<std::int32_t>(raw, "motor velocity");
}

double Axis::raw_to_motor_rpm(std::int32_t raw) const noexcept
{
    return static_cast<double>(raw) * 1875.0 /
           (512.0 * impl_->configuration.encoder_resolution);
}

Result<std::uint32_t> Axis::acceleration_to_raw(double rps2) const
{
    if (!std::isfinite(rps2) || rps2 < 0.0) {
        return Result<std::uint32_t>::failure(ErrorCode::invalid_argument,
                                              "acceleration must be finite and non-negative");
    }
    const double raw = rps2 * 65536.0 * impl_->configuration.encoder_resolution /
                       4000000.0;
    return checked_round<std::uint32_t>(raw, "acceleration");
}

double Axis::raw_to_acceleration(std::uint32_t raw) const noexcept
{
    return static_cast<double>(raw) * 4000000.0 /
           (65536.0 * impl_->configuration.encoder_resolution);
}

Result<std::int32_t> Axis::output_revolutions_to_inc(double revolutions) const
{
    if (!std::isfinite(revolutions)) {
        return Result<std::int32_t>::failure(ErrorCode::invalid_argument,
                                             "output revolutions are not finite");
    }
    return checked_round<std::int32_t>(
        revolutions * impl_->configuration.gear_ratio *
            impl_->configuration.encoder_resolution,
        "output position");
}

double Axis::inc_to_output_revolutions(std::int32_t increments) const noexcept
{
    return static_cast<double>(increments) /
           (impl_->configuration.gear_ratio *
            impl_->configuration.encoder_resolution);
}

Result<std::int32_t> Axis::steering_radians_to_inc(double radians) const
{
    if (impl_->configuration.kind != AxisKind::steering) {
        return Result<std::int32_t>::failure(ErrorCode::illegal_state,
                                             "axis is not configured as a steering axis");
    }
    return output_revolutions_to_inc(radians / (2.0 * kPi));
}

double Axis::inc_to_steering_radians(std::int32_t increments) const noexcept
{
    return inc_to_output_revolutions(increments) * 2.0 * kPi;
}

Result<double> Axis::travel_mps_to_motor_rpm(double metres_per_second) const
{
    if (impl_->configuration.kind != AxisKind::travel ||
        !std::isfinite(metres_per_second)) {
        return Result<double>::failure(ErrorCode::invalid_argument,
                                       "invalid travel-axis linear velocity");
    }
    const double wheel_rps = metres_per_second /
                             (kPi * impl_->configuration.wheel_diameter_m);
    return Result<double>::success(wheel_rps * 60.0 *
                                   impl_->configuration.gear_ratio);
}

Result<double> Axis::motor_rpm_to_travel_mps(double rpm) const
{
    if (impl_->configuration.kind != AxisKind::travel || !std::isfinite(rpm)) {
        return Result<double>::failure(ErrorCode::invalid_argument,
                                       "invalid travel-axis motor velocity");
    }
    const double wheel_rps = rpm / (60.0 * impl_->configuration.gear_ratio);
    return Result<double>::success(wheel_rps * kPi *
                                   impl_->configuration.wheel_diameter_m);
}

}  // namespace iswv
