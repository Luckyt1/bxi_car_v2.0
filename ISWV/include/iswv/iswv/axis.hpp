#pragma once

#include "iswv/cia402/drive.hpp"
#include "iswv/iswv/faults.hpp"
#include "iswv/iswv/object_dictionary.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>

namespace iswv {

enum class AxisKind : std::uint8_t { travel, steering, generic };

struct AxisConfiguration {
    AxisKind kind{AxisKind::generic};
    std::uint8_t node_id{1};
    std::uint32_t encoder_resolution{65536};
    double gear_ratio{1.0};
    double wheel_diameter_m{0.130};
    bool inverted{false};

    static AxisConfiguration manual_travel(std::uint8_t node_id);
    static AxisConfiguration manual_steering(std::uint8_t node_id);
};

struct DefaultPdoOptions {
    std::uint8_t rpdo_transmission_type{254};
    std::uint8_t tpdo_transmission_type{254};
    std::uint16_t tpdo_inhibit_time{0};
    std::uint16_t tpdo_event_timer_ms{10};
};

enum class TimeoutAction : std::uint8_t { notify_only, quick_stop, disable };

struct SafetyPolicy {
    bool enabled{false};
    bool configure_heartbeat_producer{false};
    std::chrono::milliseconds heartbeat_producer_time{100};
    std::chrono::milliseconds local_heartbeat_timeout{0};
    TimeoutAction timeout_action{TimeoutAction::notify_only};
    bool configure_device_interruption_fault{false};
};

struct HomingConfiguration {
    std::int8_t method{35};
    double switch_velocity_rpm{100.0};
    double zero_velocity_rpm{50.0};
    double acceleration_rps2{50.0};
    std::int32_t home_offset_inc{0};
    std::uint8_t offset_mode{0};
};

struct InterpolationConfiguration {
    std::chrono::milliseconds sync_period{4};
    bool enable_sync_clock{true};
};

struct AxisState {
    std::uint8_t node_id{0};
    bool online{false};
    canopen::NmtState nmt_state{canopen::NmtState::unknown};
    std::uint16_t status_word{0};
    cia402::DriveState drive_state{cia402::DriveState::unknown};
    std::int32_t position_raw{0};
    std::int32_t velocity_raw{0};
    std::int16_t current_raw{0};
    std::uint32_t digital_inputs{0};
    std::uint16_t error_status{0};
    std::uint16_t error_status_2{0};
    std::uint16_t last_emergency_code{0};
    std::uint8_t last_emergency_register{0};
    bool has_process_data{false};
    std::chrono::steady_clock::time_point updated_at{};
};

namespace detail {
class AxisStateImpl;
}

class Axis {
public:
    Axis(std::shared_ptr<canopen::CanopenNode> node, AxisConfiguration configuration);
    ~Axis();

    Axis(const Axis&) = delete;
    Axis& operator=(const Axis&) = delete;
    Axis(Axis&&) noexcept;
    Axis& operator=(Axis&&) noexcept;

    const AxisConfiguration& configuration() const noexcept;
    std::shared_ptr<canopen::CanopenNode> node() const noexcept;
    cia402::Drive& drive() noexcept;
    const cia402::Drive& drive() const noexcept;

    Result<void> configure_default_pdos(DefaultPdoOptions options = {},
                                        canopen::SdoOptions sdo = {});
    Result<void> apply_safety_policy(SafetyPolicy policy,
                                     canopen::SdoOptions sdo = {});

    Result<void> configure_velocity_mode(bool immediate,
                                         double acceleration_rps2,
                                         double deceleration_rps2,
                                         canopen::SdoOptions sdo = {});
    Result<void> command_motor_velocity_rpm(double rpm,
                                            canopen::SdoOptions sdo = {});
    Result<void> command_travel_velocity_mps(double metres_per_second,
                                             canopen::SdoOptions sdo = {});
    Result<void> command_velocity_pdo(double motor_rpm);

    Result<void> configure_position_mode(double profile_velocity_rpm,
                                         double acceleration_rps2,
                                         double deceleration_rps2,
                                         canopen::SdoOptions sdo = {});
    Result<void> move_absolute_inc(std::int32_t target,
                                   canopen::SdoOptions sdo = {});
    Result<void> move_relative_inc(std::int32_t delta,
                                   canopen::SdoOptions sdo = {});
    Result<void> move_continuous_absolute_inc(std::int32_t target,
                                              canopen::SdoOptions sdo = {});
    Result<void> move_steering_angle_rad(double radians,
                                         canopen::SdoOptions sdo = {});
    Result<void> command_position_pdo(std::int32_t target, bool relative);
    Result<void> command_interpolated_position_pdo(std::int32_t target);

    Result<void> configure_torque_mode(canopen::SdoOptions sdo = {});
    Result<void> command_torque_percent(double percent,
                                        canopen::SdoOptions sdo = {});
    Result<void> command_torque_pdo(double percent);

    Result<void> configure_homing(const HomingConfiguration& configuration,
                                  canopen::SdoOptions sdo = {});
    Result<void> start_homing(canopen::SdoOptions sdo = {});
    Result<void> wait_homing(std::chrono::milliseconds timeout,
                             canopen::SdoOptions sdo = {});

    Result<void> configure_interpolation(const InterpolationConfiguration& configuration,
                                         canopen::SdoOptions sdo = {});
    Result<void> start_interpolation(canopen::SdoOptions sdo = {});
    Result<void> stop_interpolation(canopen::SdoOptions sdo = {});
    Result<std::uint16_t> read_sync_lost_count(canopen::SdoOptions sdo = {}) const;

    Result<void> set_direction(bool inverted, canopen::SdoOptions sdo = {});
    Result<void> set_software_limits(std::int32_t negative,
                                     std::int32_t positive,
                                     canopen::SdoOptions sdo = {});
    Result<void> set_stop_options(std::int16_t quick_stop,
                                  std::int16_t shutdown,
                                  std::int16_t disable,
                                  std::int16_t halt,
                                  std::int16_t fault,
                                  canopen::SdoOptions sdo = {});
    Result<void> store_parameters(canopen::SdoOptions sdo = {});
    Result<void> restore_default_parameters(canopen::SdoOptions sdo = {});
    Result<void> set_can_parameters(std::uint16_t new_node_id,
                                    std::uint8_t bitrate_code,
                                    bool store,
                                    canopen::SdoOptions sdo = {});

    Result<AxisState> refresh_state(canopen::SdoOptions sdo = {});
    AxisState state_snapshot() const;
    std::vector<FaultInfo> faults() const;
    Subscription on_state(std::function<void(const AxisState&)> callback);

    Result<std::int32_t> motor_rpm_to_raw(double rpm) const;
    double raw_to_motor_rpm(std::int32_t raw) const noexcept;
    Result<std::uint32_t> acceleration_to_raw(double rps2) const;
    double raw_to_acceleration(std::uint32_t raw) const noexcept;
    Result<std::int32_t> output_revolutions_to_inc(double revolutions) const;
    double inc_to_output_revolutions(std::int32_t increments) const noexcept;
    Result<std::int32_t> steering_radians_to_inc(double radians) const;
    double inc_to_steering_radians(std::int32_t increments) const noexcept;
    Result<double> travel_mps_to_motor_rpm(double metres_per_second) const;
    Result<double> motor_rpm_to_travel_mps(double rpm) const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace iswv
