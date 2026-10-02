#pragma once

#include "iswv/canopen/types.hpp"

#include <cstdint>

namespace iswv::objects
{

using canopen::Object;

// CANopen and communication objects.
inline constexpr Object<std::uint32_t> sync_cob_id{{0x1005, 0x00}, "SYNC COB-ID"};
inline constexpr Object<std::uint16_t> guard_time{{0x100C, 0x00}, "Guard time"};
inline constexpr Object<std::uint8_t> life_time_factor{{0x100D, 0x00},
  "Life time factor"};
inline constexpr Object<std::uint32_t> guard_cob_id{{0x100E, 0x00},
  "Node guard COB-ID"};
inline constexpr Object<std::uint32_t> emergency_cob_id{{0x1014, 0x00},
  "Emergency COB-ID"};
inline constexpr Object<std::uint16_t> heartbeat_producer_time{{0x1017, 0x00},
  "Heartbeat producer time"};
inline constexpr Object<std::uint16_t> device_node_id{{0x100B, 0x00}, "Device Node-ID"};
inline constexpr Object<std::uint8_t> can_bitrate_code{{0x2F81, 0x00},
  "CAN bitrate code"};
inline constexpr Object<std::uint16_t> communication_interruption_mode{
  {0x6007, 0x00}, "Communication interruption mode"};

// CiA 402 control and stop behavior.
inline constexpr Object<std::uint16_t> control_word{{0x6040, 0x00}, "Control word"};
inline constexpr Object<std::uint16_t> status_word{{0x6041, 0x00}, "Status word"};
inline constexpr Object<std::int16_t> quick_stop_option{{0x605A, 0x00},
  "Quick stop option"};
inline constexpr Object<std::int16_t> shutdown_option{{0x605B, 0x00},
  "Shutdown option"};
inline constexpr Object<std::int16_t> disable_operation_option{
  {0x605C, 0x00}, "Disable operation option"};
inline constexpr Object<std::int16_t> halt_option{{0x605D, 0x00}, "Halt option"};
inline constexpr Object<std::int16_t> fault_reaction_option{{0x605E, 0x00},
  "Fault reaction option"};
inline constexpr Object<std::int8_t> mode{{0x6060, 0x00}, "Mode of operation"};

// Feedback, command and profile objects.
inline constexpr Object<std::int32_t> actual_position{{0x6063, 0x00},
  "Actual position"};
inline constexpr Object<std::uint32_t> maximum_following_error{{0x6065, 0x00},
  "Maximum following error"};
inline constexpr Object<std::uint32_t> position_window{{0x6067, 0x00},
  "Position window"};
inline constexpr Object<std::int32_t> actual_velocity{{0x606C, 0x00},
  "Actual velocity"};
inline constexpr Object<std::int16_t> target_torque{{0x6071, 0x00}, "Target torque"};
inline constexpr Object<std::uint16_t> target_current_limit{{0x6073, 0x00},
  "Target current limit"};
inline constexpr Object<std::int16_t> actual_current{{0x6078, 0x00}, "Actual current"};
inline constexpr Object<std::int32_t> target_position{{0x607A, 0x00},
  "Target position"};
inline constexpr Object<std::int32_t> home_offset{{0x607C, 0x00}, "Home offset"};
inline constexpr Object<std::int32_t> positive_software_limit{{0x607D, 0x01},
  "Positive software limit"};
inline constexpr Object<std::int32_t> negative_software_limit{{0x607D, 0x02},
  "Negative software limit"};
inline constexpr Object<std::uint8_t> direction{{0x607E, 0x00}, "Direction"};
inline constexpr Object<std::uint32_t> maximum_profile_velocity{{0x607F, 0x00},
  "Maximum profile velocity"};
inline constexpr Object<std::uint16_t> maximum_motor_speed{{0x6080, 0x00},
  "Maximum motor speed"};
inline constexpr Object<std::uint32_t> profile_velocity{{0x6081, 0x00},
  "Profile velocity"};
inline constexpr Object<std::uint32_t> profile_acceleration{{0x6083, 0x00},
  "Profile acceleration"};
inline constexpr Object<std::uint32_t> profile_deceleration{{0x6084, 0x00},
  "Profile deceleration"};
inline constexpr Object<std::uint32_t> quick_stop_deceleration{{0x6085, 0x00},
  "Quick stop deceleration"};
inline constexpr Object<std::int32_t> target_velocity{{0x60FF, 0x00},
  "Target velocity"};
inline constexpr Object<std::int16_t> target_current{{0x60F6, 0x08}, "Target current"};
inline constexpr Object<std::uint32_t> digital_inputs{{0x60FD, 0x00},
  "Digital inputs"};

// Homing.
inline constexpr Object<std::int8_t> homing_method{{0x6098, 0x00}, "Homing method"};
inline constexpr Object<std::uint32_t> homing_switch_velocity{{0x6099, 0x01},
  "Homing switch velocity"};
inline constexpr Object<std::uint32_t> homing_zero_velocity{{0x6099, 0x02},
  "Homing zero velocity"};
inline constexpr Object<std::uint8_t> power_on_homing{{0x6099, 0x03},
  "Power-on homing"};
inline constexpr Object<std::int16_t> homing_maximum_current{{0x6099, 0x04},
  "Homing maximum current"};
inline constexpr Object<std::uint8_t> homing_offset_mode{{0x6099, 0x05},
  "Homing offset mode"};
inline constexpr Object<std::uint8_t> homing_index_blind_zone{{0x6099, 0x06},
  "Homing index blind zone"};
inline constexpr Object<std::uint32_t> homing_acceleration{{0x609A, 0x00},
  "Homing acceleration"};

// I/O and multi-segment control.
inline constexpr Object<std::uint16_t> input_polarity{{0x2010, 0x01},
  "Input polarity"};
inline constexpr Object<std::uint16_t> input_simulation{{0x2010, 0x02},
  "Input simulation"};
inline constexpr Object<std::uint16_t> digital_input_1{{0x2010, 0x03}, "Digital input 1"};
inline constexpr Object<std::uint16_t> digital_input_2{{0x2010, 0x04}, "Digital input 2"};
inline constexpr Object<std::uint16_t> digital_input_3{{0x2010, 0x05}, "Digital input 3"};
inline constexpr Object<std::uint16_t> digital_input_4{{0x2010, 0x06}, "Digital input 4"};
inline constexpr Object<std::uint16_t> input_state{{0x2010, 0x0A}, "Input state"};
inline constexpr Object<std::uint16_t> output_polarity{{0x2010, 0x0D},
  "Output polarity"};
inline constexpr Object<std::uint16_t> output_simulation{{0x2010, 0x0E},
  "Output simulation"};
inline constexpr Object<std::uint16_t> digital_output_1{{0x2010, 0x0F}, "Digital output 1"};
inline constexpr Object<std::uint16_t> digital_output_2{{0x2010, 0x10}, "Digital output 2"};
inline constexpr Object<std::uint16_t> output_state{{0x2010, 0x14}, "Output state"};
inline constexpr Object<std::uint16_t> zero_speed_window{{0x2010, 0x18},
  "Zero speed window"};
inline constexpr Object<std::uint8_t> limit_function{{0x2010, 0x19}, "Limit function"};
inline constexpr Object<std::int32_t> positive_limit_capture{{0x2010, 0x1B},
  "Positive limit captured position"};
inline constexpr Object<std::int32_t> negative_limit_capture{{0x2010, 0x1C},
  "Negative limit captured position"};
inline constexpr Object<std::uint16_t> absolute_relative_selection{{0x2020, 0x0F},
  "Absolute/relative selection"};

inline constexpr Object<std::int32_t> segment_position(std::uint8_t segment)
{
  return Object<std::int32_t>{{0x2020,
    segment < 4 ? static_cast<std::uint8_t>(segment + 1U) :
    static_cast<std::uint8_t>(segment + 0x0CU)},
    "Segment position"};
}

inline constexpr Object<std::int32_t> segment_velocity(std::uint8_t segment)
{
  return Object<std::int32_t>{{0x2020,
    segment < 4 ? static_cast<std::uint8_t>(segment + 5U) :
    static_cast<std::uint8_t>(segment + 0x10U)},
    "Segment velocity"};
}

// Persistence, faults, tuning and monitoring.
inline constexpr Object<std::uint8_t> store_control_parameters{{0x2FF0, 0x01},
  "Store control parameters"};
inline constexpr Object<std::uint8_t> store_motor_parameters{{0x2FF0, 0x03},
  "Store motor parameters"};
inline constexpr Object<std::uint16_t> error_status{{0x2601, 0x00}, "Error status"};
inline constexpr Object<std::uint16_t> error_status_2{{0x2602, 0x00}, "Error status 2"};
inline constexpr Object<std::uint16_t> velocity_loop_kp{{0x60F9, 0x01},
  "Velocity loop Kp"};
inline constexpr Object<std::uint16_t> velocity_loop_ki{{0x60F9, 0x02},
  "Velocity loop Ki"};
inline constexpr Object<std::uint8_t> velocity_feedback_filter{{0x60F9, 0x05},
  "Velocity feedback filter"};
inline constexpr Object<std::uint16_t> velocity_loop_ki_div32{{0x60F9, 0x07},
  "Velocity loop Ki / 32"};
inline constexpr Object<std::int32_t> velocity_window{{0x60F9, 0x0A},
  "Velocity window"};
inline constexpr Object<std::uint16_t> zero_speed_time{{0x60F9, 0x14},
  "Zero speed time"};
inline constexpr Object<std::int32_t> following_error_actual{{0x60F4, 0x00},
  "Following error actual"};
inline constexpr Object<std::int32_t> position_demand{{0x60FC, 0x00},
  "Position demand"};
inline constexpr Object<std::int16_t> torque_reached_threshold{{0x60F5, 0x06},
  "Torque reached threshold"};
inline constexpr Object<std::int16_t> torque_reached_filter_time{{0x60F5, 0x07},
  "Torque reached filter time"};
inline constexpr Object<std::int16_t> torque_reached_actual{{0x60F5, 0x08},
  "Torque reached actual"};
inline constexpr Object<std::int16_t> position_loop_kp{{0x60FB, 0x01},
  "Position loop Kp"};
inline constexpr Object<std::int16_t> position_velocity_feedforward{{0x60FB, 0x02},
  "Position velocity feedforward"};
inline constexpr Object<std::int16_t> position_acceleration_feedforward{
  {0x60FB, 0x03}, "Position acceleration feedforward"};
inline constexpr Object<std::uint16_t> position_smoothing_filter{{0x60FB, 0x05},
  "Position smoothing filter"};
inline constexpr Object<std::uint16_t> position_reached_time_window{{0x2508, 0x09},
  "Position reached time window"};

// Brake, temperature and voltage alarm parameters described by the manual.
inline constexpr Object<std::uint16_t> brake_duty_cycle{{0x6410, 0x11},
  "Brake duty cycle"};
inline constexpr Object<std::uint8_t> motor_accessory{{0x6410, 0x17},
  "Motor accessory"};
inline constexpr Object<std::int16_t> motor_temperature_alarm{{0x6410, 0x18},
  "Motor temperature alarm"};
inline constexpr Object<std::int16_t> motor_temperature{{0x6410, 0x19},
  "Motor temperature"};
inline constexpr Object<std::uint16_t> under_voltage_alarm{{0x6510, 0x07},
  "Under-voltage alarm"};
inline constexpr Object<std::uint16_t> chopper_voltage{{0x6510, 0x08},
  "Chopper voltage"};
inline constexpr Object<std::uint16_t> over_voltage_alarm{{0x6510, 0x09},
  "Over-voltage alarm"};

// iSWV ECAN interpolation extensions.
inline constexpr Object<std::uint8_t> ecan_sync_period{{0x3011, 0x01},
  "ECAN sync period"};
inline constexpr Object<std::uint8_t> ecan_sync_clock_mode{{0x3011, 0x02},
  "ECAN sync clock mode"};
inline constexpr Object<std::uint16_t> ecan_sync_lost_count{{0x3011, 0x04},
  "ECAN sync lost count"};

}  // namespace iswv::objects
