#pragma once

#include "chassis/steering_calibration.hpp"
#include "chassis/steering_config.hpp"
#include "iswv/fake_transport.hpp"
#include "iswv/iswv.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <vector>

namespace chassis::test
{

inline iswv::canopen::ObjectAddress request_object(const iswv::CanFrame & request)
{
  return iswv::canopen::ObjectAddress{
    static_cast<std::uint16_t>(
      request.data[1] | (static_cast<std::uint16_t>(request.data[2]) << 8U)),
    request.data[3]};
}

template<typename T, std::size_t Size>
T read_le(const std::array<std::uint8_t, Size> & data, std::size_t offset)
{
  T value{};
  std::memcpy(&value, data.data() + offset, sizeof(T));
  return value;
}

template<typename T, std::size_t Size>
void write_le(std::array<std::uint8_t, Size> & data, std::size_t offset, T value)
{
  std::memcpy(data.data() + offset, &value, sizeof(T));
}

inline iswv::CanFrame sdo_download_response(const iswv::CanFrame & request)
{
  iswv::CanFrame response;
  response.id = 0x580U + (request.id - 0x600U);
  response.size = 8;
  response.data[0] = 0x60;
  response.data[1] = request.data[1];
  response.data[2] = request.data[2];
  response.data[3] = request.data[3];
  return response;
}

template<typename T>
iswv::CanFrame sdo_upload_response(const iswv::CanFrame & request, T value)
{
  iswv::CanFrame response;
  response.id = 0x580U + (request.id - 0x600U);
  response.size = 8;
  response.data[0] = sizeof(T) == 1 ? 0x4F : (sizeof(T) == 2 ? 0x4B : 0x43);
  response.data[1] = request.data[1];
  response.data[2] = request.data[2];
  response.data[3] = request.data[3];
  write_le(response.data, 4, value);
  return response;
}

struct FakeAxis
{
  std::uint16_t status{0x0040};
  std::int8_t mode{0};
  std::int32_t position{0};
  std::int32_t target_position{0};
  std::int32_t velocity_raw{0};
  std::int32_t negative_limit{-100};
  std::int32_t positive_limit{100};
  bool drift_after_midpoint_reached{false};
  bool freeze_position{false};
  bool abort_protection_read{false};
  unsigned nonzero_velocity_frames{0};
  unsigned position_pdo_frames{0};
  unsigned zero_velocity_sdo_writes{0};
  unsigned quick_stops{0};
  unsigned shutdowns{0};
};

struct FakeCan0
{
  std::shared_ptr<iswv::FakeTransport> transport = std::make_shared<iswv::FakeTransport>();
  std::array<FakeAxis, 4> axes{};
  std::vector<std::uint8_t> nonzero_velocity_nodes;
  std::vector<std::uint8_t> zero_velocity_nodes;
  std::vector<std::uint8_t> position_nodes;
  std::array<bool, 4> velocity_active{};
  std::size_t max_simultaneous_velocity_nodes{0};
  bool second_pair_started_before_first_pair_stopped{false};

  FakeCan0()
  {
    axes[0].negative_limit = -100;
    axes[1].negative_limit = -200;
    axes[2].negative_limit = -300;
    axes[3].negative_limit = -400;
    axes[0].positive_limit = 100;
    axes[1].positive_limit = 200;
    axes[2].positive_limit = 300;
    axes[3].positive_limit = 400;
    transport->set_send_hook(
      [this](const iswv::CanFrame & frame) {
        handle(frame);
        publish_feedback();
      });
  }

  void publish_feedback()
  {
    for (std::size_t i = 0; i < axes.size(); ++i) {
      for (unsigned pdo = 1; pdo <= 3; ++pdo) {
        iswv::CanFrame feedback;
        feedback.id = 0x180U + (pdo - 1) * 0x100U + i + 1;
        feedback.size = pdo == 3 ? 8 : 6;
        if (pdo == 1) {
          write_le(feedback.data, 0, axes[i].status);
          write_le(feedback.data, 2, axes[i].position);
        } else if (pdo == 2) {
          write_le(feedback.data, 0, axes[i].velocity_raw);
        }
        transport->inject(feedback);
      }
    }
  }

  chassis::SteeringConfigFile config() const
  {
    chassis::SteeringConfigFile config;
    config.can_bus = 0;
    config.calibration_search_speed_rpm = 1.0;
    config.calibration_speed_rpm = 1000000000.0;
    config.calibration_contact_torque_nm = 0.2;
    config.calibration_torque_limit_nm = 5.0;
    config.calibration_min_limit_separation_rad = 1.0;
    config.calibration_position_epsilon_inc = 0;
    config.calibration_stall_time_s = 0.0;
    config.calibration_timeout_s = 5.0;
    config.calibration_ignore_drive_fault = false;
    config.motor_peak_current_a = 2048.0 * 1.414;
    config.motor_torque_constant_nm_per_a = 1.0;
    config.motor.node_ids = {1, 2, 3, 4};
    config.motor.encoder_resolution = 100;
    config.motor.gear_ratio = 1.0;
    config.motor.wheel_diameter_m = 0.1;
    config.motor.profile_velocity_rpm = 1.0;
    config.motor.acceleration_rps2 = 1.0;
    config.motor.deceleration_rps2 = 1.0;
    config.drive_can_buses = {1, 1, 2, 2};
    config.drive_node_ids = {1, 2, 1, 2};
    config.drive_encoder_resolution = 100;
    config.drive_gear_ratio = 1.0;
    return config;
  }

  void handle(const iswv::CanFrame & frame)
  {
    if (frame.id == 0) {
      return;
    }
    if (frame.id >= 0x601U && frame.id <= 0x604U) {
      handle_sdo(frame, static_cast<std::uint8_t>(frame.id - 0x600U));
      return;
    }
    if (frame.id >= 0x201U && frame.id <= 0x204U) {
      handle_velocity_pdo(frame, static_cast<std::uint8_t>(frame.id - 0x200U));
      return;
    }
    if (frame.id >= 0x301U && frame.id <= 0x304U) {
      handle_position_pdo(frame, static_cast<std::uint8_t>(frame.id - 0x300U));
    }
  }

  void handle_sdo(const iswv::CanFrame & frame, std::uint8_t node_id)
  {
    auto & axis = axes[node_id - 1U];
    const auto object = request_object(frame);
    if (frame.data[0] == 0x40) {
      inject_upload(frame, axis, object);
      return;
    }
    if (object == iswv::objects::control_word.address) {
      const auto command = read_le<std::uint16_t>(frame.data, 4);
      if (command == iswv::cia402::control::disable_voltage) {
        axis.status = 0x0040;
      } else if (command == iswv::cia402::control::shutdown) {
        ++axis.shutdowns;
        axis.status = 0x0031;
      } else if (command == iswv::cia402::control::switch_on) {
        axis.status = 0x0033;
      } else if (command == iswv::cia402::control::enable_operation) {
        axis.status = 0x0037;
      } else if (command == iswv::cia402::control::quick_stop) {
        ++axis.quick_stops;
        axis.status = 0x0037;
      }
    } else if (object == iswv::objects::mode.address) {
      axis.mode = read_le<std::int8_t>(frame.data, 4);
    } else if (object == iswv::objects::target_velocity.address) {
      axis.velocity_raw = read_le<std::int32_t>(frame.data, 4);
      if (axis.velocity_raw == 0) {
        velocity_active[node_id - 1U] = false;
        ++axis.zero_velocity_sdo_writes;
        zero_velocity_nodes.push_back(node_id);
      }
    }
    transport->inject(sdo_download_response(frame));
  }

  void inject_upload(
    const iswv::CanFrame & frame, FakeAxis & axis,
    const iswv::canopen::ObjectAddress & object)
  {
    if (object == iswv::objects::status_word.address) {
      transport->inject(sdo_upload_response(frame, axis.status));
    } else if (object == iswv::objects::mode.address) {
      transport->inject(sdo_upload_response(frame, axis.mode));
    } else if (object == iswv::objects::actual_position.address) {
      step_position(axis);
      transport->inject(sdo_upload_response(frame, axis.position));
    } else if (object == iswv::objects::actual_current.address) {
      const bool at_negative = axis.position <= axis.negative_limit;
      const bool at_positive = axis.position >= axis.positive_limit;
      const std::int16_t current = (at_negative || at_positive) ? 1 : 0;
      transport->inject(sdo_upload_response(frame, current));
    } else if (object == iswv::objects::error_status.address ||
      object == iswv::objects::error_status_2.address)
    {
      if (axis.abort_protection_read) {
        auto response = sdo_download_response(frame);
        response.data[0] = 0x80;
        write_le(response.data, 4, std::uint32_t{0x06020000});
        transport->inject(response);
      } else {
        transport->inject(sdo_upload_response(frame, std::uint16_t{0}));
      }
    } else if (object == iswv::objects::actual_velocity.address) {
      transport->inject(sdo_upload_response(frame, axis.velocity_raw));
    } else if (object == iswv::objects::digital_inputs.address) {
      transport->inject(sdo_upload_response(frame, std::uint32_t{0}));
    } else {
      transport->inject(sdo_upload_response(frame, std::uint32_t{0}));
    }
  }

  void step_position(FakeAxis & axis)
  {
    if (axis.mode == static_cast<std::int8_t>(iswv::cia402::OperationMode::profile_position)) {
      if (axis.freeze_position) {return;}
      if (axis.drift_after_midpoint_reached) {
        axis.position = axis.target_position + 1;
        return;
      }
      axis.position = axis.target_position;
      return;
    }
    if (axis.velocity_raw < 0 && axis.position > axis.negative_limit) {
      axis.position = std::max(axis.position - 100, axis.negative_limit);
    } else if (axis.velocity_raw > 0 && axis.position < axis.positive_limit) {
      axis.position = std::min(axis.position + 100, axis.positive_limit);
    }
  }

  void handle_velocity_pdo(const iswv::CanFrame & frame, std::uint8_t node_id)
  {
    auto & axis = axes[node_id - 1U];
    axis.status = 0x0037;
    axis.mode = read_le<std::int8_t>(frame.data, 2);
    axis.velocity_raw = read_le<std::int32_t>(frame.data, 3);
    if (axis.velocity_raw != 0) {
      if (node_id >= 3 && (velocity_active[0] || velocity_active[1])) {
        second_pair_started_before_first_pair_stopped = true;
      }
      velocity_active[node_id - 1U] = true;
      max_simultaneous_velocity_nodes = std::max(
        max_simultaneous_velocity_nodes,
        static_cast<std::size_t>(std::count(velocity_active.begin(), velocity_active.end(), true)));
      ++axis.nonzero_velocity_frames;
      nonzero_velocity_nodes.push_back(node_id);
    }
  }

  void handle_position_pdo(const iswv::CanFrame & frame, std::uint8_t node_id)
  {
    auto & axis = axes[node_id - 1U];
    axis.status = 0x0037;
    axis.target_position = read_le<std::int32_t>(frame.data, 2);
    ++axis.position_pdo_frames;
    if (axis.position_pdo_frames % 2 == 0 && !axis.freeze_position) {step_position(axis);}
    position_nodes.push_back(node_id);
  }
};

inline std::array<std::int32_t, 4> run_calibration(
  FakeCan0 & fake, const std::function<bool()> & running)
{
  iswv::canopen::CanopenMaster master(fake.transport);
  auto config = fake.config();
  iswv::SteeringLayout steering(master, config.motor);
  return chassis::calibrate_steering(steering, config, running);
}

}  // namespace chassis::test
