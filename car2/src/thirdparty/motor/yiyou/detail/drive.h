#pragma once

#include "motor/yiyou/communication.h"
#include "iswv/cia402/drive.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace chassis::ethercat
{

class Drive
{
public:
  explicit Drive(std::shared_ptr<Node> node)
  : node_(std::move(node))
  {
    if (!node_) {
      throw std::invalid_argument("CiA 402 drive requires an EtherCAT node");
    }
  }

  std::shared_ptr<Node> node() const noexcept {return node_;}

  iswv::Result<std::uint16_t> read_status(
    iswv::canopen::SdoOptions options = {}) const
  {
    return node_->read(iswv::cia402::status_word, options);
  }

  iswv::Result<iswv::cia402::DriveState> state(
    iswv::canopen::SdoOptions options = {}) const
  {
    auto word = read_status(options);
    if (!word) {
      return iswv::Result<iswv::cia402::DriveState>::failure(word.error());
    }
    return iswv::Result<iswv::cia402::DriveState>::success(
      iswv::cia402::decode_state(word.value()));
  }

  iswv::Result<void> write_control(
    std::uint16_t value,
    iswv::canopen::SdoOptions options = {}) const
  {
    return node_->write(iswv::cia402::control_word, value, options);
  }

  iswv::Result<void> set_mode(
    iswv::cia402::OperationMode mode,
    iswv::canopen::SdoOptions options = {}) const
  {
    return node_->write(
      iswv::cia402::modes_of_operation, static_cast<std::int8_t>(mode), options);
  }

  iswv::Result<void> transition_to(
    iswv::cia402::DriveState target,
    iswv::cia402::TransitionOptions options = {}) const
  {
    if (options.timeout.count() <= 0 || options.poll_interval.count() <= 0) {
      return iswv::Result<void>::failure(
        iswv::ErrorCode::invalid_argument,
        "transition timeout and poll interval must be positive");
    }
    const auto deadline = std::chrono::steady_clock::now() + options.timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      auto current_result = state(options.sdo);
      if (!current_result) {
        return iswv::Result<void>::failure(current_result.error());
      }
      const auto current = current_result.value();
      if (current == target) {
        return iswv::Result<void>::success();
      }
      if (current == iswv::cia402::DriveState::fault ||
        current == iswv::cia402::DriveState::fault_reaction_active)
      {
        return iswv::Result<void>::failure(
          iswv::ErrorCode::illegal_state,
          "drive is in a fault state");
      }

      std::uint16_t command = 0;
      bool command_needed = true;
      if (target == iswv::cia402::DriveState::operation_enabled) {
        switch (current) {
          case iswv::cia402::DriveState::switch_on_disabled:
          case iswv::cia402::DriveState::not_ready_to_switch_on:
          case iswv::cia402::DriveState::unknown:
            command = iswv::cia402::control::shutdown;
            break;
          case iswv::cia402::DriveState::ready_to_switch_on:
            command = iswv::cia402::control::switch_on;
            break;
          case iswv::cia402::DriveState::switched_on:
          case iswv::cia402::DriveState::quick_stop_active:
            command = iswv::cia402::control::enable_operation;
            break;
          case iswv::cia402::DriveState::operation_enabled:
          case iswv::cia402::DriveState::fault_reaction_active:
          case iswv::cia402::DriveState::fault:
            command_needed = false;
            break;
        }
      } else if (target == iswv::cia402::DriveState::switched_on) {
        command = current == iswv::cia402::DriveState::switch_on_disabled ?
          iswv::cia402::control::shutdown :
          iswv::cia402::control::switch_on;
      } else if (target == iswv::cia402::DriveState::ready_to_switch_on) {
        command = iswv::cia402::control::shutdown;
      } else if (target == iswv::cia402::DriveState::switch_on_disabled) {
        command = iswv::cia402::control::disable_voltage;
      } else {
        return iswv::Result<void>::failure(
          iswv::ErrorCode::unsupported,
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
    return iswv::Result<void>::failure(
      iswv::ErrorCode::timeout,
      std::string("timeout entering state ") +
      iswv::cia402::state_name(target));
  }

  iswv::Result<void> enable(iswv::cia402::TransitionOptions options = {}) const
  {
    return transition_to(iswv::cia402::DriveState::operation_enabled, options);
  }

  iswv::Result<std::uint16_t> wait_for_status(
    const std::function<bool(std::uint16_t)> & predicate,
    std::chrono::milliseconds timeout,
    std::chrono::milliseconds poll_interval = std::chrono::milliseconds{10},
    iswv::canopen::SdoOptions sdo = {}) const
  {
    if (!predicate || timeout.count() <= 0 || poll_interval.count() <= 0) {
      return iswv::Result<std::uint16_t>::failure(
        iswv::ErrorCode::invalid_argument,
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
    return iswv::Result<std::uint16_t>::failure(
      iswv::ErrorCode::timeout,
      "status condition timed out");
  }

private:
  std::shared_ptr<Node> node_;
};

}  // namespace chassis::ethercat
