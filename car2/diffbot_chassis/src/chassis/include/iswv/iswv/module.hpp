#pragma once

#include "iswv/iswv/axis.hpp"

#include <memory>

namespace iswv
{

struct ModuleState
{
  AxisState travel;
  AxisState steering;
};

class Module
{
public:
  Module(std::shared_ptr<Axis> travel, std::shared_ptr<Axis> steering);

  static std::shared_ptr<Module> from_manual(
    canopen::CanopenMaster & master,
    std::uint8_t travel_node_id,
    std::uint8_t steering_node_id);

  std::shared_ptr<Axis> travel() const noexcept;
  std::shared_ptr<Axis> steering() const noexcept;

  Result<void> enter_pre_operational();
  Result<void> start_network();
  Result<void> configure_default_pdos(
    DefaultPdoOptions options = {},
    canopen::SdoOptions sdo = {});
  Result<void> enable(cia402::TransitionOptions options = {});
  Result<void> quick_stop(canopen::SdoOptions sdo = {});
  Result<void> disable(canopen::SdoOptions sdo = {});
  ModuleState state_snapshot() const;

private:
  std::shared_ptr<Axis> travel_;
  std::shared_ptr<Axis> steering_;
};

}  // namespace iswv
