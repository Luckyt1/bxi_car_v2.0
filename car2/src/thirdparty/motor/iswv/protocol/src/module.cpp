#include "iswv/iswv/module.hpp"

#include <stdexcept>

namespace iswv
{

Module::Module(std::shared_ptr<Axis> travel, std::shared_ptr<Axis> steering)
: travel_(std::move(travel)), steering_(std::move(steering))
{
  if (!travel_ || !steering_) {
    throw std::invalid_argument("iSWV module requires travel and steering axes");
  }
}

std::shared_ptr<Module> Module::from_manual(
  canopen::CanopenMaster & master,
  std::uint8_t travel_node_id,
  std::uint8_t steering_node_id)
{
  if (travel_node_id == steering_node_id) {
    throw std::invalid_argument(
            "travel and steering axes on one CAN bus require distinct Node-IDs");
  }
  auto travel = std::make_shared<Axis>(
    master.node(travel_node_id),
    AxisConfiguration::manual_travel(travel_node_id));
  auto steering = std::make_shared<Axis>(
    master.node(steering_node_id),
    AxisConfiguration::manual_steering(steering_node_id));
  return std::make_shared<Module>(std::move(travel), std::move(steering));
}

std::shared_ptr<Axis> Module::travel() const noexcept
{
  return travel_;
}

std::shared_ptr<Axis> Module::steering() const noexcept
{
  return steering_;
}

Result<void> Module::enter_pre_operational()
{
  auto result = travel_->node()->send_nmt(canopen::NmtCommand::enter_pre_operational);
  if (!result) {
    return result;
  }
  return steering_->node()->send_nmt(canopen::NmtCommand::enter_pre_operational);
}

Result<void> Module::start_network()
{
  auto result = travel_->node()->send_nmt(canopen::NmtCommand::start);
  if (!result) {
    return result;
  }
  return steering_->node()->send_nmt(canopen::NmtCommand::start);
}

Result<void> Module::configure_default_pdos(
  DefaultPdoOptions options,
  canopen::SdoOptions sdo)
{
  auto result = travel_->configure_default_pdos(options, sdo);
  if (!result) {
    return result;
  }
  return steering_->configure_default_pdos(options, sdo);
}

Result<void> Module::enable(cia402::TransitionOptions options)
{
  auto result = travel_->drive().enable(options);
  if (!result) {
    return result;
  }
  result = steering_->drive().enable(options);
  if (!result) {
    (void)travel_->drive().disable(options.sdo);
    return result;
  }
  return Result<void>::success();
}

Result<void> Module::quick_stop(canopen::SdoOptions sdo)
{
  // Stop the travel command first; failure still proceeds to both quick-stop commands.
  auto velocity = travel_->command_motor_velocity_rpm(0.0, sdo);
  auto travel_stop = travel_->drive().quick_stop(sdo);
  auto steering_stop = steering_->drive().quick_stop(sdo);
  if (!velocity) {
    return velocity;
  }
  if (!travel_stop) {
    return travel_stop;
  }
  return steering_stop;
}

Result<void> Module::disable(canopen::SdoOptions sdo)
{
  auto travel_result = travel_->drive().disable(sdo);
  auto steering_result = steering_->drive().disable(sdo);
  if (!travel_result) {
    return travel_result;
  }
  return steering_result;
}

ModuleState Module::state_snapshot() const
{
  return ModuleState{travel_->state_snapshot(), steering_->state_snapshot()};
}

}  // namespace iswv
