#include "chassis/steering_config.hpp"
#include "chassis/steering_tuning_persistence.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

void current_vehicle(const chassis::SteeringConfigFile & config)
{
  const auto modules = chassis::resolved_modules(config);
  const std::array<double, 4> x{0.21, -0.21, -0.21, 0.21};
  const std::array<double, 4> y{0.18, 0.18, -0.18, -0.18};
  for (std::size_t i = 0; i < modules.size(); ++i) {
    const auto & m = modules[i];
    require(m.x_m == x[i] && m.y_m == y[i], "module positions changed");
    require(m.wheel_radius_m == 0.065, "wheel radius changed");
    require(
      m.steering_gear_ratio == 9.0252707581 && m.drive_gear_ratio == 10,
      "gear ratios changed");
    require(
      m.steering_encoder_resolution == 65536 && m.drive_encoder_resolution == 65536,
      "encoder counts changed");
    require(
      m.steering_sign == 1 && m.steering_inverted && m.drive_inverted == (i >= 2),
      "motor directions changed");
    require(
      m.heading_offset_rad == -std::acos(-1.0) / 2 &&
      m.steering_min_rad == -std::acos(-1.0) && m.steering_max_rad == std::acos(-1.0),
      "steering geometry changed");
    require(m.soft_margin_rad == 0.0872664626, "soft margin changed");
    require(m.drive_max_motor_rpm == 1050, "effective wheel speed changed");
    require(
      m.drive_acceleration_m_s2 == 2 && m.drive_deceleration_m_s2 == 2.4,
      "effective wheel acceleration/deceleration changed");
    require(m.steering_rate_limit_rad_s == 4, "steering rate changed");
    require(
      config.steering_velocity_loop_kp[i] == 4 && config.drive_velocity_loop_kp[i] == 32,
      "driver tuning changed");
  }
  require(
    config.drive_can_buses == std::array<unsigned int, 4>{1, 1, 2, 2},
    "CAN mapping changed");
  require(
    config.motor.profile_velocity_rpm == 400 && config.motor.acceleration_rps2 == 400 &&
    config.motor.deceleration_rps2 == 400, "steering motor profile changed");
  require(
    config.drive_alignment_policy == "none" && config.swerve_branch_hysteresis_rad == 0,
    "alignment behavior changed");
  require(
    config.calibration_speed_rpm == 500 && config.calibration_search_speed_rpm == 8 &&
    config.calibration_torque_limit_nm == 25 && config.calibration_contact_torque_nm == 1 &&
    config.calibration_timeout_s == 230, "calibration protection changed");
  require(
    config.steering_target_margin_rad == 0.05235987755982989,
    "target safety margin changed");
}

struct TemporaryDirectory
{
  std::filesystem::path path;
  TemporaryDirectory()
  {
    char pattern[] = "/tmp/chassis-config-test-XXXXXX";
    const char * directory = ::mkdtemp(pattern);
    if (!directory) {throw std::runtime_error("cannot create test directory");}
    path = directory;
  }
  ~TemporaryDirectory()
  {
    std::error_code error;
    std::filesystem::remove_all(path, error);
  }
};

void write_yaml(const std::filesystem::path & path, const YAML::Node & document)
{
  std::ofstream file(path);
  file << document;
  require(file.good(), "cannot write test YAML");
}

std::string contents(const std::filesystem::path & path)
{
  std::ifstream file(path);
  std::ostringstream text;
  text << file.rdbuf();
  return text.str();
}

template<typename Function>
void rejects(Function function)
{
  bool rejected = false;
  try {
    function();
  } catch (const std::exception &) {
    rejected = true;
  }
  require(rejected, "invalid configuration was accepted");
}

void split_config_and_persistence(const std::string & source)
{
  TemporaryDirectory directory;
  const auto daily_path = directory.path / "daily.yaml";
  const auto hardware_path = directory.path / "hardware.yaml";
  auto daily = YAML::LoadFile(source);
  auto daily_parameters = chassis::steering_parameters(daily);
  const auto source_hardware = chassis::steering_hardware_config_path(source, daily_parameters);
  require(!source_hardware.empty(), "daily configuration must select hardware file");
  auto hardware = YAML::LoadFile(source_hardware);
  auto hardware_parameters = chassis::steering_parameters(hardware);
  hardware_parameters["steering_rate_limit_rad_s"] = 1.0;
  daily_parameters["hardware_config_file"] = "hardware.yaml";
  write_yaml(hardware_path, hardware);
  write_yaml(daily_path, daily);
  current_vehicle(chassis::load_steering_config(daily_path.string()));

  std::filesystem::create_directory(directory.path / "links");
  const auto symlink = directory.path / "links" / "daily.yaml";
  std::filesystem::create_symlink(daily_path, symlink);
  current_vehicle(chassis::load_steering_config(symlink.string()));

  // Common daily settings reach every wheel after removing duplicate module overrides.
  daily_parameters["drive_max_output_rpm"] = 80.0;
  daily_parameters["drive_acceleration_m_s2"] = 1.3;
  daily_parameters["drive_deceleration_m_s2"] = 1.7;
  write_yaml(daily_path, daily);
  for (const auto & module : chassis::resolved_modules(
      chassis::load_steering_config(daily_path.string())))
  {
    require(
      module.drive_max_motor_rpm == 800 && module.drive_acceleration_m_s2 == 1.3 &&
      module.drive_deceleration_m_s2 == 1.7, "daily wheel settings did not reach all modules");
  }

  // Readback belongs in the hardware file, leaving the daily file byte-identical.
  hardware_parameters["drive_velocity_loop_kp"] = YAML::Node(YAML::NodeType::Null);
  write_yaml(hardware_path, hardware);
  auto config = chassis::load_steering_config(symlink.string());
  config.drive_velocity_loop_kp.fill(35);
  const auto original_daily = contents(daily_path);
  chassis::persist_steering_tuning(config.source_path, config);
  require(contents(daily_path) == original_daily, "hardware readback rewrote daily YAML");
  require(
    chassis::load_steering_config(daily_path.string()).drive_velocity_loop_kp[0] == 35,
    "hardware readback was not saved");
  const auto saved_hardware = contents(hardware_path);
  chassis::persist_steering_tuning(config.source_path, config);
  require(contents(hardware_path) == saved_hardware, "readback save must be idempotent");
  require(std::filesystem::is_symlink(symlink), "save replaced the configuration symlink");

  // Explicit tuning fields in the daily file own their readback; no cross-file corruption.
  daily_parameters["drive_velocity_loop_kp"] = YAML::Node(YAML::NodeType::Null);
  write_yaml(daily_path, daily);
  config = chassis::load_steering_config(daily_path.string());
  config.drive_velocity_loop_kp.fill(40);
  chassis::persist_steering_tuning(config.source_path, config);
  require(contents(hardware_path) == saved_hardware, "daily override corrupted hardware tuning");
  require(
    chassis::load_steering_config(daily_path.string()).drive_velocity_loop_kp[0] == 40,
    "daily tuning override was not persisted to its owner");
  require(
    YAML::LoadFile(daily_path.string())["remote_ctrl"]["ros__parameters"]
    ["scale_linear"].as<double>() == 0.6, "save lost remote settings");

  for (const char * key : {"drive_acceleration_m_s2", "drive_deceleration_m_s2",
      "max_velocity_x_m_s", "max_velocity_y_m_s", "max_velocity_yaw_rad_s"})
  {
    for (const double invalid : {-1.0, std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()})
    {
      auto invalid_document = YAML::Clone(daily);
      chassis::steering_parameters(invalid_document)[key] = invalid;
      write_yaml(daily_path, invalid_document);
      rejects([&] {chassis::load_steering_config(daily_path.string());});
    }
  }
  for (const char * key : {"drive_acceleration_m_s2", "drive_deceleration_m_s2"}) {
    auto invalid_document = YAML::Clone(daily);
    chassis::steering_parameters(invalid_document)[key] = 0.0;
    write_yaml(daily_path, invalid_document);
    rejects([&] {chassis::load_steering_config(daily_path.string());});
  }
  for (const auto & path : {"", "missing.yaml", "daily.yaml"}) {
    daily_parameters["hardware_config_file"] = path;
    write_yaml(daily_path, daily);
    rejects([&] {chassis::load_steering_config(daily_path.string());});
  }
  daily_parameters["hardware_config_file"] = "hardware.yaml";
  write_yaml(daily_path, daily);
  hardware_parameters["hardware_config_file"] = "daily.yaml";
  write_yaml(hardware_path, hardware);
  rejects([&] {chassis::load_steering_config(daily_path.string());});
  hardware_parameters.remove("hardware_config_file");
  hardware_parameters.remove("wheel_base_m");
  write_yaml(hardware_path, hardware);
  rejects([&] {chassis::load_steering_config(daily_path.string());});
}

void legacy_config(const std::string & source)
{
  TemporaryDirectory directory;
  auto daily = YAML::LoadFile(source);
  const auto params = chassis::steering_parameters(daily);
  auto document = YAML::LoadFile(chassis::steering_hardware_config_path(source, params));
  auto legacy = chassis::steering_parameters(document);
  legacy["drive_max_output_rpm"] = 200.0;
  legacy["drive_acceleration_rpm_s"] = 200.0;
  legacy["drive_deceleration_rpm_s"] = 300.0;
  legacy["steering_rate_limit_rad_s"] = 4.0;
  for (const auto & module : chassis::resolved_modules(chassis::load_steering_config(source))) {
    YAML::Node entry;
    entry["x_m"] = module.x_m;
    entry["y_m"] = module.y_m;
    entry["drive_max_motor_rpm"] = 1050.0;
    entry["drive_acceleration_m_s2"] = 2.0;
    entry["drive_deceleration_m_s2"] = 2.4;
    legacy["modules"].push_back(entry);
  }
  const auto path = directory.path / "legacy.yaml";
  write_yaml(path, document);
  auto config = chassis::load_steering_config(path.string());
  current_vehicle(config);
  require(
    config.max_velocity_x_m_s == 0 && config.max_velocity_y_m_s == 0 &&
    config.max_velocity_yaw_rad_s == 0, "legacy commands acquired new caps");
  legacy.remove("modules");
  legacy["steering_velocity_loop_kp"] = YAML::Node(YAML::NodeType::Null);
  write_yaml(path, document);
  config = chassis::load_steering_config(path.string());
  require(
    chassis::resolved_modules(config)[0].drive_acceleration_m_s2 ==
    chassis::drive_output_rpm_s_to_m_s2(200, 0.065), "legacy rpm/s conversion changed");
  config.steering_velocity_loop_kp.fill(5);
  chassis::persist_steering_tuning(config.source_path, config);
  require(
    chassis::load_steering_config(path.string()).steering_velocity_loop_kp[0] == 5,
    "legacy tuning readback no longer persists");
}

void velocity_limits()
{
  chassis::SteeringConfigFile config;
  require(
    chassis::limit_chassis_velocity(2, -3, 4, config) == std::array<double, 3>{2, -3, 4},
    "disabled caps changed command");
  config.max_velocity_x_m_s = 0.5;
  config.max_velocity_y_m_s = 0.3;
  config.max_velocity_yaw_rad_s = 0.4;
  for (const double sign : {-1.0, 1.0}) {
    const auto command = chassis::limit_chassis_velocity(sign, 0.6 * sign, 0.8 * sign, config);
    require(
      command == std::array<double, 3>{0.5 * sign, 0.3 * sign, 0.4 * sign},
      "caps must preserve ratios and apply symmetrically");
  }
  require(
    chassis::limit_chassis_velocity(0.1, 0.2, 0.3, config) ==
    std::array<double, 3>{0.1, 0.2, 0.3}, "in-range command changed");
  require(
    chassis::limit_chassis_velocity(0, 0, 0, config) == std::array<double, 3>{},
    "stop command changed");
  require(
    chassis::limit_chassis_velocity(0.4, 0.2, 1.6, config) ==
    std::array<double, 3>{0.1, 0.05, 0.4}, "yaw cap did not scale translation");
  require(
    chassis::limit_chassis_velocity(0.5, 1.2, 0.4, config) ==
    std::array<double, 3>{0.125, 0.3, 0.1}, "y cap did not scale other axes");
  require(
    chassis::limit_chassis_velocity(1, 0, 0, config)[0] == 0.5,
    "straight forward command bypasses x cap");
  for (const double invalid : {std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()})
  {
    rejects([&] {chassis::limit_chassis_velocity(invalid, 0, 0, config);});
    rejects([&] {chassis::limit_chassis_velocity(0, invalid, 0, config);});
    rejects([&] {chassis::limit_chassis_velocity(0, 0, invalid, config);});
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    require(argc == 2 || argc == 3, "expected reference YAML and optional active YAML path");
    if (argc == 3) {chassis::load_steering_config(argv[2]);}
    current_vehicle(chassis::load_steering_config(argv[1]));
    split_config_and_persistence(argv[1]);
    legacy_config(argv[1]);
    velocity_limits();
    std::cout << "PASS: vehicle values, split/legacy loading, persistence and velocity limits\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
