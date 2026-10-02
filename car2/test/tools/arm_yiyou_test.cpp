#include "bxi/board.h"
#include "motor/yiyou/communication.h"
#include "motor/yiyou/motor.h"

#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{
using namespace std::chrono_literals;
volatile std::sig_atomic_t stop_requested = 0;

#ifdef YIYOU_ZERO_TEST_FAST_WAIT
// 仅供链接假 PCI 驱动的离线测试使用。
constexpr auto power_on_wait = 100ms;
constexpr auto power_off_wait = 100ms;
constexpr auto save_wait = 100ms;
#else
constexpr auto power_on_wait = 4s;
constexpr auto power_off_wait = 2s;
constexpr auto save_wait = 2s;
#endif

void request_stop(int)
{
  stop_requested = 1;
}

void check_cancelled()
{
  if (stop_requested) {throw std::runtime_error("test interrupted");}
}

void wait_power(std::chrono::milliseconds duration)
{
  const auto ready = std::chrono::steady_clock::now() + duration;
  while (std::chrono::steady_clock::now() < ready) {
    check_cancelled();
    std::this_thread::sleep_for(20ms);
  }
  check_cancelled();
}

void power(chassis::BxiPciTransport & transport, bool enabled)
{
  const auto result = transport.set_motor_power(enabled);
  if (!result) {throw std::runtime_error(result.error().message);}
  std::cout << (enabled ? "共享电机电源开启命令已提交。\n" : "共享电机电源关闭命令已提交。\n")
            << std::flush;
}

struct Options
{
  std::string interface;
  std::uint16_t slave{1};
  unsigned int power_bus{3};
  bool info_only{false};
  bool zero_cycle{false};
  bool use_degrees{false};
  double degrees{0};
  std::int32_t pulses{0};
  double speed_rpm{1};
  double acceleration_rpm_s{2};
  double timeout_s{30};
  std::uint32_t position_tolerance_pulses{100};
};

void usage()
{
  std::cout <<
    "Usage: arm_yiyou_test --interface NIC --slave 1..199 [--power-bus 3]\n"
    "       (--info-only | --degrees DEG | --pulses COUNT | --zero-cycle) [options]\n"
    "Power on, read Yiyou motor information, then run the selected test.\n"
    "  --degrees DEG          Relative output-shaft angle from startup position\n"
    "  --pulses COUNT         Relative raw position pulses (signed int32)\n"
    "  --zero-cycle           Set persistent zero, move +60 deg, power-cycle, return to zero\n"
    "  --speed-rpm RPM        Positive output speed (default: 1)\n"
    "  --acceleration-rpm-s A Positive acceleration/deceleration (default: 2)\n"
    "  --timeout SEC         Motion timeout (default: 30, maximum: 3600)\n"
    "  --position-tolerance-pulses N  Allowed final position error (default: 100)\n"
    "  --info-only           Read information without enabling the motor\n"
    "Run exclusively: this test controls the shared motor power through BXI CAN power only.\n"
    "--zero-cycle overwrites the motor zero and saves all current saveable parameters.\n"
    "Normal completion, errors and Ctrl+C stop the motor and switch power off.\n"
    "Help does not open EtherCAT or CAN devices.\n";
}

double real_number(const std::string & text)
{
  std::size_t end = 0;
  const double value = std::stod(text, &end);
  if (end != text.size() || !std::isfinite(value)) {
    throw std::invalid_argument("expected a finite number: " + text);
  }
  return value;
}

std::int64_t integer(const std::string & text, std::int64_t low, std::int64_t high)
{
  std::size_t end = 0;
  const auto value = std::stoll(text, &end, 10);
  if (end != text.size() || value < low || value > high) {
    throw std::invalid_argument("integer outside allowed range: " + text);
  }
  return value;
}

Options parse(int argc, char ** argv)
{
  Options options;
  std::set<std::string> seen;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (!seen.insert(key).second) {throw std::invalid_argument("duplicate option: " + key);}
    if (key == "--info-only") {options.info_only = true; continue;}
    if (key == "--zero-cycle") {options.zero_cycle = true; continue;}
    if (key == "--bus" || key == "--node" || key == "--nodes") {
      throw std::invalid_argument(
              key + " is a retired CAN option; use --interface NIC and --slave position");
    }
    if (++i == argc) {throw std::invalid_argument("missing value for " + key);}
    const std::string value = argv[i];
    if (key == "--interface") {
      if (value.empty()) {throw std::invalid_argument("--interface must not be empty");}
      options.interface = value;
    } else if (key == "--slave") {
      options.slave = static_cast<std::uint16_t>(integer(value, 1, 199));
    } else if (key == "--power-bus") {
      options.power_bus = static_cast<unsigned int>(integer(value, 0, CANFD_DEVICE_NUM - 1));
    } else if (key == "--degrees") {
      options.use_degrees = true;
      options.degrees = real_number(value);
    } else if (key == "--pulses") {
      options.pulses = static_cast<std::int32_t>(integer(
          value, std::numeric_limits<std::int32_t>::min(),
          std::numeric_limits<std::int32_t>::max()));
    } else if (key == "--speed-rpm") {
      options.speed_rpm = real_number(value);
    } else if (key == "--acceleration-rpm-s") {
      options.acceleration_rpm_s = real_number(value);
    } else if (key == "--timeout") {
      options.timeout_s = real_number(value);
    } else if (key == "--position-tolerance-pulses") {
      options.position_tolerance_pulses = static_cast<std::uint32_t>(integer(
          value, 0, std::numeric_limits<std::uint32_t>::max()));
    } else {
      throw std::invalid_argument("unknown option: " + key);
    }
  }
  if (options.interface.empty() || seen.count("--slave") == 0) {
    throw std::invalid_argument("--interface and --slave are required");
  }
  if (seen.count("--info-only") + seen.count("--degrees") + seen.count("--pulses") +
    seen.count("--zero-cycle") != 1)
  {
    throw std::invalid_argument(
            "choose exactly one of --info-only, --degrees, --pulses, --zero-cycle");
  }
  if (options.speed_rpm <= 0 || options.acceleration_rpm_s <= 0 ||
    options.timeout_s <= 0 || options.timeout_s > 3600)
  {
    throw std::invalid_argument("speed/acceleration must be positive; timeout must be in (0, 3600]");
  }
  chassis::ethercat::validate_selection({options.interface, {options.slave}});
  return options;
}

void move_and_wait(
  chassis::YiyouMotor & motor, const Options & options, std::int32_t target, double scale)
{
  const auto start = motor.actual_position();
  const auto speed = motor.velocity_from_rpm(options.speed_rpm);
  const auto acceleration = motor.velocity_from_rpm(options.acceleration_rpm_s);
  std::cout << "输出轴标尺=" << scale << " pulse/rev；相对位移="
            << static_cast<std::int64_t>(target) - start
            << " pulse；目标=" << target << " pulse；到位容差="
            << options.position_tolerance_pulses << " pulse\n" << std::flush;
  check_cancelled();
  motor.enable_position(
    static_cast<std::uint32_t>(speed), static_cast<std::uint32_t>(acceleration),
    static_cast<std::uint32_t>(acceleration));
  check_cancelled();
  std::cout << "已确认 CoE 控制源 (0x2100=1) 和 PP 位置模式 (0x6061=1)，准备发送目标。\n"
            << std::flush;
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(options.timeout_s);
  motor.move_to_position(target);
  std::cout << "目标已确认接收，等待实际位置到达。\n" << std::flush;
  auto next_report = std::chrono::steady_clock::now();
  auto position = start;
  while (true) {
    check_cancelled();
    if (std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error(
              "PP motion timed out before target reached: target=" + std::to_string(target) +
              ", last actual=" + std::to_string(position) +
              ", error=" + std::to_string(static_cast<std::int64_t>(target) - position) +
              " pulse, tolerance=" + std::to_string(options.position_tolerance_pulses));
    }
    const bool reached = motor.position_reached(options.position_tolerance_pulses);
    position = motor.actual_position();
    if (reached || std::chrono::steady_clock::now() >= next_report) {
      std::cout << "位置=" << position << " pulse；位移="
                << (static_cast<double>(position) - start) * 360.0 / scale
                << " deg；剩余误差=" << static_cast<std::int64_t>(target) - position
                << " pulse；速度=" << motor.actual_velocity_rpm() << " rpm\n" << std::flush;
      next_report = std::chrono::steady_clock::now() + 200ms;
    }
    if (reached) {
      std::cout << "目标已到达（0x6041 bit10=1，实际位置已进入目标容差）。\n";
      return;
    }
    std::this_thread::sleep_for(20ms);
  }
}

void test_motor(chassis::YiyouMotor & motor, const Options & options)
{
  const auto start = motor.actual_position();
  std::cout << "当前位置=" << start << " pulse\n";
  if (options.info_only) {return;}
  const auto scale = motor.configure_rpm_units();
  const auto offset = options.use_degrees ?
    motor.position_from_degrees(options.degrees) : options.pulses;
  const auto target = static_cast<std::int64_t>(start) + offset;
  if (target < std::numeric_limits<std::int32_t>::min() ||
    target > std::numeric_limits<std::int32_t>::max())
  {
    throw std::out_of_range("current position + requested offset exceeds int32 position range");
  }
  move_and_wait(motor, options, static_cast<std::int32_t>(target), scale);
}

std::array<std::uint32_t, 4> identity(chassis::ethercat::Master & master, std::uint16_t id)
{
  std::array<std::uint32_t, 4> fields{};
  for (std::uint8_t sub = 1; sub <= fields.size(); ++sub) {
    const auto result = master.node(id)->read(
      iswv::canopen::Object<std::uint32_t>{{0x1018, sub}, "identity"});
    if (!result) {throw std::runtime_error("read identity: " + result.error().message);}
    fields[sub - 1] = result.value();
  }
  return fields;
}

void test_zero_cycle(
  std::unique_ptr<chassis::YiyouMotor> & motor, std::unique_ptr<chassis::ethercat::Master> & master,
  chassis::BxiPciTransport & transport, const Options & options, bool & save_pending)
{
  const auto before_identity = identity(*master, options.slave);
  const auto scale = motor->configure_rpm_units();
  const auto target = motor->position_from_degrees(60);
  const auto tolerance = static_cast<std::int64_t>(options.position_tolerance_pulses);
  if (tolerance * 2 >= target) {
    throw std::invalid_argument("zero-cycle tolerance must be less than half of the 60-degree move");
  }
  // 在改零点之前校验速度/加速度的标尺和数值范围。
  (void)motor->velocity_from_rpm(options.speed_rpm);
  (void)motor->velocity_from_rpm(options.acceleration_rpm_s);
  check_cancelled();
  std::cout << "设置当前位置为零点，并保存至 EEPROM（会保存全部可持久化参数）。\n" << std::flush;
  save_pending = true;  // SDO 失败也可能已经写入；退出路径仍须留出保存时间。
  motor->save_zero_position(options.position_tolerance_pulses);
  std::this_thread::sleep_for(save_wait);  // 保存期间收到退出信号也不能立即断电。
  save_pending = false;
  check_cancelled();
  std::cout << "零点已确认并提交保存，开始正转 60°。\n" << std::flush;
  move_and_wait(*motor, options, target, scale);
  motor->stop();
  const auto before_power_off = motor->actual_position();
  if (std::abs(static_cast<std::int64_t>(before_power_off) - target) > tolerance) {
    throw std::runtime_error("position drifted after stopping at 60 degrees");
  }
  motor.reset();  // 断电前释放驱动；重新上电不沿用 enabled/target 等主机状态。
  master.reset();  // 停止 PDO 线程并释放独占主站，再断电和重新建站。
  check_cancelled();
  power(transport, false);
  wait_power(power_off_wait);
  power(transport, true);
  wait_power(power_on_wait);

  master = std::make_unique<chassis::ethercat::Master>(
    chassis::ethercat::Options{options.interface, {options.slave}});
  motor = std::make_unique<chassis::YiyouMotor>(*master, options.slave);
  if (identity(*master, options.slave) != before_identity) {
    throw std::runtime_error("motor identity changed after power cycle; return-to-zero cancelled");
  }
  if (motor->configure_rpm_units() != scale) {
    throw std::runtime_error("position scale changed after power cycle; return-to-zero cancelled");
  }
  const auto after_power_on = motor->actual_position();
  std::cout << "断电前位置=" << before_power_off << "，重新上电位置=" << after_power_on
            << " pulse\n" << std::flush;
  if (std::abs(static_cast<std::int64_t>(after_power_on) - before_power_off) > tolerance) {
    throw std::runtime_error(
            "position was not retained across power cycle (or shaft moved while unpowered); "
            "return-to-zero cancelled");
  }
  check_cancelled();
  std::cout << "重启后身份、标尺和位置检查通过，返回保存的零点。\n" << std::flush;
  move_and_wait(*motor, options, 0, scale);
  motor->stop();
  if (std::abs(static_cast<std::int64_t>(motor->actual_position())) > tolerance) {
    throw std::runtime_error("position drifted after stopping at zero");
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc == 2 && std::string(argv[1]) == "--help") {usage(); return 0;}
  Options options;
  try {
    options = parse(argc, argv);
  } catch (const std::exception & error) {
    std::cerr << "Arguments: " << error.what() << '\n';
    usage();
    return 2;
  }

  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  std::signal(SIGHUP, request_stop);
  int result = 0;
  bool save_pending = false;
  std::shared_ptr<chassis::BxiPciTransport> transport;
  std::unique_ptr<chassis::ethercat::Master> master;
  std::unique_ptr<chassis::YiyouMotor> motor;
  try {
    check_cancelled();
    transport = std::make_shared<chassis::BxiPciTransport>(options.power_bus);
    power(*transport, true);
    std::cout << "共享电机电源已开启；EtherCAT " << options.interface << " slave "
              << static_cast<unsigned>(options.slave) << "，等待电源稳定。\n" << std::flush;
    wait_power(power_on_wait);
    master = std::make_unique<chassis::ethercat::Master>(
      chassis::ethercat::Options{options.interface, {options.slave}});
    motor = std::make_unique<chassis::YiyouMotor>(*master, options.slave);
    check_cancelled();
    std::cout << "上电诊断（切换控制源和模式前）：\n"
              << motor->diagnostic_report() << std::flush;
    check_cancelled();
    if (options.zero_cycle) {
      test_zero_cycle(motor, master, *transport, options, save_pending);
    } else {
      test_motor(*motor, options);
    }
  } catch (const std::exception & error) {
    std::cerr << "Yiyou arm test: " << error.what() << '\n';
    result = 1;
  }

  if (save_pending) {std::this_thread::sleep_for(save_wait);}
  if (motor) {
    try {
      motor->stop();
    } catch (const std::exception & error) {
      std::cerr << "Stop failed: " << error.what() << '\n';
      result = 1;
    }
    motor.reset();
  }
  master.reset();
  if (transport) {
    const auto power = transport->set_motor_power(false);
    if (!power) {
      std::cerr << "Power off failed: " << power.error().message << '\n';
      result = 1;
    } else {
      std::cout << "共享电机电源关闭命令已提交。\n";
    }
  }
  if (result == 0 && options.zero_cycle) {
    std::cout << "零点循环完成：设零 → +60° → 断电/上电 → 回零；现已停机并提交断电命令。\n";
  }
  return result;
}
