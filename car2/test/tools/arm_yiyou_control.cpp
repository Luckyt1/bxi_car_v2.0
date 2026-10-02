#include "bxi/board.h"
#include "motor/yiyou/communication.h"
#include "motor/yiyou/motor.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <poll.h>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace
{
using namespace std::chrono_literals;
volatile std::sig_atomic_t stop_requested = 0;
#ifdef YIYOU_CONTROL_TEST_FAST_WAIT
constexpr auto power_on_wait = 100ms;
constexpr auto save_wait = 100ms;
#else
constexpr auto power_on_wait = 4s;
constexpr auto save_wait = 2s;
#endif

// The vendor library writes diagnostics with printf() and write(1, ...), including
// from its background threads. Reserve the original pipe for complete JSON records
// before initializing the library, and route all ordinary stdout to stderr.
class JsonOutput
{
public:
  JsonOutput()
  {
    std::cout.flush();
    std::fflush(stdout);
    fd_ = dup(STDOUT_FILENO);
    if (fd_ < 0) {throw std::runtime_error("cannot reserve JSON output pipe");}
    if (dup2(STDERR_FILENO, STDOUT_FILENO) < 0) {
      close(fd_);
      throw std::runtime_error("cannot redirect vendor diagnostics to stderr");
    }
    std::setvbuf(stdout, nullptr, _IONBF, 0);
  }

  ~JsonOutput() {close(fd_);}
  JsonOutput(const JsonOutput &) = delete;
  JsonOutput & operator=(const JsonOutput &) = delete;

  void send(std::string record) const
  {
    record += '\n';
    std::size_t offset = 0;
    while (offset < record.size()) {
      const auto count = ::write(fd_, record.data() + offset, record.size() - offset);
      if (count < 0 && errno == EINTR) {continue;}
      if (count <= 0) {throw std::runtime_error("controller output disconnected");}
      offset += static_cast<std::size_t>(count);
    }
  }

private:
  int fd_{-1};
};

void request_stop(int) {stop_requested = 1;}

void check_cancelled()
{
  if (stop_requested) {throw std::runtime_error("control interrupted");}
  pollfd input{STDIN_FILENO, POLLIN, 0};
  if (poll(&input, 1, 0) > 0 && (input.revents & (POLLHUP | POLLERR | POLLNVAL))) {
    throw std::runtime_error("controller input disconnected");
  }
}

void wait_power()
{
  const auto ready = std::chrono::steady_clock::now() + power_on_wait;
  while (std::chrono::steady_clock::now() < ready) {
    check_cancelled();
    std::this_thread::sleep_for(20ms);
  }
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

double real_number(const std::string & text)
{
  std::size_t end = 0;
  const auto value = std::stod(text, &end);
  if (end != text.size() || !std::isfinite(value)) {
    throw std::invalid_argument("expected a finite number: " + text);
  }
  return value;
}

struct Options
{
  std::string interface;
  std::vector<std::uint16_t> slaves;
  unsigned power_bus{3};
  double speed_rpm{1};
  double timeout_s{60};
  bool current_feedback{false};
};

void usage()
{
  std::cout <<
    "Usage: arm_yiyou_control --interface NIC --slaves 1,2,3 [--power-bus 3]\n"
    "                         [--speed-rpm 1] [--timeout 60] [--current-feedback]\n"
    "Persistent backend for yiyou_control.py; use start_yiyou_jog.sh / start_yiyou_home.sh.\n"
    "Commands on stdin: hold; step SLAVE 1|-1; zero SLAVE; home; quit.\n"
    "JSON responses on stdout. --slaves are EtherCAT physical chain positions 1..199.\n"
    "BXI CAN is used only for shared motor power via --power-bus; no motor CAN traffic is sent.\n"
    "Startup reads information without enabling. Home is an explicit command.\n"
    "zero overwrites EEPROM zero and saves all persistent parameters.\n"
    "Help does not open EtherCAT or CAN devices.\n";
}

Options parse(int argc, char ** argv)
{
  Options result;
  std::set<std::string> seen;
  for (int i = 1; i < argc; ++i) {
    const std::string key = argv[i];
    if (!seen.insert(key).second) {throw std::invalid_argument("duplicate option: " + key);}
    if (key == "--current-feedback") {
      result.current_feedback = true;
      continue;
    }
    if (key == "--bus" || key == "--nodes" || key == "--node") {
      throw std::invalid_argument(
              key + " is a retired CAN option; use --interface NIC and --slaves positions");
    }
    if (++i == argc) {throw std::invalid_argument("missing value for " + key);}
    const std::string value = argv[i];
    if (key == "--interface") {
      if (value.empty()) {throw std::invalid_argument("--interface must not be empty");}
      result.interface = value;
    } else if (key == "--slaves") {
      if (value.empty() || value.back() == ',') {throw std::invalid_argument("invalid slave list");}
      std::istringstream list(value);
      std::set<std::uint16_t> ids;
      std::string item;
      while (std::getline(list, item, ',')) {
        const auto id = static_cast<std::uint16_t>(integer(item, 1, 199));
        if (!ids.insert(id).second) {throw std::invalid_argument("duplicate slave position");}
        result.slaves.push_back(id);
      }
    } else if (key == "--power-bus") {
      result.power_bus = static_cast<unsigned>(integer(value, 0, CANFD_DEVICE_NUM - 1));
    } else if (key == "--speed-rpm") {
      result.speed_rpm = real_number(value);
    } else if (key == "--timeout") {
      result.timeout_s = real_number(value);
    } else {
      throw std::invalid_argument("unknown option: " + key);
    }
  }
  if (result.interface.empty() || result.slaves.empty()) {
    throw std::invalid_argument("--interface and --slaves are required");
  }
  if (result.speed_rpm <= 0 || result.speed_rpm >= 60 ||
    result.timeout_s <= 0 || result.timeout_s > 3600)
  {
    throw std::invalid_argument("speed must be in (0, 60) RPM; timeout must be in (0, 3600] seconds");
  }
  chassis::ethercat::validate_selection({result.interface, result.slaves});
  return result;
}

template<typename T>
T read(
  chassis::ethercat::Master & master, std::uint16_t id,
  std::uint16_t index, std::uint8_t sub = 0)
{
  const auto result = master.node(id)->read(
    iswv::canopen::Object<T>{{index, sub},
      "control preflight"});
  if (!result) {throw std::runtime_error(result.error().message);}
  return result.value();
}

struct Axis
{
  std::uint16_t id;
  std::unique_ptr<chassis::YiyouMotor> motor;
  std::array<std::uint32_t, 4> identity{};
  double scale{0};
  std::uint32_t speed{0};
  std::uint32_t acceleration{0};
  std::int32_t step{0};
  std::uint32_t tolerance{0};
  std::int32_t target{0};
  bool held{false};
  std::uint32_t rated_current_ma{0};
  std::chrono::steady_clock::time_point next_current_feedback{};
};

void current_feedback(Axis & axis, const Options & options, const JsonOutput & output)
{
  if (!options.current_feedback || !axis.held ||
    std::chrono::steady_clock::now() < axis.next_current_feedback)
  {
    return;
  }
  const auto current = axis.motor->actual_current_permille();
  const auto position = axis.motor->actual_position();
  std::ostringstream record;
  record << "{\"event\":\"feedback\",\"slave\":" << axis.id
         << ",\"position\":" << position
         << ",\"current_permille\":" << current
         << ",\"rated_current_ma\":" << axis.rated_current_ma << '}';
  output.send(record.str());
  axis.next_current_feedback = std::chrono::steady_clock::now() + 200ms;
}

std::int64_t error_from_target(Axis & axis)
{
  return std::abs(static_cast<std::int64_t>(axis.motor->actual_position()) - axis.target);
}

bool monitor(Axis & axis)
{
  const bool reached = axis.motor->position_reached(axis.tolerance);
  const auto velocity = axis.motor->actual_velocity();
  if (std::abs(static_cast<double>(velocity) * 60.0 / axis.scale) >= 60.0) {
    throw std::runtime_error("slave " + std::to_string(axis.id) + " exceeds 60 RPM");
  }
  return reached && axis.motor->motion_stopped();
}

void monitor_all(std::vector<Axis> & axes)
{
  for (auto & axis : axes) {
    if (axis.held) {(void)monitor(axis);}
  }
}

void wait_reached(
  Axis & moving, std::vector<Axis> & axes, const Options & options, const JsonOutput & output)
{
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(options.timeout_s);
  while (true) {
    check_cancelled();
    bool reached = false;
    for (auto & axis : axes) {
      if (!axis.held) {continue;}
      const bool stationary = monitor(axis);
      current_feedback(axis, options, output);
      if (&axis == &moving) {reached = stationary;}
    }
    if (reached) {return;}
    if (std::chrono::steady_clock::now() >= deadline) {
      throw std::runtime_error(
              "slave " + std::to_string(moving.id) +
              " motion timed out; target=" + std::to_string(moving.target) +
              ", position error=" + std::to_string(error_from_target(moving)));
    }
    std::this_thread::sleep_for(20ms);
  }
}

void enable_hold(Axis & axis)
{
  check_cancelled();
  axis.target = axis.motor->actual_position();
  axis.motor->enable_position(axis.speed, axis.acceleration, axis.acceleration);
  axis.held = true;
  check_cancelled();
  axis.motor->move_to_position(axis.target);
}

void done(const JsonOutput & output, const std::string & command, Axis * axis = nullptr)
{
  const auto position = axis == nullptr ? 0 : axis->motor->actual_position();
  std::ostringstream record;
  record << "{\"event\":\"done\",\"command\":\"" << command << '"';
  if (axis != nullptr) {
    record << ",\"slave\":" << static_cast<unsigned>(axis->id)
           << ",\"position\":" << position;
  }
  record << '}';
  output.send(record.str());
}

// No dependency is needed for the small outbound JSON protocol.
std::string json_string(const std::string & text)
{
  std::ostringstream out;
  out << '"';
  for (const unsigned char c : text) {
    if (c == '"' || c == '\\') {out << '\\' << c;} else if (c < 0x20) {
      out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<unsigned>(c);
    } else {out << c;}
  }
  out << '"';
  return out.str();
}

void ready(const JsonOutput & output, const std::vector<Axis> & axes)
{
  std::ostringstream record;
  record << "{\"event\":\"ready\",\"axes\":[";
  bool first = true;
  for (const auto & axis : axes) {
    if (!first) {record << ',';}
    first = false;
    record << "{\"slave\":" << static_cast<unsigned>(axis.id) << ",\"identity\":[";
    for (std::size_t i = 0; i < axis.identity.size(); ++i) {
      if (i != 0) {record << ',';}
      record << axis.identity[i];
    }
    record << "],\"scale\":" << std::setprecision(17) << axis.scale
           << ",\"position\":" << axis.target << '}';
  }
  record << "]}";
  output.send(record.str());
}

bool command(
  const std::string & line, std::vector<Axis> & axes, const Options & options,
  bool & save_pending, const JsonOutput & output)
{
  std::istringstream input(line);
  std::vector<std::string> words;
  std::string word;
  while (input >> word) {words.push_back(word);}
  if (words.empty()) {throw std::invalid_argument("empty command");}
  const auto & action = words.front();
  if (action == "quit" && words.size() == 1) {return false;}
  if ((action == "hold" || action == "home") && words.size() == 1) {
    if (std::any_of(axes.begin(), axes.end(), [](const Axis & a) {return a.held;})) {
      throw std::logic_error("hold/home is only allowed once at startup");
    }
    for (auto & axis : axes) {
      enable_hold(axis);
    }
    for (auto & axis : axes) {
      if (action == "home") {
        check_cancelled();
        axis.target = 0;
        axis.motor->move_to_position(0);
      }
      wait_reached(axis, axes, options, output);
    }
    done(output, action);
    return true;
  }
  if ((action != "step" || words.size() != 3) &&
    (action != "zero" || words.size() != 2))
  {
    throw std::invalid_argument("invalid control command");
  }
  const auto id = integer(words[1], 1, 199);
  auto found = std::find_if(
    axes.begin(), axes.end(),
    [id](const Axis & axis) {return axis.id == id;});
  if (found == axes.end()) {throw std::invalid_argument("slave is not in --slaves");}
  auto & axis = *found;
  if (!axis.held) {throw std::logic_error("hold must complete before step or zero");}
  check_cancelled();
  if (action == "step") {
    const auto sign = integer(words[2], -1, 1);
    if (sign == 0) {throw std::invalid_argument("step sign must be 1 or -1");}
    const auto target = static_cast<std::int64_t>(axis.target) + sign * axis.step;
    if (target < std::numeric_limits<std::int32_t>::min() ||
      target > std::numeric_limits<std::int32_t>::max())
    {
      throw std::out_of_range("step exceeds int32 position range");
    }
    axis.target = static_cast<std::int32_t>(target);
    axis.motor->move_to_position(axis.target);
    wait_reached(axis, axes, options, output);
  } else {
    wait_reached(axis, axes, options, output);
    axis.motor->stop();
    axis.held = false;
    if (error_from_target(axis) > axis.tolerance) {
      throw std::runtime_error("position drifted after disable; zero was not changed");
    }
    check_cancelled();
    save_pending = true;  // A failed SDO may still have reached the EEPROM.
    axis.motor->save_zero_position(axis.tolerance);
    std::this_thread::sleep_for(save_wait);  // Do not cut power midway through saving.
    save_pending = false;
    check_cancelled();
    axis.target = 0;
    if (error_from_target(axis) > axis.tolerance) {
      throw std::runtime_error("position drifted while saving zero");
    }
    enable_hold(axis);
    axis.target = 0;
    axis.motor->move_to_position(0);
    wait_reached(axis, axes, options, output);
  }
  done(output, action, &axis);
  return true;
}

void serve(
  std::vector<Axis> & axes, const Options & options, bool & save_pending, const JsonOutput & output)
{
  std::string pending;
  while (!stop_requested) {
    monitor_all(axes);
    for (auto & axis : axes) {
      current_feedback(axis, options, output);
    }
    pollfd input{STDIN_FILENO, POLLIN, 0};
    const auto count = poll(&input, 1, 20);
    if (count < 0) {
      if (errno == EINTR) {continue;}
      throw std::runtime_error("poll stdin failed");
    }
    if (count == 0) {continue;}
    if (input.revents & (POLLERR | POLLNVAL)) {throw std::runtime_error("stdin failed");}
    char buffer[256];
    const auto received = ::read(STDIN_FILENO, buffer, sizeof(buffer));
    if (received == 0) {return;}
    if (received < 0) {
      if (errno == EINTR) {continue;}
      throw std::runtime_error("read stdin failed");
    }
    pending.append(buffer, static_cast<std::size_t>(received));
    if (pending.size() > 1024) {throw std::invalid_argument("control input too long");}
    auto end = pending.find('\n');
    while (end != std::string::npos) {
      const auto line = pending.substr(0, end);
      pending.erase(0, end + 1);
      if (!command(line, axes, options, save_pending, output)) {return;}
      end = pending.find('\n');
    }
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
    return 2;
  }
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  std::signal(SIGHUP, request_stop);
  std::signal(SIGPIPE, SIG_IGN);
  int result = 0;
  bool save_pending = false;
  std::unique_ptr<JsonOutput> output;
  std::shared_ptr<chassis::BxiPciTransport> transport;
  std::unique_ptr<chassis::ethercat::Master> master;
  std::vector<Axis> axes;
  try {
    output = std::make_unique<JsonOutput>();
    check_cancelled();
    transport = std::make_shared<chassis::BxiPciTransport>(options.power_bus);
    const auto powered = transport->set_motor_power(true);
    if (!powered) {throw std::runtime_error(powered.error().message);}
    wait_power();
    master = std::make_unique<chassis::ethercat::Master>(
      chassis::ethercat::Options{options.interface, options.slaves});
    for (const auto id : options.slaves) {
      check_cancelled();
      Axis axis{};
      axis.id = id;
      axis.motor = std::make_unique<chassis::YiyouMotor>(*master, id);
      axis.scale = axis.motor->configure_rpm_units();
      axis.speed = static_cast<std::uint32_t>(axis.motor->velocity_from_rpm(options.speed_rpm));
      axis.acceleration = static_cast<std::uint32_t>(axis.motor->velocity_from_rpm(10));
      axis.step = axis.motor->position_from_degrees(100);
      axis.tolerance = std::min(100U, static_cast<std::uint32_t>(axis.step / 4));
      axis.target = axis.motor->actual_position();
      for (std::uint8_t sub = 1; sub <= axis.identity.size(); ++sub) {
        axis.identity[sub - 1] = read<std::uint32_t>(*master, id, 0x1018, sub);
      }
      const auto state = iswv::cia402::decode_state(read<std::uint16_t>(*master, id, 0x6041));
      if ((state != iswv::cia402::DriveState::switch_on_disabled &&
        state != iswv::cia402::DriveState::ready_to_switch_on &&
        state != iswv::cia402::DriveState::switched_on) ||
        !axis.motor->motion_stopped())
      {
        throw std::runtime_error(
                "slave " + std::to_string(id) +
                " must be disabled and stationary");
      }
      if (options.current_feedback) {
        // Some firmware omits rated current. Keep the raw permille reading usable.
        const auto rated = master->node(id)->read(
          iswv::canopen::Object<std::uint32_t>{{0x6075, 0}, "rated current (mA)"});
        if (rated) {axis.rated_current_ma = rated.value();}
      }
      axes.push_back(std::move(axis));
    }
    ready(*output, axes);
    serve(axes, options, save_pending, *output);
  } catch (const std::exception & error) {
    result = 1;
    try {
      if (output) {
        output->send("{\"event\":\"error\",\"message\":" + json_string(error.what()) + "}");
      } else {
        std::cerr << error.what() << '\n';
      }
    } catch (const std::exception &) {
      // A closed frontend pipe must not bypass motor stop or the EEPROM save wait.
      std::cerr << error.what() << '\n';
    }
  }
  if (save_pending) {std::this_thread::sleep_for(save_wait);}
  for (auto & axis : axes) {
    try {
      axis.motor->stop();
    } catch (const std::exception & error) {
      std::cerr << "Stop slave " << static_cast<unsigned>(axis.id) << ": " << error.what() << '\n';
      result = 1;
    }
  }
  axes.clear();
  master.reset();
  if (transport) {
    const auto off = transport->set_motor_power(false);
    if (!off) {std::cerr << off.error().message << '\n'; result = 1;}
  }
  return result;
}
