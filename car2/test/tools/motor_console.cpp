#include "bxi/board.h"
#include "motor/bxi/motor.h"
#include "motor/iswv/motor.h"
#include "motor/iswv/bxi_transport.h"

#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unistd.h>

namespace
{
using namespace std::chrono_literals;
volatile std::sig_atomic_t stopped = 0;
void request_stop(int) {stopped = 1;}
void require(bxi::Result<void> result)
{
  if (!result) {throw std::runtime_error(result.error().message);}
}

struct Options
{
  std::string brand;
  unsigned bus = 0;
  unsigned node = 0;
  bxi::Model model = bxi::Model::BXI5014_19;
  bool dry_run = false;
};

unsigned integer(const std::string & value)
{
  std::size_t used = 0;
  if (value.empty() || value.front() == '-') {
    throw std::invalid_argument("invalid unsigned integer");
  }
  const auto result = std::stoul(value, &used, value.compare(0, 2, "0x") == 0 ? 16 : 10);
  if (used != value.size() || result > 0xffff) {throw std::invalid_argument("invalid integer");}
  return static_cast<unsigned>(result);
}

Options parse(int argc, char ** argv)
{
  Options options;
  bool bus = false, node = false;
  for (int i = 1; i < argc; ++i) {
    const std::string key(argv[i]);
    if (key == "--dry-run") {options.dry_run = true; continue;}
    if (++i >= argc) {throw std::invalid_argument("missing option value");}
    const std::string value(argv[i]);
    if (key == "--brand") {options.brand = value;} else if (key == "--bus") {
      options.bus = integer(value); bus = true;
    } else if (key == "--node") {
      options.node = integer(value); node = true;
    } else if (key == "--model") {
      if (value == "5014") {options.model = bxi::Model::BXI5014_19;} else if (value == "5018") {
        options.model = bxi::Model::BXI5018_19;
      } else if (value == "7010") {
        options.model = bxi::Model::BXI7010_19;
      } else if (value == "8515") {options.model = bxi::Model::BXI8515_19;} else {
        throw std::invalid_argument("unknown BXI model");
      }
    } else {throw std::invalid_argument("unknown option: " + key);}
  }
  if ((options.brand != "bxi" && options.brand != "iswv") || !bus || !node || options.bus > 6 ||
    options.node > (options.brand == "bxi" ? 0x7ffU : 127U) ||
    (options.brand == "iswv" && options.node == 0))
  {
    throw std::invalid_argument("require --brand bxi|iswv --bus 0..6 --node CAN-ID/Node-ID");
  }
  return options;
}

void no_extra(std::istringstream & input)
{
  std::string extra;
  if (input >> extra) {throw std::invalid_argument("unexpected extra input");}
}

// Polling keeps SIGINT/SIGTERM cleanup responsive even while waiting for terminal input.
std::optional<std::string> read_line(std::string & pending)
{
  while (!stopped) {
    const auto newline = pending.find('\n');
    if (newline != std::string::npos) {
      const auto line = pending.substr(0, newline);
      pending.erase(0, newline + 1);
      return line;
    }
    pollfd descriptor{STDIN_FILENO, POLLIN, 0};
    const auto ready = poll(&descriptor, 1, 100);
    if (ready < 0) {if (stopped) {break;} throw std::runtime_error("stdin poll failed");}
    if (ready == 0) {continue;}
    char buffer[256];
    const auto count = read(STDIN_FILENO, buffer, sizeof(buffer));
    if (count == 0) {return std::nullopt;}
    if (count < 0) {if (stopped) {break;} throw std::runtime_error("stdin read failed");}
    pending.append(buffer, static_cast<std::size_t>(count));
    if (pending.size() > 1024) {throw std::invalid_argument("input too long");}
  }
  return std::nullopt;
}

struct Feedback
{
  std::mutex mutex;
  std::optional<bxi::Feedback> value;
};
}  // namespace

int main(int argc, char ** argv)
{
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "Usage: motor_console --brand bxi|iswv --bus 0..6 --node ID "
      "[--model 5014|5018|7010|8515] [--dry-run]\n"
      "1 使能；2 禁用；3 控制；4 反馈；0/EOF 退出。\n"
      "BXI: 3 position(rad) velocity(rad/s) kp kd torque(Nm)，单次 MIT 帧。\n"
      "ISWV: 3 output_rpm，使用现有 manual_travel（减速比 10:1），加减速 1 RPM/s。\n"
      "独占共享电源；上电等待 4 秒，显式使能后才能运动；退出禁用并断电。\n"
      "Help and --dry-run never initialize hardware.\n";
    return 0;
  }
  Options options;
  try {
    options = parse(argc, argv);
  } catch (const std::exception & error) {
    std::cerr << "Arguments: " << error.what() << '\n'; return 2;
  }
  if (options.dry_run) {
    std::cout << "dry-run: " << options.brand << " bus=" << options.bus << " node=" << options.node
              << "; no hardware initialized\n";
    return 0;
  }
  std::signal(SIGINT, request_stop);
  std::signal(SIGTERM, request_stop);
  std::signal(SIGHUP, request_stop);
  std::signal(SIGPIPE, SIG_IGN);
  int result = 0;
  std::shared_ptr<chassis::BxiPciTransport> transport;
  std::unique_ptr<iswv::canopen::CanopenMaster> master;
  std::unique_ptr<chassis::IswvDrive> iswv_motor;
  std::unique_ptr<bxi::Motor> bxi_motor;
  auto feedback = std::make_shared<Feedback>();
  bool enabled = false;
  try {
    transport = std::make_shared<chassis::BxiPciTransport>(options.bus);
    require(transport->set_motor_power(true));
    std::cout << "共享电源已开启，等待 4 秒稳定；尚未使能电机。\n" << std::flush;
    const auto ready = std::chrono::steady_clock::now() + 4s;
    while (!stopped && std::chrono::steady_clock::now() < ready) {
      std::this_thread::sleep_for(20ms);
    }
    if (!stopped) {
      if (options.brand == "bxi") {
        bxi_motor = std::make_unique<bxi::Motor>(*transport, options.node);
        transport->set_receive_handler(
          [feedback, id = options.node | 0x10U,
          ranges = bxi::encoding_ranges(options.model)](const bxi::CanFrame & frame) {
            if (frame.id != id) {return;}
            const auto value = bxi::decode_feedback(frame, ranges);
            if (value) {
              std::lock_guard<std::mutex> lock(feedback->mutex);
              feedback->value = value.value();
            }
          });
      } else {
        master = std::make_unique<iswv::canopen::CanopenMaster>(
          std::make_shared<iswv::BxiTransport>(transport));
        iswv_motor = std::make_unique<chassis::IswvDrive>(
          *master,
          iswv::AxisConfiguration::manual_travel(static_cast<std::uint8_t>(options.node)));
      }
    }
    std::cout << "1 使能；2 禁用；3 控制；4 反馈；0/EOF 退出\n"
              << (options.brand == "bxi" ? "控制：3 position(rad) velocity(rad/s) kp kd torque(Nm)\n" :
    "控制：3 output_rpm（10:1 输出轴；加減速 1 RPM/s）\n");
    std::string pending;
    while (!stopped) {
      std::cout << "> " << std::flush;
      const auto line = read_line(pending);
      if (!line) {break;}
      try {
        std::istringstream input(*line);
        std::string command;
        input >> command;
        if (command == "0") {no_extra(input); break;}
        if (command == "1") {
          no_extra(input);
          if (bxi_motor) {require(bxi_motor->enter_motor_mode());} else {
            iswv_motor->enable_rpm(1.0, 1.0);
          }
          enabled = true;
        } else if (command == "2") {
          no_extra(input);
          if (bxi_motor) {require(bxi_motor->exit_motor_mode());} else {iswv_motor->stop();}
          enabled = false;
        } else if (command == "3") {
          if (!enabled) {throw std::invalid_argument("请先输入 1 显式使能");}
          if (bxi_motor) {
            bxi::Command target;
            target.ranges = bxi::encoding_ranges(options.model);
            if (!(input >> target.position >> target.velocity >> target.kp >> target.kd >>
              target.torque))
            {
              throw std::invalid_argument("BXI 控制需要五个数值");
            }
            no_extra(input);
            const auto packed = bxi::pack_command(target);
            if (!packed) {throw std::invalid_argument(packed.error().message);}
            require(bxi_motor->command(target));
          } else {
            double rpm;
            if (!(input >> rpm) || !std::isfinite(rpm)) {
              throw std::invalid_argument("需要有限 RPM 数值");
            }
            no_extra(input);
            iswv_motor->set_velocity_rpm(rpm);
          }
        } else if (command == "4") {
          no_extra(input);
          if (bxi_motor) {
            std::lock_guard<std::mutex> lock(feedback->mutex);
            if (feedback->value) {
              std::cout << "最近反馈 position=" << feedback->value->position
                        << " velocity=" << feedback->value->velocity << '\n';
            } else {std::cout << "尚无反馈（默认反馈 ID = command_id | 0x10）。\n";}
          } else {std::cout << "output_rpm=" << iswv_motor->actual_velocity_rpm() << '\n';}
        } else {throw std::invalid_argument("请输入 0..4");}
      } catch (const std::invalid_argument & error) {
        std::cerr << "输入无效：" << error.what() << '\n';
      }
    }
  } catch (const std::exception & error) {
    std::cerr << "Motor console: " << error.what() << '\n'; result = 1;
  }
  try {
    if (bxi_motor) {require(bxi_motor->exit_motor_mode());}
    if (iswv_motor) {iswv_motor->stop();}
  } catch (const std::exception & error) {
    std::cerr << "Stop failed: " << error.what() << '\n'; result = 1;
  }
  bxi_motor.reset();
  iswv_motor.reset();
  master.reset();
  if (transport) {
    transport->set_receive_handler({});
    const auto off = transport->set_motor_power(false);
    if (!off) {std::cerr << "Power off failed: " << off.error().message << '\n'; result = 1;}
  }
  return result;
}
