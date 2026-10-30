// Exercise the real command handlers with a simulated CoE object dictionary.
#define YIYOU_CONTROL_TEST_FAST_WAIT
#define main yiyou_control_program_main
#include "../../tools/arm_yiyou_control.cpp"
#undef main

#include <filesystem>
#include <functional>
#include <map>
#include <yaml-cpp/yaml.h>

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

template<typename Exception = std::exception, typename Function>
void rejects(Function function)
{
  try {
    function();
  } catch (const Exception &) {
    return;
  }
  throw std::runtime_error("expected rejection");
}

// The command handlers expect a connected frontend, independently of whether
// ctest itself was launched from a terminal, /dev/null, or an exhausted SSH pipe.
class LiveFrontendInput
{
public:
  LiveFrontendInput()
  {
    saved_stdin_ = ::dup(STDIN_FILENO);
    if (saved_stdin_ < 0) {throw std::runtime_error("cannot save test stdin");}
    int connection[2];
    if (::pipe(connection) != 0) {
      ::close(saved_stdin_);
      throw std::runtime_error("cannot create test frontend pipe");
    }
    if (::dup2(connection[0], STDIN_FILENO) < 0) {
      ::close(connection[0]);
      ::close(connection[1]);
      ::close(saved_stdin_);
      throw std::runtime_error("cannot connect test frontend stdin");
    }
    ::close(connection[0]);
    writer_ = connection[1];
  }

  ~LiveFrontendInput()
  {
    (void)::dup2(saved_stdin_, STDIN_FILENO);
    ::close(saved_stdin_);
    ::close(writer_);
  }

  LiveFrontendInput(const LiveFrontendInput &) = delete;
  LiveFrontendInput & operator=(const LiveFrontendInput &) = delete;

private:
  int saved_stdin_{-1};
  int writer_{-1};
};

class FakeNode : public chassis::ethercat::Node
{
public:
  FakeNode()
  {
    put<std::uint16_t>(0x6041, 0x40);
    put<std::uint8_t>(0x2100, 1);
    put<std::int8_t>(0x6060, 0);
    put<std::int8_t>(0x6061, 0);
    put<std::int8_t>(0x2707, 1);
    put<std::int32_t>(0x6064, 12000);
    put<std::int32_t>(0x606C, 0);
  }

  std::uint16_t id() const noexcept override {return 4;}

  template<typename T>
  void put(std::uint16_t index, T value)
  {
    const auto bytes = iswv::canopen::encode_little_endian(value);
    values[{index, 0}] = {bytes.begin(), bytes.end()};
  }

  iswv::Result<iswv::canopen::SdoResponse> upload_blocking(
    iswv::canopen::ObjectAddress object, iswv::canopen::SdoOptions) override
  {
    const auto found = values.find(object);
    if (found == values.end()) {
      return iswv::Result<iswv::canopen::SdoResponse>::failure(
        iswv::make_sdo_abort(0x06020000, "unknown test object"));
    }
    iswv::canopen::SdoResponse response;
    response.object = object;
    response.size = static_cast<std::uint8_t>(found->second.size());
    std::copy(found->second.begin(), found->second.end(), response.data.begin());
    return iswv::Result<iswv::canopen::SdoResponse>::success(response);
  }

  iswv::Result<void> download_blocking(
    iswv::canopen::ObjectAddress object, const std::uint8_t * data, std::size_t size,
    iswv::canopen::SdoOptions) override
  {
    std::uint32_t value = 0;
    for (std::size_t i = 0; i < size; ++i) {value |= std::uint32_t{data[i]} << (8U * i);}
    if (before_write) {before_write(object.index, value);}
    values[object] = {data, data + size};
    if (object.index == 0x6060) {put<std::int8_t>(0x6061, static_cast<std::int8_t>(value));}
    if (object.index == 0x607A) {target = static_cast<std::int32_t>(value);}
    if (object.index == 0x6040) {
      std::uint16_t status = 0x40;
      switch (value & 0x0f) {
        case 6: status = 0x21; break;
        case 7: status = 0x23; break;
        case 15: status = (value & 0x10) ? 0x1427 : 0x0427; break;
        default: break;
      }
      if (value == 0x3f) {
        ++triggers;
        put<std::int32_t>(0x6064, target);
      }
      if (value == 0x10) {put<std::int32_t>(0x6064, 0);}
      put<std::uint16_t>(0x6041, status);
    }
    return iswv::Result<void>::success();
  }

  std::function<void(std::uint16_t, std::uint32_t)> before_write;
  std::int32_t target{};
  unsigned triggers{};
  std::map<iswv::canopen::ObjectAddress, std::vector<std::uint8_t>> values;
};

constexpr std::array<std::uint32_t, 4> identity{4247, 65537, 64, 65553};
constexpr double scale = 36000;

struct Fixture
{
  Fixture(const std::filesystem::path & path, bool seeded)
  : node(std::make_shared<FakeNode>())
  {
    options.position_memory = path.string();
    options.timeout_s = 0.2;
    if (seeded) {
      chassis::PositionMemory seed(path.string(), "eth0", 4, identity, scale);
      seed.set_reference(120000, 120000);
    }
    Axis axis{};
    axis.id = 4;
    axis.identity = identity;
    axis.scale = scale;
    axis.speed = 1200;
    axis.acceleration = 2400;
    axis.step = 10000;
    axis.tolerance = 2;
    axis.target = 12000;
    axis.motor = std::make_unique<chassis::YiyouMotor>(node);
    axis.position_memory = std::make_unique<chassis::PositionMemory>(
      path.string(), "eth0", 4, identity, scale);
    try {
      (void)axis.position_memory->restore(axis.target);
    } catch (const std::exception & error) {
      axis.position_valid = false;
      axis.position_error = error.what();
    }
    axes.push_back(std::move(axis));
  }

  void run(const std::string & text, const JsonOutput & output)
  {
    require(command(text, axes, options, save_pending, output), "unexpected quit");
  }

  std::shared_ptr<FakeNode> node;
  Options options;
  bool save_pending{false};
  std::vector<Axis> axes;
};

void restored_motion(const std::filesystem::path & path, const JsonOutput & output)
{
  Fixture fixture(path, true);
  auto & axis = fixture.axes.front();
  require(axis.position_valid, "1200 degree record restores from raw 120 degrees");
  std::ostringstream json;
  position_fields(json, axis, 12000);
  require(json.str().find("\"position\":120000") != std::string::npos,
    "debug protocol exposes the logical position");
  require(json.str().find("\"raw_position\":12000") != std::string::npos,
    "debug protocol retains raw feedback separately");
  fixture.node->before_write = [&](std::uint16_t index, std::uint32_t value) {
      if (index == 0x6040 && value == 0x3f) {
        const auto record = YAML::LoadFile(path.string());
        const auto logical_target = fixture.node->target + 108000;
        require(record["valid"].as<bool>(), "trigger requires a valid durable record");
        require(record["lower"].as<std::int64_t>() <= logical_target &&
          record["upper"].as<std::int64_t>() >= logical_target,
          "new target must be durable before the trigger");
      }
    };
  fixture.run("hold", output);
  require(fixture.node->target == 12000, "startup holds current raw position");
  const auto triggers = fixture.node->triggers;
  rejects<CommandRefused>([&]() {fixture.run("step 4 1", output);});
  require(fixture.node->triggers == triggers, "upper limit does not trigger motion");
  fixture.run("step 4 -1", output);
  require(fixture.node->target == 2000, "logical 1100 degrees converts to raw 20 degrees");
  require(axis.position_memory->logical_position(2000) == 110000, "logical feedback follows inward motion");
  fixture.run("step 4 1", output);
  require(fixture.node->target == 12000, "inward and outward steps retain the fixed offset");
  fixture.node->put<std::int32_t>(0x606C, 120);  // 0.2 RPM despite a set stop flag.
  const auto before_moving = fixture.node->triggers;
  rejects([&]() {fixture.run("step 4 -1", output);});
  require(fixture.node->triggers == before_moving, "nonzero velocity blocks a replacement target");
  fixture.node->put<std::int32_t>(0x606C, 0);
  std::filesystem::remove(path);
  std::filesystem::create_directory(path);
  rejects([&]() {fixture.run("step 4 -1", output);});
  require(fixture.node->triggers == before_moving, "failed durable write blocks motion");
}

void untrusted_zero(const std::filesystem::path & path, const JsonOutput & output)
{
  Fixture fixture(path, false);
  auto & axis = fixture.axes.front();
  require(!axis.position_valid, "missing state does not silently adopt raw position");
  rejects<CommandRefused>([&]() {fixture.run("home", output);});
  require(fixture.node->triggers == 0, "home refusal precedes enabling");
  fixture.run("hold", output);
  rejects<CommandRefused>([&]() {fixture.run("step 4 -1", output);});
  fixture.node->before_write = [&](std::uint16_t index, std::uint32_t value) {
      if ((index == 0x6040 && value == 0x10) || index == 0x2130) {
        require(!YAML::LoadFile(path.string())["valid"].as<bool>(),
          "zero must invalidate the record before changing hardware");
      }
    };
  fixture.run("zero 4", output);
  require(axis.position_valid && axis.position_memory->logical_position(0) == 0,
    "explicit zero repairs missing state");
  rejects<CommandRefused>([&]() {fixture.run("step 4 -1", output);});
  fixture.run("step 4 1", output);
  require(fixture.node->target == 10000, "valid zero permits bounded positive steps");
  fixture.node->before_write = [&](std::uint16_t index, std::uint32_t) {
      if (index == 0x2130) {throw std::runtime_error("injected EEPROM save failure");}
    };
  rejects([&]() {fixture.run("zero 4", output);});
  require(!YAML::LoadFile(path.string())["valid"].as<bool>(), "failed zero leaves durable record invalid");
}
}  // namespace

int main()
{
  char pattern[] = "/tmp/yiyou-control-memory-XXXXXX";
  const auto directory = ::mkdtemp(pattern);
  if (!directory) {return 1;}
  const std::filesystem::path root(directory);
  try {
    LiveFrontendInput input;
    JsonOutput output;
    restored_motion(root / "restored.yaml", output);
    untrusted_zero(root / "untrusted.yaml", output);
    std::filesystem::remove_all(root);
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    std::filesystem::remove_all(root);
    return 1;
  }
}
