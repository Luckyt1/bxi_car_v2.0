#include "motor/yiyou/position_memory.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <future>
#include <limits>
#include <set>
#include <stdexcept>
#include <system_error>
#include <utility>
#include <vector>

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

namespace chassis
{
namespace
{
using Count = std::int64_t;
using Clock = std::chrono::steady_clock;

[[noreturn]] void fail(const std::string & message)
{
  throw std::runtime_error("fourth-axis position memory: " + message);
}

[[noreturn]] void io_fail(const std::string & operation)
{
  throw std::system_error(errno, std::generic_category(), "position memory " + operation);
}

Count add(Count left, Count right)
{
  if ((right > 0 && left > std::numeric_limits<Count>::max() - right) ||
    (right < 0 && left < std::numeric_limits<Count>::min() - right))
  {
    fail("position overflow");
  }
  return left + right;
}

Count subtract(Count left, Count right)
{
  if ((right < 0 && left > std::numeric_limits<Count>::max() + right) ||
    (right > 0 && left < std::numeric_limits<Count>::min() + right))
  {
    fail("position overflow");
  }
  return left - right;
}

Count modulo(Count value, Count period)
{
  const auto remainder = value % period;
  return remainder < 0 ? remainder + period : remainder;
}

struct File
{
  explicit File(int value) : fd(value) {}
  ~File() {if (fd >= 0) {::close(fd);}}
  File(const File &) = delete;
  File & operator=(const File &) = delete;
  int fd;
};

struct Record
{
  bool valid{false};
  std::string interface;
  std::uint16_t slave{};
  std::array<std::uint32_t, 4> identity{};
  Count period{};
  Count lower{};
  Count upper{};
};

void check_window(const Record & record)
{
  if (record.lower > record.upper || subtract(record.upper, record.lower) >= record.period) {
    fail("saved interval is not narrower than one revolution");
  }
}

Count integer(const YAML::Node & node)
{
  if (!node.IsScalar()) {fail("expected an integer field");}
  const auto text = node.Scalar();
  Count result{};
  const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result);
  if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size()) {
    fail("invalid integer field");
  }
  return result;
}

Record read_record(const std::string & path)
{
  File file(::open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW));
  if (file.fd < 0) {io_fail("open state (a trusted reference is required)");}
  std::string content;
  char buffer[4096];
  while (true) {
    const auto size = ::read(file.fd, buffer, sizeof(buffer));
    if (size < 0) {
      if (errno == EINTR) {continue;}
      io_fail("read state");
    }
    if (size == 0) {break;}
    content.append(buffer, static_cast<std::size_t>(size));
    if (content.size() > 16384) {fail("state file is too large");}
  }
  const auto documents = YAML::LoadAll(content);
  if (documents.size() != 1) {fail("state must contain exactly one document");}
  const auto root = documents.front();
  if (!root.IsMap()) {fail("state must be a map");}
  const std::set<std::string> expected{
    "version", "valid", "interface", "slave", "identity", "counts_per_revolution", "lower", "upper"};
  std::set<std::string> fields;
  for (const auto & field : root) {
    if (!field.first.IsScalar() || !fields.insert(field.first.Scalar()).second) {
      fail("duplicate or invalid state field");
    }
  }
  if (fields != expected || integer(root["version"]) != 1) {fail("unsupported state schema");}
  Record record;
  if (!root["valid"].IsScalar() ||
    (root["valid"].Scalar() != "true" && root["valid"].Scalar() != "false"))
  {
    fail("invalid validity marker");
  }
  record.valid = root["valid"].Scalar() == "true";
  if (!root["interface"].IsScalar()) {fail("invalid interface binding");}
  record.interface = root["interface"].Scalar();
  const auto slave = integer(root["slave"]);
  if (slave <= 0 || slave > std::numeric_limits<std::uint16_t>::max()) {fail("invalid slave");}
  record.slave = static_cast<std::uint16_t>(slave);
  if (!root["identity"].IsSequence() || root["identity"].size() != record.identity.size()) {
    fail("invalid device identity");
  }
  for (std::size_t i = 0; i < record.identity.size(); ++i) {
    const auto value = integer(root["identity"][i]);
    if (value < 0 || value > std::numeric_limits<std::uint32_t>::max()) {
      fail("invalid device identity number");
    }
    record.identity[i] = static_cast<std::uint32_t>(value);
  }
  record.period = integer(root["counts_per_revolution"]);
  record.lower = integer(root["lower"]);
  record.upper = integer(root["upper"]);
  if (record.period <= 0) {fail("invalid encoder scale");}
  check_window(record);
  return record;
}

void write_record(const std::string & path, const Record & record)
{
  YAML::Emitter emitter;
  emitter << YAML::BeginMap << YAML::Key << "version" << YAML::Value << 1
          << YAML::Key << "valid" << YAML::Value << record.valid
          << YAML::Key << "interface" << YAML::Value << record.interface
          << YAML::Key << "slave" << YAML::Value << record.slave
          << YAML::Key << "identity" << YAML::Value << YAML::Flow << YAML::BeginSeq;
  for (const auto value : record.identity) {emitter << value;}
  emitter << YAML::EndSeq << YAML::Key << "counts_per_revolution" << YAML::Value << record.period
          << YAML::Key << "lower" << YAML::Value << record.lower
          << YAML::Key << "upper" << YAML::Value << record.upper << YAML::EndMap;
  if (!emitter.good()) {fail("cannot encode state");}
  const std::string content = std::string(emitter.c_str()) + "\n";
  std::string pattern = path + ".tmp.XXXXXX";
  std::vector<char> temporary(pattern.begin(), pattern.end());
  temporary.push_back('\0');
  File file(::mkstemp(temporary.data()));
  if (file.fd < 0) {io_fail("create temporary state");}
  bool renamed = false;
  try {
    std::size_t offset = 0;
    while (offset < content.size()) {
      const auto written = ::write(file.fd, content.data() + offset, content.size() - offset);
      if (written < 0) {
        if (errno == EINTR) {continue;}
        io_fail("write temporary state");
      }
      if (written == 0) {fail("empty state write");}
      offset += static_cast<std::size_t>(written);
    }
    if (::fsync(file.fd) != 0) {io_fail("sync state");}
    if (::rename(temporary.data(), path.c_str()) != 0) {io_fail("replace state");}
    renamed = true;
    const auto parent = std::filesystem::path(path).parent_path();
    File directory(::open(parent.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC));
    if (directory.fd < 0) {io_fail("open state directory");}
    if (::fsync(directory.fd) != 0) {io_fail("sync state directory");}
  } catch (...) {
    if (!renamed) {::unlink(temporary.data());}
    throw;
  }
}
}  // namespace

struct PositionMemory::Impl
{
  Impl(std::string state_path, std::string interface, std::uint16_t slave,
    std::array<std::uint32_t, 4> identity, double scale)
  : lock(-1)
  {
    if (state_path.empty() || interface.empty() || slave == 0 || !std::isfinite(scale) ||
      scale <= 0 || std::floor(scale) != scale ||
      static_cast<long double>(scale) >= static_cast<long double>(std::numeric_limits<Count>::max()))
    {
      fail("invalid path, binding, or integral encoder scale");
    }
    binding.interface = std::move(interface);
    binding.slave = slave;
    binding.identity = identity;
    binding.period = static_cast<Count>(scale);
    tolerance = binding.period / 180 + (binding.period % 180 != 0 ? 1 : 0);
    if (tolerance >= binding.period - tolerance) {fail("encoder scale is too small");}
    path = std::filesystem::absolute(state_path).lexically_normal().string();
    std::filesystem::create_directories(std::filesystem::path(path).parent_path());
    lock.fd = ::open((path + ".lock").c_str(), O_RDWR | O_CREAT | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (lock.fd < 0) {io_fail("open lock");}
    if (::flock(lock.fd, LOCK_EX | LOCK_NB) != 0) {io_fail("lock (another controller owns state)");}
  }

  void no_job() const
  {
    if (future.valid()) {fail("a state write is still outstanding");}
    if (write_error) {std::rethrow_exception(write_error);}
  }

  void require_ready() const
  {
    if (!ready) {fail("no trusted reference; restore or explicitly set a reference first");}
    if (write_error) {std::rethrow_exception(write_error);}
  }

  Record interval(Count lower, Count upper) const
  {
    auto result = binding;
    result.valid = true;
    result.lower = subtract(lower, tolerance);
    result.upper = add(upper, tolerance);
    check_window(result);
    return result;
  }

  std::string path;
  File lock;  // Declared before future: retain the lock until its writer has joined.
  Record binding;
  Record committed;
  Count tolerance{};
  Count offset{};
  Count last_triggered{};
  bool ready{false};
  std::exception_ptr write_error;
  Clock::time_point write_started;
  std::future<Record> future;
};

PositionMemory::PositionMemory(
  std::string path, std::string interface, std::uint16_t slave,
  std::array<std::uint32_t, 4> identity, double scale)
: impl_(std::make_unique<Impl>(std::move(path), std::move(interface), slave, identity, scale)) {}

PositionMemory::~PositionMemory() = default;

std::string PositionMemory::default_path()
{
#ifdef CHASSIS_DEFAULT_POSITION_MEMORY_FILE
  return CHASSIS_DEFAULT_POSITION_MEMORY_FILE;
#else
  return "state/yiyou_motor4_position.yaml";
#endif
}

std::int64_t PositionMemory::restore(std::int32_t raw)
{
  auto & state = *impl_;
  state.no_job();
  state.ready = false;
  const auto record = read_record(state.path);
  if (!record.valid) {fail("reference was invalidated; explicit trusted reference required");}
  if (record.interface != state.binding.interface || record.slave != state.binding.slave ||
    record.identity != state.binding.identity || record.period != state.binding.period)
  {
    fail("saved device identity or encoder scale does not match");
  }
  const auto lower_remainder = modulo(record.lower, record.period);
  const auto raw_remainder = modulo(raw, record.period);
  const auto distance = raw_remainder >= lower_remainder ? raw_remainder - lower_remainder :
    record.period - (lower_remainder - raw_remainder);
  const auto logical = add(record.lower, distance);
  if (logical > record.upper) {fail("feedback has no unique position inside saved interval");}
  state.offset = subtract(logical, raw);
  state.committed = record;
  state.last_triggered = logical;
  state.ready = true;
  return logical;
}

std::int64_t PositionMemory::logical_position(std::int32_t raw) const
{
  const auto & state = *impl_;
  state.require_ready();
  const auto logical = add(raw, state.offset);
  if (logical < state.committed.lower || logical > state.committed.upper) {
    fail("live feedback left the committed motion interval; reference is no longer trusted");
  }
  return logical;
}

std::int32_t PositionMemory::raw_target(std::int64_t logical) const
{
  const auto & state = *impl_;
  state.require_ready();
  if (logical < state.committed.lower || logical > state.committed.upper) {
    fail("target is outside the committed motion interval");
  }
  const auto raw = subtract(logical, state.offset);
  if (raw < std::numeric_limits<std::int32_t>::min() || raw > std::numeric_limits<std::int32_t>::max()) {
    fail("target exceeds the drive position range");
  }
  return static_cast<std::int32_t>(raw);
}

void PositionMemory::queue_target(std::int32_t raw_actual, std::int64_t logical_target)
{
  auto & state = *impl_;
  state.no_job();
  const auto actual = logical_position(raw_actual);
  const auto raw = subtract(logical_target, state.offset);
  if (raw < std::numeric_limits<std::int32_t>::min() || raw > std::numeric_limits<std::int32_t>::max()) {
    fail("target exceeds the drive position range");
  }
  const auto record = state.interval(
    std::min({actual, state.last_triggered, logical_target}),
    std::max({actual, state.last_triggered, logical_target}));
  state.write_started = Clock::now();
  state.future = std::async(std::launch::async, [path = state.path, record]() {
      write_record(path, record);
      return record;
    });
}

bool PositionMemory::poll()
{
  auto & state = *impl_;
  if (state.write_error) {std::rethrow_exception(state.write_error);}
  if (!state.future.valid()) {return true;}
  if (state.future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    try {
      state.committed = state.future.get();
      return true;
    } catch (...) {
      state.write_error = std::current_exception();
      throw;
    }
  }
  if (Clock::now() - state.write_started > std::chrono::seconds(1)) {
    state.write_error = std::make_exception_ptr(
      std::runtime_error("fourth-axis position memory: durable write exceeded one second"));
    std::rethrow_exception(state.write_error);
  }
  return false;
}

bool PositionMemory::pending() const {return impl_->future.valid();}

void PositionMemory::mark_target(std::int64_t logical)
{
  impl_->no_job();
  (void)raw_target(logical);
  impl_->last_triggered = logical;
}

void PositionMemory::invalidate()
{
  auto & state = *impl_;
  state.no_job();
  state.ready = false;
  write_record(state.path, state.binding);
}

void PositionMemory::set_reference(std::int32_t raw, std::int64_t logical)
{
  auto & state = *impl_;
  state.no_job();
  state.ready = false;
  const auto offset = subtract(logical, raw);
  if (offset % state.binding.period != 0) {fail("reference offset must be a whole number of turns");}
  const auto record = state.interval(logical, logical);
  write_record(state.path, record);
  state.offset = offset;
  state.committed = record;
  state.last_triggered = logical;
  state.ready = true;
}

}  // namespace chassis
