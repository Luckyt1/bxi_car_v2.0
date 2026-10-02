#include "motor/yiyou/communication.h"
#include "iswv/cia402/drive.hpp"

#include <ethercat.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <condition_variable>
#include <cstring>
#include <ifaddrs.h>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <net/if.h>
#include <netinet/in.h>
#include <set>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace chassis::ethercat
{
namespace
{
using namespace std::chrono_literals;
using iswv::canopen::ObjectAddress;
using iswv::canopen::SdoOptions;
using iswv::canopen::SdoResponse;
constexpr auto cycle_period = 2ms;
constexpr auto feedback_deadline = 20ms;
constexpr std::uint8_t process_group = 1;  // SOEM group 0 means ALL slaves.
constexpr std::size_t profile_output_bytes = 11;
constexpr std::size_t profile_input_bytes = 11;
constexpr std::size_t cst_output_bytes = 5;
constexpr std::size_t cst_input_bytes = 13;
constexpr std::size_t max_process_bytes = cst_input_bytes;
constexpr std::uint32_t yiyou_vendor_id = 0x00001097;  // ethercat.org registered vendor ID.
std::mutex master_mutex;  // SOEM 1.4 legacy context: one master per process.

std::string address_text(std::uint16_t slave, ObjectAddress object)
{
  std::ostringstream out;
  out << "EtherCAT slave " << slave << " CoE 0x" << std::hex << object.index
      << ':' << static_cast<unsigned>(object.subindex);
  return out.str();
}

template<typename T>
std::string hex_text(T value, int width = 0)
{
  std::ostringstream out;
  out << "0x" << std::hex << std::nouppercase << std::setfill('0');
  if (width > 0) {out << std::setw(width);}
  out << static_cast<std::uint64_t>(value);
  return out.str();
}

std::string printable_text(const char * text, std::size_t limit)
{
  std::string result;
  for (std::size_t i = 0; i < limit && text[i] != '\0'; ++i) {
    const unsigned char c = static_cast<unsigned char>(text[i]);
    result.push_back(std::isprint(c) ? static_cast<char>(c) : '?');
  }
  return result.empty() ? "<empty>" : result;
}

void validate_options(const Options & options)
{
  validate_selection(options);
  if (if_nametoindex(options.interface.c_str()) == 0) {
    throw std::invalid_argument("EtherCAT interface does not exist: " + options.interface);
  }
  ifaddrs * interfaces = nullptr;
  if (getifaddrs(&interfaces) != 0) {throw std::runtime_error("cannot inspect network interfaces");}
  bool has_ip = false;
  bool up = false;
  for (auto * item = interfaces; item; item = item->ifa_next) {
    if (options.interface != item->ifa_name) {continue;}
    up = (item->ifa_flags & IFF_UP) != 0;
    if (!item->ifa_addr) {continue;}
    if (item->ifa_addr->sa_family == AF_INET) {has_ip = true;}
    if (item->ifa_addr->sa_family == AF_INET6) {
      const auto * v6 = reinterpret_cast<const sockaddr_in6 *>(item->ifa_addr);
      if (!IN6_IS_ADDR_LINKLOCAL(&v6->sin6_addr)) {has_ip = true;}
    }
  }
  freeifaddrs(interfaces);
  if (!up || has_ip) {
    throw std::invalid_argument(
            "EtherCAT requires an UP dedicated interface without IPv4/global IPv6 addresses: " +
            options.interface + "; do not use the SSH/network interface");
  }
}

enum class Layout {profile, cst};

struct Field {std::size_t offset; std::size_t bytes;};
std::size_t output_bytes(Layout layout)
{
  return layout == Layout::cst ? cst_output_bytes : profile_output_bytes;
}
std::size_t input_bytes(Layout layout)
{
  return layout == Layout::cst ? cst_input_bytes : profile_input_bytes;
}
std::optional<Field> output_field(Layout layout, ObjectAddress object)
{
  if (object.subindex != 0) {return {};}
  if (layout == Layout::cst) {
    switch (object.index) {
      case 0x6040: return Field{0, 2};
      case 0x6071: return Field{2, 2};
      case 0x6060: return Field{4, 1};
      default: return {};
    }
  } else {
    switch (object.index) {
      case 0x6040: return Field{0, 2};
      case 0x607A: return Field{2, 4};
      case 0x6060: return Field{6, 1};
      case 0x60FF: return Field{7, 4};
      default: return {};
    }
  }
}
std::optional<Field> input_field(Layout layout, ObjectAddress object)
{
  if (object.subindex != 0) {return {};}
  if (layout == Layout::cst && object.index == 0x6078) {return Field{11, 2};}
  switch (object.index) {
    case 0x6041: return Field{0, 2};
    case 0x6064: return Field{2, 4};
    case 0x6061: return Field{6, 1};
    case 0x606C: return Field{7, 4};
    default: return {};
  }
}
}  // namespace

struct Master::Impl : public std::enable_shared_from_this<Master::Impl>
{
  struct Image
  {
    Layout layout{Layout::profile};
    std::array<std::uint8_t, max_process_bytes> output{};
    std::array<std::uint8_t, max_process_bytes> input{};
  };
  struct SlaveNode final : Node
  {
    SlaveNode(std::weak_ptr<Impl> owner, std::uint16_t position)
    : owner_(std::move(owner)), position_(position) {}
    std::uint16_t id() const noexcept override {return position_;}
    iswv::Result<SdoResponse> upload_blocking(ObjectAddress object, SdoOptions options) override
    {
      auto owner = owner_.lock();
      if (!owner) {
        return iswv::Result<SdoResponse>::failure(
          iswv::ErrorCode::transport_closed, "EtherCAT master is closed");
      }
      return owner->read(position_, object, options);
    }
    iswv::Result<void> download_blocking(
      ObjectAddress object, const std::uint8_t * data, std::size_t size,
      SdoOptions options) override
    {
      auto owner = owner_.lock();
      if (!owner) {
        return iswv::Result<void>::failure(
          iswv::ErrorCode::transport_closed, "EtherCAT master is closed");
      }
      return owner->write(position_, object, data, size, options);
    }
    std::weak_ptr<Impl> owner_;
    std::uint16_t position_;
  };

  explicit Impl(const Options & options)
  : exclusive_(master_mutex, std::try_to_lock), options_(options)
  {
    if (!exclusive_.owns_lock()) {throw std::runtime_error("an EtherCAT master is already open");}
    validate_options(options_);
  }
  ~Impl() {close();}

  // Mailbox operations are serialized. SOEM's Linux port has separate frame-index,
  // TX and RX locks, so a slow CoE/EEPROM response cannot suspend cyclic PDOs.
  iswv::Error sdo_error(std::uint16_t slave, ObjectAddress object)
  {
    std::string message = address_text(slave, object) + " failed";
    ec_errort error{};
    std::optional<std::uint32_t> abort;
    bool had_detail = false;
    while (ec_poperror(&error)) {
      if (error.Slave != slave) {continue;}
      if (error.Index != 0 && (error.Index != object.index || error.SubIdx != object.subindex)) {
        continue;
      }
      had_detail = true;
      message += " SOEM{type=" + std::to_string(static_cast<int>(error.Etype)) +
        " index=" + hex_text(error.Index, 4) + " sub=" +
        std::to_string(static_cast<unsigned>(error.SubIdx));
      if (error.Etype == EC_ERR_TYPE_SDO_ERROR && error.Index == object.index &&
        error.SubIdx == object.subindex)
      {
        abort = error.AbortCode;
        message += " abort=" + hex_text(static_cast<std::uint32_t>(error.AbortCode), 8);
      } else if (error.Etype == EC_ERR_TYPE_SDOINFO_ERROR) {
        message += " abort=" + hex_text(static_cast<std::uint32_t>(error.AbortCode), 8);
      } else {
        message += " detail=" + hex_text(error.ErrorCode, 4);
        if (error.Etype == EC_ERR_TYPE_EMERGENCY) {
          message += " error_reg=" + hex_text(error.ErrorReg, 2);
        }
      }
      message += "}";
    }
    if (!had_detail) {message += " (no SOEM error detail)";}
    return {abort ? iswv::ErrorCode::sdo_abort : iswv::ErrorCode::transport_error, message, abort};
  }

  iswv::Result<SdoResponse> sdo_read(
    std::uint16_t slave, ObjectAddress object, SdoOptions options)
  {
    SdoResponse result{};
    result.object = object;
    int size = static_cast<int>(result.data.size());
    const auto timeout =
      std::chrono::duration_cast<std::chrono::microseconds>(options.timeout).count();
    if (timeout <= 0 || timeout > 10000000) {
      return iswv::Result<SdoResponse>::failure(
        iswv::ErrorCode::invalid_argument,
        "invalid SDO timeout");
    }
    if (ec_SDOread(
        slave, object.index, object.subindex, FALSE, &size,
        result.data.data(), static_cast<int>(timeout)) <= 0)
    {
      return iswv::Result<SdoResponse>::failure(sdo_error(slave, object));
    }
    if (size < 1 || size > 4) {
      return iswv::Result<SdoResponse>::failure(
        iswv::ErrorCode::protocol_error,
        "invalid CoE object size");
    }
    result.size = static_cast<std::uint8_t>(size);
    return iswv::Result<SdoResponse>::success(result);
  }

  iswv::Result<void> sdo_write(
    std::uint16_t slave, ObjectAddress object, const std::uint8_t * data,
    std::size_t size, SdoOptions options)
  {
    const auto timeout =
      std::chrono::duration_cast<std::chrono::microseconds>(options.timeout).count();
    if (!data || size == 0 || size > 4 || timeout <= 0 || timeout > 10000000) {
      return iswv::Result<void>::failure(iswv::ErrorCode::invalid_argument, "invalid CoE write");
    }
    if (ec_SDOwrite(
        slave, object.index, object.subindex, FALSE, static_cast<int>(size),
        const_cast<std::uint8_t *>(data), static_cast<int>(timeout)) <= 0)
    {
      return iswv::Result<void>::failure(sdo_error(slave, object));
    }
    return iswv::Result<void>::success();
  }

  template<typename T>
  T startup_read(std::uint16_t slave, std::uint16_t index, std::uint8_t sub = 0)
  {
    auto result = sdo_read(slave, {index, sub}, {});
    if (!result) {throw std::runtime_error(result.error().message);}
    auto value = iswv::canopen::decode_little_endian<T>(
      result.value().data.data(),
      result.value().size);
    if (!value) {throw std::runtime_error(address_text(slave, {index, sub}) + " width mismatch");}
    return value.value();
  }
  template<typename T>
  void startup_download(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, T value)
  {
    const auto bytes = iswv::canopen::encode_little_endian(value);
    const auto result = sdo_write(slave, {index, sub}, bytes.data(), bytes.size(), {});
    if (!result) {throw std::runtime_error(result.error().message + "; PDO/CoE setup rejected");}
  }
  template<typename T>
  void verify_startup_value(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, T value)
  {
    if (startup_read<T>(slave, index, sub) != value) {
      throw std::runtime_error(address_text(slave, {index, sub}) + " setup readback mismatch");
    }
  }
  template<typename T>
  void startup_write(std::uint16_t slave, std::uint16_t index, std::uint8_t sub, T value)
  {
    startup_download<T>(slave, index, sub, value);
    verify_startup_value<T>(slave, index, sub, value);
  }
  void map_pdo(
    std::uint16_t slave, std::uint16_t index,
    std::initializer_list<std::uint32_t> entries)
  {
    startup_download<std::uint8_t>(slave, index, 0, 0);
    std::uint8_t sub = 0;
    for (auto entry : entries) {startup_download<std::uint32_t>(slave, index, ++sub, entry);}
    startup_write<std::uint8_t>(slave, index, 0, sub);
    sub = 0;
    for (auto entry : entries) {verify_startup_value<std::uint32_t>(slave, index, ++sub, entry);}
  }
  void assign_pdo(
    std::uint16_t slave, std::uint16_t index,
    std::initializer_list<std::uint16_t> entries)
  {
    startup_download<std::uint8_t>(slave, index, 0, 0);
    std::uint8_t sub = 0;
    for (auto entry : entries) {startup_download<std::uint16_t>(slave, index, ++sub, entry);}
    startup_write<std::uint8_t>(slave, index, 0, sub);
    sub = 0;
    for (auto entry : entries) {verify_startup_value<std::uint16_t>(slave, index, ++sub, entry);}
  }

  bool in_state(std::uint16_t slave, std::uint16_t requested, int timeout)
  {
    const auto state = ec_statecheck(slave, requested, timeout);
    // SOEM returns the low nibble; the stored state retains AL error bit 0x10.
    return state == requested && (ec_slave[slave].state & 0x1fU) == requested &&
           ec_slave[slave].ALstatuscode == 0;
  }

  Layout layout_for(std::uint16_t slave) const
  {
    return options_.cst_slave && *options_.cst_slave == slave ? Layout::cst : Layout::profile;
  }

  bool mode_allowed(Layout layout, std::int8_t mode) const
  {
    if (layout == Layout::cst) {return mode == 10;}
    return mode == 1 || mode == 3 || mode == 6;
  }

  void zero_output(std::uint16_t slave, Image & image)
  {
    image.output[0] = 0;
    image.output[1] = 0;
    const auto command = image.layout == Layout::cst ? Field{2, 2} : Field{7, 4};
    std::fill_n(image.output.begin() + command.offset, command.bytes, 0);
    if (ec_slave[slave].outputs) {
      std::copy_n(image.output.begin(), output_bytes(image.layout), ec_slave[slave].outputs);
    }
  }

  std::string slave_context(std::uint16_t slave) const
  {
    std::ostringstream out;
    out << " state=" << hex_text(ec_slave[slave].state, 2)
        << " al=" << hex_text(ec_slave[slave].ALstatuscode, 4)
        << " sii_vendor=" << hex_text(ec_slave[slave].eep_man)
        << " sii_product=" << hex_text(ec_slave[slave].eep_id)
        << " sii_revision=" << hex_text(ec_slave[slave].eep_rev)
        << " sii_name=" << printable_text(ec_slave[slave].name, sizeof(ec_slave[slave].name));
    return out.str();
  }

  std::string startup_name(std::uint16_t slave)
  {
    char name[128]{};
    int name_size = sizeof(name) - 1;
    const int wkc = ec_SDOread(slave, 0x1008, 0, FALSE, &name_size, name, EC_TIMEOUTRXM);
    if (wkc <= 0 || name_size <= 0 || name_size >= static_cast<int>(sizeof(name))) {
      std::cerr << "EtherCAT warning: cannot read Yiyou device name at slave " << slave
                << " wkc=" << wkc << " size=" << name_size << "; "
                << sdo_error(slave, {0x1008, 0}).message << ';' << slave_context(slave) << '\n';
      return "<unreadable>";
    }
    return printable_text(name, static_cast<std::size_t>(name_size));
  }

  void open()
  {
    try {
      if (!ec_init(options_.interface.c_str())) {
        throw std::runtime_error("cannot open EtherCAT raw socket (requires root/CAP_NET_RAW)");
      }
      opened_ = true;
      if (ec_config_init(FALSE) <= 0) {throw std::runtime_error("no EtherCAT slaves found");}
      for (auto slave : options_.slaves) {
        if (slave > ec_slavecount || !(ec_slave[slave].mbx_proto & ECT_MBXPROT_COE)) {
          throw std::runtime_error(
                  "selected EtherCAT slave missing or has no CoE: " + std::to_string(
                    slave));
        }
      }
      for (auto slave : options_.slaves) {
        if (!in_state(slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE)) {
          throw std::runtime_error(
                  "EtherCAT PRE-OP failed at slave " + std::to_string(slave) +
                  slave_context(slave));
        }
      }
      // Check every requested device before changing authority or any PDO mappings.
      for (auto slave : options_.slaves) {
        const auto coe_vendor = startup_read<std::uint32_t>(slave, 0x1018, 1);
        if (ec_slave[slave].eep_man != yiyou_vendor_id || coe_vendor != yiyou_vendor_id) {
          throw std::runtime_error(
                  "slave " + std::to_string(slave) + " is not a Yiyou EtherCAT drive: " +
                  "expected vendor=" + hex_text(yiyou_vendor_id) +
                  " sii_vendor=" + hex_text(ec_slave[slave].eep_man) +
                  " coe_vendor=" + hex_text(coe_vendor) + ";" + slave_context(slave));
        }
        const auto name = startup_name(slave);
        const auto status_word = startup_read<std::uint16_t>(slave, 0x6041);
        const auto state = iswv::cia402::decode_state(status_word);
        const auto raw_velocity = startup_read<std::int32_t>(slave, 0x606C);
        const auto stopped = startup_read<std::int8_t>(slave, 0x2707);
        if ((state != iswv::cia402::DriveState::switch_on_disabled &&
          state != iswv::cia402::DriveState::ready_to_switch_on &&
          state != iswv::cia402::DriveState::switched_on) ||
          stopped != 1)
        {
          throw std::runtime_error(
                  "slave " + std::to_string(
                    slave) + " must be disabled and stationary: status=" +
                  hex_text(status_word, 4) + " stopped=" + std::to_string(stopped) +
                  " velocity_raw=" + std::to_string(raw_velocity));
        }
        const auto source = startup_read<std::uint8_t>(slave, 0x2100);
        if (source > 2) {throw std::runtime_error("unknown Yiyou control source");}
        (void)startup_read<std::int8_t>(slave, 0x6060);  // Confirm actual OD width, not PDF table typo.
        Image image;
        image.layout = layout_for(slave);
        if (image.layout == Layout::cst) {
          (void)startup_read<std::int16_t>(slave, 0x6071);
          (void)startup_read<std::int16_t>(slave, 0x6078);
          image.output[4] = 10;
        } else {
          const auto position =
            iswv::canopen::encode_little_endian(startup_read<std::int32_t>(slave, 0x6064));
          std::copy(position.begin(), position.end(), image.output.begin() + 2);
          image.output[6] = 1;
        }
        images_.emplace(slave, image);
        std::cerr << "EtherCAT slave=" << slave << " name=" << name
                  << " vendor=" << coe_vendor
                  << " product=" << startup_read<std::uint32_t>(slave, 0x1018, 2)
                  << " serial=" << startup_read<std::uint32_t>(slave, 0x1018, 4) << '\n';
      }
      for (auto slave : options_.slaves) {
        ec_slave[slave].group = process_group;
        startup_write<std::uint8_t>(slave, 0x2100, 0, 1);
        // Explicit widths and verified readback. Fail on incompatible firmware
        // rather than interpreting an unknown wire layout.
        if (images_.at(slave).layout == Layout::cst) {
          map_pdo(slave, 0x1600, {0x60400010, 0x60710010, 0x60600008});
          map_pdo(slave, 0x1A00, {0x60410010, 0x60640020, 0x60610008, 0x606C0020, 0x60780010});
        } else {
          map_pdo(slave, 0x1600, {0x60400010, 0x607A0020, 0x60600008, 0x60FF0020});
          map_pdo(slave, 0x1A00, {0x60410010, 0x60640020, 0x60610008, 0x606C0020});
        }
        assign_pdo(slave, 0x1C12, {0x1600});
        assign_pdo(slave, 0x1C13, {0x1A00});
        // ESC process-data watchdog: 40 ns * (2498+2) * 1000 = 100 ms.
        // It still acts if the host process dies, unlike a host-only timeout.
        for (auto reg : {std::pair<std::uint16_t, std::uint16_t>{0x0400, 2498}, {0x0420, 1000}}) {
          std::uint16_t value = htoes(reg.second);
          if (ec_FPWR(
              ec_slave[slave].configadr, reg.first, sizeof(value), &value,
              EC_TIMEOUTRET) <= 0)
          {
            throw std::runtime_error("cannot configure EtherCAT process-data watchdog");
          }
          value = 0;
          if (ec_FPRD(
              ec_slave[slave].configadr, reg.first, sizeof(value), &value,
              EC_TIMEOUTRET) <= 0 ||
            etohs(value) != reg.second)
          {
            throw std::runtime_error("EtherCAT watchdog readback mismatch");
          }
        }
        ec_slave[slave].SM[2].SMflags |= htoel(0x40U);
      }
      // Only group 1 is mapped and promoted to OP. Unselected slaves remain PRE-OP.
      const int bytes = ec_config_map_group(io_map_.data(), process_group);
      std::size_t expected_bytes = 0;
      for (const auto & entry : images_) {
        expected_bytes += output_bytes(entry.second.layout) + input_bytes(entry.second.layout);
      }
      if (bytes != static_cast<int>(expected_bytes)) {
        throw std::runtime_error("unexpected EtherCAT process image size");
      }
      for (auto & entry : images_) {
        const auto slave = entry.first;
        if (ec_slave[slave].Obytes != output_bytes(entry.second.layout) ||
          ec_slave[slave].Ibytes != input_bytes(entry.second.layout) ||
          ec_slave[slave].Ostartbit != 0 ||
          ec_slave[slave].Istartbit != 0)
        {
          throw std::runtime_error("unexpected Yiyou PDO byte alignment");
        }
        std::copy_n(
          entry.second.output.begin(), output_bytes(
            entry.second.layout), ec_slave[slave].outputs);
      }
      (void)ec_configdc();
      // SOEM 1.4 initializes only the first global DC slave's group. Selection
      // may start later in the chain, so explicitly anchor our process group.
      ec_group[process_group].hasdc = TRUE;
      ec_group[process_group].DCnext = *std::min_element(
        options_.slaves.begin(), options_.slaves.end());
      for (auto slave : options_.slaves) {
        if (!ec_slave[slave].hasdc) {
          throw std::runtime_error("Yiyou slave has no distributed clock");
        }
        ec_dcsync0(slave, TRUE, 2000000, 0);
        if (!in_state(slave, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE)) {
          throw std::runtime_error("EtherCAT SAFE-OP failed at slave " + std::to_string(slave));
        }
      }
      expected_wkc_ = ec_group[process_group].outputsWKC * 2 + ec_group[process_group].inputsWKC;
      if (expected_wkc_ <= 0) {throw std::runtime_error("invalid EtherCAT working counter");}
      ec_send_processdata_group(process_group);
      (void)ec_receive_processdata_group(process_group, EC_TIMEOUTRET);
      for (auto slave : options_.slaves) {
        ec_slave[slave].state = EC_STATE_OPERATIONAL;
        ec_writestate(slave);
      }
      const auto deadline = std::chrono::steady_clock::now() + 2s;
      bool operational = false;
      while (std::chrono::steady_clock::now() < deadline) {
        ec_send_processdata_group(process_group);
        const int wkc = ec_receive_processdata_group(process_group, 1000);
        operational = wkc == expected_wkc_;
        for (auto slave : options_.slaves) {
          operational = in_state(slave, EC_STATE_OPERATIONAL, 1000) && operational;
        }
        if (operational) {break;}
        std::this_thread::sleep_for(cycle_period);
      }
      if (!operational) {throw std::runtime_error("EtherCAT OP transition/WKC failed");}
      for (auto & entry : images_) {
        std::copy_n(
          ec_slave[entry.first].inputs, input_bytes(entry.second.layout),
          entry.second.input.begin());
      }
      last_feedback_ = std::chrono::steady_clock::now();
      running_ = true;
      cycle_thread_ = std::thread([this] {cycle();});
    } catch (...) {close(); throw;}
  }

  void fault_locked(const std::string & message)
  {
    if (fault_.empty()) {fault_ = message; std::cerr << "EtherCAT fault: " << message << '\n';}
    for (auto & entry : images_) {
      zero_output(entry.first, entry.second);
    }
    changed_.notify_all();
  }
  void health_locked() const
  {
    if (!fault_.empty()) {throw std::runtime_error(fault_);}
    if (!running_) {throw std::runtime_error("EtherCAT master is closed");}
    if (std::chrono::steady_clock::now() - last_feedback_ > feedback_deadline) {
      throw std::runtime_error("EtherCAT feedback deadline exceeded");
    }
  }
  void cycle()
  {
    auto next = std::chrono::steady_clock::now();
    std::int64_t integral = 0;
    unsigned count = 0;
    while (running_) {
      const auto cycle_count = ++count;
      {
        std::lock_guard<std::mutex> guard(image_mutex_);
        if (std::chrono::steady_clock::now() - last_feedback_ > feedback_deadline) {
          fault_locked("EtherCAT cyclic deadline exceeded (20 ms)");
        }
        for (auto & entry : images_) {
          std::copy_n(
            entry.second.output.begin(), output_bytes(entry.second.layout),
            ec_slave[entry.first].outputs);
        }
        sent_generation_ = requested_generation_;
      }
      ec_send_processdata_group(process_group);
      const int wkc = ec_receive_processdata_group(process_group, 1000);
      std::ostringstream fault;
      if (wkc != expected_wkc_) {
        fault << "EtherCAT process WKC mismatch: actual=" << wkc << " expected=" << expected_wkc_;
      }
      if (cycle_count % 10 == 0) {
        for (auto slave : options_.slaves) {
          std::array<std::uint8_t, 6> al{};
          const int al_wkc = ec_FPRD(
            ec_slave[slave].configadr, ECT_REG_ALSTAT, al.size(), al.data(), 1000);
          if (al_wkc <= 0) {
            if (fault.tellp() > 0) {fault << "; ";}
            fault << "AL poll timeout at slave " << slave << " fprd_wkc=" << al_wkc;
            continue;
          }
          const std::uint16_t state =
            static_cast<std::uint16_t>(al[0]) | (static_cast<std::uint16_t>(al[1]) << 8);
          const std::uint16_t code =
            static_cast<std::uint16_t>(al[4]) | (static_cast<std::uint16_t>(al[5]) << 8);
          if ((state & 0x1fU) != EC_STATE_OPERATIONAL) {
            if (fault.tellp() > 0) {fault << "; ";}
            fault << "AL non-OP at slave " << slave << " state=" << hex_text(state, 4)
                  << " al=" << hex_text(code, 4);
          }
        }
      }
      {
        std::lock_guard<std::mutex> guard(image_mutex_);
        if (fault.tellp() > 0) {
          const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - last_feedback_);
          if (wkc == expected_wkc_) {
            fault << "; process_wkc_actual=" << wkc << " process_wkc_expected=" << expected_wkc_;
          }
          fault << "; cycle=" << cycle_count << " last_feedback_ms=" << age.count()
                << "; outputs disabled until process restart";
          fault_locked(fault.str());
        } else {
          for (auto & entry : images_) {
            std::copy_n(
              ec_slave[entry.first].inputs, input_bytes(entry.second.layout),
              entry.second.input.begin());
          }
          last_feedback_ = std::chrono::steady_clock::now();
          applied_generation_ = sent_generation_;
          changed_.notify_all();
        }
      }
      // Bound the DC PI correction; scheduling is best effort on normal Linux.
      auto phase = (ec_DCtime - 50000) % 2000000;
      if (phase > 1000000) {phase -= 2000000;}
      if (phase < -1000000) {phase += 2000000;}
      integral = std::clamp<std::int64_t>(
        integral + (phase > 0 ? 1 : (phase < 0 ? -1 : 0)),
        -100000, 100000);
      const auto correction = std::clamp<std::int64_t>(-phase / 100 - integral / 20, -50000, 50000);
      next += cycle_period + std::chrono::nanoseconds(correction);
      if (next < std::chrono::steady_clock::now()) {next = std::chrono::steady_clock::now();}
      std::this_thread::sleep_until(next);
    }
  }

  iswv::Result<SdoResponse> read(std::uint16_t slave, ObjectAddress object, SdoOptions options)
  {
    try {
      {
        std::lock_guard<std::mutex> guard(image_mutex_);
        health_locked();
        const auto image = images_.find(slave);
        if (image == images_.end()) {
          throw std::invalid_argument("EtherCAT slave was not selected");
        }
        if (auto field = input_field(image->second.layout, object)) {
          SdoResponse result{};
          result.object = object;
          result.size = static_cast<std::uint8_t>(field->bytes);
          std::copy_n(
            image->second.input.begin() + field->offset, field->bytes, result.data.begin());
          return iswv::Result<SdoResponse>::success(result);
        }
      }
      std::lock_guard<std::mutex> mailbox(mailbox_mutex_);
      auto result = sdo_read(slave, object, options);
      std::lock_guard<std::mutex> guard(image_mutex_);
      health_locked();
      return result;
    } catch (const std::exception & error) {
      return iswv::Result<SdoResponse>::failure(iswv::ErrorCode::transport_error, error.what());
    }
  }
  iswv::Result<void> write(
    std::uint16_t slave, ObjectAddress object, const std::uint8_t * data,
    std::size_t size, SdoOptions options)
  {
    try {
      {
        std::unique_lock<std::mutex> guard(image_mutex_);
        health_locked();
        auto image = images_.find(slave);
        if (image == images_.end()) {
          throw std::invalid_argument("EtherCAT slave was not selected");
        }
        if (object.index == 0x6071 && image->second.layout != Layout::cst) {
          throw std::invalid_argument("0x6071 target torque is only PDO-mapped on the CST slave");
        }
        if (object.index == 0x6060 && data && size == 1) {
          const auto mode = static_cast<std::int8_t>(*data);
          if (!mode_allowed(image->second.layout, mode)) {
            throw std::invalid_argument(
                    "0x6060 mode is not compatible with this EtherCAT PDO layout");
          }
        }
        if (auto field = output_field(image->second.layout, object)) {
          if (!data || size != field->bytes || options.timeout.count() <= 0) {
            throw std::invalid_argument("PDO object width/timeout mismatch");
          }
          std::copy_n(data, size, image->second.output.begin() + field->offset);
          const auto generation = ++requested_generation_;
          if (!changed_.wait_for(
              guard, options.timeout, [&] {
                return applied_generation_ >= generation || !fault_.empty() || !running_;
              }))
          {
            fault_locked("EtherCAT PDO command delivery timed out");
          }
          health_locked();
          return iswv::Result<void>::success();
        }
      }
      std::lock_guard<std::mutex> mailbox(mailbox_mutex_);
      auto result = sdo_write(slave, object, data, size, options);
      std::lock_guard<std::mutex> guard(image_mutex_);
      health_locked();
      return result;
    } catch (const std::exception & error) {
      return iswv::Result<void>::failure(iswv::ErrorCode::transport_error, error.what());
    }
  }
  void close() noexcept
  {
    running_ = false;
    changed_.notify_all();
    if (cycle_thread_.joinable()) {cycle_thread_.join();}
    if (!opened_) {return;}
    for (auto & entry : images_) {
      const auto slave = entry.first;
      if (ec_slave[slave].outputs && ec_slave[slave].Obytes == output_bytes(entry.second.layout)) {
        zero_output(slave, entry.second);
      }
    }
    if (expected_wkc_ > 0) {
      ec_send_processdata_group(process_group);
      (void)ec_receive_processdata_group(process_group, EC_TIMEOUTRET);
    }
    for (auto & entry : images_) {
      const auto slave = entry.first;
      if (ec_slave[slave].hasdc) {ec_dcsync0(slave, FALSE, 0, 0);}
      ec_slave[slave].state = EC_STATE_INIT;
      ec_writestate(slave);
    }
    ec_close();
    opened_ = false;
  }

  std::unique_lock<std::mutex> exclusive_;
  Options options_;
  bool opened_{false};
  std::atomic<bool> running_{false};
  std::array<std::uint8_t, EC_MAXSLAVE * max_process_bytes * 2> io_map_{};
  std::map<std::uint16_t, Image> images_;
  std::mutex image_mutex_;
  std::mutex mailbox_mutex_;
  std::condition_variable changed_;
  std::thread cycle_thread_;
  std::string fault_;
  int expected_wkc_{0};
  std::uint64_t requested_generation_{0}, sent_generation_{0}, applied_generation_{0};
  std::chrono::steady_clock::time_point last_feedback_{};
};

Master::Master(const Options & options)
: impl_(std::make_shared<Impl>(options)) {impl_->open();}
Master::~Master() = default;
std::shared_ptr<Node> Master::node(std::uint16_t position)
{
  if (impl_->images_.count(position) == 0) {
    throw std::invalid_argument("EtherCAT slave was not selected");
  }
  return std::make_shared<Impl::SlaveNode>(impl_, position);
}
void Master::check_health() const
{
  std::lock_guard<std::mutex> guard(impl_->image_mutex_);
  impl_->health_locked();
}
}  // namespace chassis::ethercat
