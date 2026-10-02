#pragma once

#include "iswv/canopen/types.hpp"

#include <optional>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace chassis::ethercat
{
// CoE uses the same typed object dictionary as CANopen, but never CAN frames/NMT.
// This small boundary also permits offline drive/state-machine tests.
class Node
{
public:
  virtual ~Node() = default;
  virtual std::uint16_t id() const noexcept = 0;
  virtual iswv::Result<iswv::canopen::SdoResponse> upload_blocking(
    iswv::canopen::ObjectAddress object, iswv::canopen::SdoOptions options = {}) = 0;
  virtual iswv::Result<void> download_blocking(
    iswv::canopen::ObjectAddress object, const std::uint8_t * data, std::size_t size,
    iswv::canopen::SdoOptions options = {}) = 0;

  template<typename T>
  iswv::Result<T> read(
    iswv::canopen::Object<T> object,
    iswv::canopen::SdoOptions options = {})
  {
    auto response = upload_blocking(object.address, options);
    if (!response) {return iswv::Result<T>::failure(response.error());}
    return iswv::canopen::decode_little_endian<T>(
      response.value().data.data(),
      response.value().size);
  }
  template<typename T, typename U>
  iswv::Result<void> write(
    iswv::canopen::Object<T> object, U value, iswv::canopen::SdoOptions options = {})
  {
    static_assert(std::is_convertible_v<U, T>, "object value is not convertible");
    const auto bytes = iswv::canopen::encode_little_endian(static_cast<T>(value));
    return download_blocking(object.address, bytes.data(), bytes.size(), options);
  }
};

struct Options
{
  std::string interface;
  // Physical chain positions (1 based), not CAN Node-IDs. Maximum 199 for SOEM 1.4.
  std::vector<std::uint16_t> slaves;
  std::optional<std::uint16_t> cst_slave {};
};

// Pure argument validation, before a caller opens the shared motor power switch.
inline void validate_selection(const Options & options)
{
  if (options.interface.empty() || options.interface.size() > 15 ||
    options.interface == "lo" ||
    options.interface.find_first_not_of(
      "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.:-") != std::string::npos)
  {
    throw std::invalid_argument("specify a physical EtherCAT --interface; lo is loopback");
  }
  if (options.slaves.empty()) {throw std::invalid_argument("select EtherCAT slave positions");}
  std::set<std::uint16_t> positions;
  for (auto slave : options.slaves) {
    if (slave == 0 || slave > 199 || !positions.insert(slave).second) {
      throw std::invalid_argument("EtherCAT positions must be unique and in 1..199");
    }
  }
  if (options.cst_slave && positions.count(*options.cst_slave) == 0) {
    throw std::invalid_argument("CST EtherCAT slave must be one of the selected positions");
  }
}

// Owns the EtherCAT session and cyclic process data, not shared board power.
// Construct one master for the selected chain and share its nodes among motors.
class Master
{
public:
  explicit Master(const Options & options);
  ~Master();
  Master(const Master &) = delete;
  Master & operator=(const Master &) = delete;
  std::shared_ptr<Node> node(std::uint16_t position);
  // Throws on a latched WKC/state/deadline fault; never automatically re-enables.
  void check_health() const;

private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};
}  // namespace chassis::ethercat
