#pragma once

#include <cstdint>
#include <string_view>
#include <vector>

namespace iswv
{

struct FaultInfo
{
  std::uint8_t register_number{0};
  std::uint8_t bit{0};
  std::string_view name;
};

std::vector<FaultInfo> decode_fault_status(
  std::uint16_t error_status,
  std::uint16_t error_status_2);
std::string_view emergency_code_description(std::uint16_t code) noexcept;

}  // namespace iswv
