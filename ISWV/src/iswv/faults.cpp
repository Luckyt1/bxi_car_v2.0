#include "iswv/iswv/faults.hpp"

#include <array>
#include <utility>

namespace iswv {
namespace {

constexpr std::array<std::string_view, 16> error_1_names{
    "extended fault (see error-status-2)",
    "encoder communication fault",
    "encoder internal fault",
    "encoder CRC fault",
    "drive over-temperature",
    "DC bus over-voltage",
    "DC bus under-voltage",
    "drive or motor short circuit / over-current",
    "braking resistor fault",
    "following error exceeded",
    "logic voltage low",
    "motor or drive overload",
    "input pulse frequency too high",
    "motor over-temperature",
    "encoder information fault",
    "non-volatile memory fault"};

constexpr std::array<std::string_view, 16> error_2_names{
    "current sensor fault",
    "software watchdog reset",
    "abnormal interrupt",
    "MCU identification fault",
    "motor configuration fault",
    "reserved error-status-2 bit 5",
    "reserved error-status-2 bit 6",
    "reserved error-status-2 bit 7",
    "pre-enable alarm",
    "positive limit alarm",
    "negative limit alarm",
    "SPI fault",
    "CAN bus interruption",
    "full-closed-loop direction fault",
    "main encoder ABZ fault",
    "main encoder count fault"};

}  // namespace

std::vector<FaultInfo> decode_fault_status(std::uint16_t error_status,
                                           std::uint16_t error_status_2)
{
    std::vector<FaultInfo> faults;
    for (std::uint8_t bit = 0; bit < 16; ++bit) {
        if ((error_status & (1U << bit)) != 0U) {
            faults.push_back(FaultInfo{1, bit, error_1_names[bit]});
        }
        if ((error_status_2 & (1U << bit)) != 0U) {
            faults.push_back(FaultInfo{2, bit, error_2_names[bit]});
        }
    }
    return faults;
}

std::string_view emergency_code_description(std::uint16_t code) noexcept
{
    switch (code) {
    case 0x7331:
        return "communication encoder disconnected or not responding";
    case 0x7320:
        return "communication encoder internal fault";
    case 0x7330:
        return "communication encoder checksum fault";
    case 0x4210:
        return "drive over-temperature";
    case 0x3210:
        return "DC bus over-voltage";
    case 0x3220:
        return "DC bus under-voltage";
    case 0x2320:
        return "drive power stage or motor short circuit";
    case 0x2321:
        return "current sampling saturation";
    case 0x7110:
        return "braking resistor fault";
    case 0x8611:
        return "following error exceeded";
    case 0x5112:
        return "logic voltage low";
    case 0x2350:
        return "motor or drive overload";
    case 0x8A80:
        return "input pulse frequency too high";
    case 0x4310:
        return "motor over-temperature";
    case 0x6310:
        return "EEPROM data fault";
    case 0x5210:
        return "current sensor fault";
    case 0x6010:
        return "software watchdog reset";
    case 0x6011:
        return "abnormal interrupt";
    case 0x7400:
        return "MCU fault";
    case 0x6320:
        return "motor model configuration fault";
    case 0x6321:
        return "motor power phase missing";
    case 0x5443:
        return "pre-enable alarm";
    case 0x5442:
        return "positive limit alarm";
    case 0x5441:
        return "negative limit alarm";
    case 0x6012:
        return "SPI fault";
    case 0x8100:
        return "CAN bus communication error";
    case 0x81FF:
        return "CAN bus communication timeout";
    case 0x8A81:
        return "full-closed-loop check fault";
    case 0x7382:
        return "main encoder ABZ fault";
    case 0x7306:
        return "main encoder count fault";
    default:
        return "unknown iSWV emergency code";
    }
}

}  // namespace iswv
