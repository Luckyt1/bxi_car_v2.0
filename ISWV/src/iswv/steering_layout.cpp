#include "iswv/iswv/steering_layout.hpp"

#include <array>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <string_view>

namespace iswv {
namespace {

std::string trim(std::string value)
{
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

Result<void> parse_uint(const std::string& value, std::uint32_t& output,
                        const char* key, std::uint32_t maximum = std::numeric_limits<std::uint32_t>::max())
{
    char* end = nullptr;
    errno = 0;
    const auto parsed = std::strtoul(value.c_str(), &end, 0);
    if (errno != 0 || end == value.c_str() || *trim(end).c_str() != '\0' ||
        parsed > maximum) {
        return Result<void>::failure(ErrorCode::invalid_argument,
                                     std::string("invalid ") + key + ": " + value);
    }
    output = static_cast<std::uint32_t>(parsed);
    return Result<void>::success();
}

Result<void> parse_double(const std::string& value, double& output, const char* key)
{
    char* end = nullptr;
    errno = 0;
    const auto parsed = std::strtod(value.c_str(), &end);
    if (errno != 0 || end == value.c_str() || *trim(end).c_str() != '\0' ||
        !std::isfinite(parsed)) {
        return Result<void>::failure(ErrorCode::invalid_argument,
                                     std::string("invalid ") + key + ": " + value);
    }
    output = parsed;
    return Result<void>::success();
}

Result<void> parse_bool(const std::string& value, bool& output, const char* key)
{
    if (value == "true" || value == "1") {
        output = true;
        return Result<void>::success();
    }
    if (value == "false" || value == "0") {
        output = false;
        return Result<void>::success();
    }
    return Result<void>::failure(ErrorCode::invalid_argument,
                                 std::string("invalid ") + key + ": " + value);
}

std::size_t wheel_index(const std::string& section)
{
    static constexpr std::array<std::string_view, 4> names{
        "front_left", "rear_left", "rear_right", "front_right"};
    for (std::size_t index = 0; index < names.size(); ++index) {
        if (section == names[index]) {
            return index;
        }
    }
    return names.size();
}

}  // namespace

Result<SteeringLayoutConfig> SteeringLayoutConfig::from_file(const std::string& path)
{
    std::ifstream input(path);
    if (!input) {
        return Result<SteeringLayoutConfig>::failure(
            ErrorCode::not_found, "cannot open steering config: " + path);
    }

    SteeringLayoutConfig configuration;
    std::string section;
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        const auto comment = line.find_first_of("#;");
        if (comment != std::string::npos) {
            line.erase(comment);
        }
        line = trim(std::move(line));
        if (line.empty()) {
            continue;
        }
        if (line.front() == '[' && line.back() == ']') {
            section = trim(line.substr(1, line.size() - 2));
            continue;
        }
        const auto equals = line.find('=');
        if (equals == std::string::npos || section.empty()) {
            return Result<SteeringLayoutConfig>::failure(
                ErrorCode::invalid_argument,
                "invalid steering config line " + std::to_string(line_number));
        }
        const auto key = trim(line.substr(0, equals));
        const auto value = trim(line.substr(equals + 1));
        if (key.empty() || value.empty()) {
            return Result<SteeringLayoutConfig>::failure(
                ErrorCode::invalid_argument,
                "invalid steering config line " + std::to_string(line_number));
        }

        std::uint32_t integer = 0;
        double real = 0.0;
        bool boolean = false;
        Result<void> parsed = Result<void>::success();
        const auto index = wheel_index(section);
        if (section == "motor") {
            if (key == "encoder_resolution") {
                parsed = parse_uint(value, integer, key.c_str());
                configuration.encoder_resolution = integer;
            } else if (key == "gear_ratio") {
                parsed = parse_double(value, real, key.c_str());
                configuration.gear_ratio = real;
            } else if (key == "wheel_diameter_m") {
                parsed = parse_double(value, real, key.c_str());
                configuration.wheel_diameter_m = real;
            } else {
                parsed = Result<void>::failure(ErrorCode::invalid_argument,
                                               "unknown motor key: " + key);
            }
        } else if (section == "position") {
            if (key == "profile_velocity_rpm") {
                parsed = parse_double(value, real, key.c_str());
                configuration.profile_velocity_rpm = real;
            } else if (key == "acceleration_rps2") {
                parsed = parse_double(value, real, key.c_str());
                configuration.acceleration_rps2 = real;
            } else if (key == "deceleration_rps2") {
                parsed = parse_double(value, real, key.c_str());
                configuration.deceleration_rps2 = real;
            } else {
                parsed = Result<void>::failure(ErrorCode::invalid_argument,
                                               "unknown position key: " + key);
            }
        } else if (section == "pdo") {
            if (key == "rpdo_transmission_type") {
                parsed = parse_uint(value, integer, key.c_str(), 255);
                configuration.pdo.rpdo_transmission_type = static_cast<std::uint8_t>(integer);
            } else if (key == "tpdo_transmission_type") {
                parsed = parse_uint(value, integer, key.c_str(), 255);
                configuration.pdo.tpdo_transmission_type = static_cast<std::uint8_t>(integer);
            } else if (key == "tpdo_inhibit_time") {
                parsed = parse_uint(value, integer, key.c_str(), 65535);
                configuration.pdo.tpdo_inhibit_time = static_cast<std::uint16_t>(integer);
            } else if (key == "tpdo_event_timer_ms") {
                parsed = parse_uint(value, integer, key.c_str(), 65535);
                configuration.pdo.tpdo_event_timer_ms = static_cast<std::uint16_t>(integer);
            } else {
                parsed = Result<void>::failure(ErrorCode::invalid_argument,
                                               "unknown pdo key: " + key);
            }
        } else if (index < configuration.node_ids.size()) {
            if (key == "node_id") {
                parsed = parse_uint(value, integer, key.c_str(), 127);
                configuration.node_ids[index] = static_cast<std::uint8_t>(integer);
            } else if (key == "inverted") {
                parsed = parse_bool(value, boolean, key.c_str());
                configuration.inverted[index] = boolean;
            } else {
                parsed = Result<void>::failure(ErrorCode::invalid_argument,
                                               "unknown wheel key: " + key);
            }
        } else {
            parsed = Result<void>::failure(ErrorCode::invalid_argument,
                                           "unknown steering config section: " + section);
        }
        if (!parsed) {
            return Result<SteeringLayoutConfig>::failure(
                parsed.error().code,
                parsed.error().message + " (line " + std::to_string(line_number) + ")");
        }
    }

    for (const auto node_id : configuration.node_ids) {
        if (node_id == 0 || node_id > 127) {
            return Result<SteeringLayoutConfig>::failure(
                ErrorCode::invalid_argument, "wheel node_id must be in range 1..127");
        }
    }
    for (std::size_t i = 0; i < configuration.node_ids.size(); ++i) {
        for (std::size_t j = i + 1; j < configuration.node_ids.size(); ++j) {
            if (configuration.node_ids[i] == configuration.node_ids[j]) {
                return Result<SteeringLayoutConfig>::failure(
                    ErrorCode::invalid_argument, "wheel node_ids must be distinct");
            }
        }
    }
    if (configuration.encoder_resolution == 0 || configuration.gear_ratio <= 0.0 ||
        configuration.wheel_diameter_m <= 0.0 || configuration.profile_velocity_rpm < 0.0 ||
        configuration.acceleration_rps2 < 0.0 || configuration.deceleration_rps2 < 0.0) {
        return Result<SteeringLayoutConfig>::failure(
            ErrorCode::invalid_argument, "invalid steering motor parameters");
    }
    return Result<SteeringLayoutConfig>::success(std::move(configuration));
}

}  // namespace iswv
