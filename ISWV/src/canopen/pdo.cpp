#include "iswv/canopen/pdo.hpp"

#include <algorithm>
#include <numeric>

namespace iswv::canopen {

Result<void> PdoCodec::validate(const PdoConfiguration& configuration)
{
    if (configuration.number < 1 || configuration.number > 4) {
        return Result<void>::failure(ErrorCode::invalid_argument,
                                     "PDO number must be in the range 1..4");
    }
    if (configuration.mapping.size() > 8) {
        return Result<void>::failure(ErrorCode::invalid_argument,
                                     "a PDO can contain at most eight mapped objects");
    }
    std::size_t bits = 0;
    for (const auto& entry : configuration.mapping) {
        if (entry.bit_length == 0 || entry.bit_length > 64 ||
            (entry.bit_length % 8U) != 0U) {
            return Result<void>::failure(ErrorCode::invalid_argument,
                                         "PDO entries must be byte aligned and non-empty");
        }
        bits += entry.bit_length;
    }
    if (bits > 64) {
        return Result<void>::failure(ErrorCode::invalid_argument,
                                     "PDO mapping exceeds eight bytes");
    }
    if (configuration.cob_id > 0x1FFFFFFFU) {
        return Result<void>::failure(ErrorCode::invalid_argument,
                                     "PDO COB-ID exceeds the CAN extended-ID range");
    }
    return Result<void>::success();
}

Result<std::vector<std::uint8_t>> PdoCodec::encode(
    const std::vector<PdoMappingEntry>& mapping,
    const std::function<Result<MappedValue>(ObjectAddress)>& value_provider)
{
    std::vector<std::uint8_t> payload;
    payload.reserve(8);
    for (const auto& entry : mapping) {
        const auto expected_size = static_cast<std::size_t>(entry.bit_length / 8U);
        auto value = value_provider(entry.object);
        if (!value) {
            return Result<std::vector<std::uint8_t>>::failure(value.error());
        }
        if (value.value().object != entry.object || value.value().size != expected_size) {
            return Result<std::vector<std::uint8_t>>::failure(
                ErrorCode::invalid_argument,
                "mapped PDO value does not match the configured object or size");
        }
        payload.insert(payload.end(), value.value().data.begin(),
                       value.value().data.begin() + value.value().size);
    }
    if (payload.size() > 8) {
        return Result<std::vector<std::uint8_t>>::failure(
            ErrorCode::invalid_argument, "encoded PDO is larger than eight bytes");
    }
    return Result<std::vector<std::uint8_t>>::success(std::move(payload));
}

Result<std::vector<MappedValue>> PdoCodec::decode(
    const std::vector<PdoMappingEntry>& mapping,
    const std::uint8_t* data,
    std::size_t size)
{
    if (data == nullptr && size != 0) {
        return Result<std::vector<MappedValue>>::failure(
            ErrorCode::invalid_argument, "null PDO data pointer");
    }
    std::size_t expected = 0;
    for (const auto& entry : mapping) {
        expected += entry.bit_length / 8U;
    }
    if (expected != size) {
        return Result<std::vector<MappedValue>>::failure(
            ErrorCode::protocol_error, "PDO payload size does not match its mapping");
    }

    std::vector<MappedValue> values;
    values.reserve(mapping.size());
    std::size_t offset = 0;
    for (const auto& entry : mapping) {
        MappedValue value;
        value.object = entry.object;
        value.size = static_cast<std::uint8_t>(entry.bit_length / 8U);
        std::copy_n(data + offset, value.size, value.data.begin());
        offset += value.size;
        values.push_back(value);
    }
    return Result<std::vector<MappedValue>>::success(std::move(values));
}

std::uint32_t PdoCodec::mapping_value(const PdoMappingEntry& entry) noexcept
{
    return (static_cast<std::uint32_t>(entry.object.index) << 16U) |
           (static_cast<std::uint32_t>(entry.object.subindex) << 8U) |
           entry.bit_length;
}

std::uint16_t PdoCodec::communication_index(PdoDirection direction,
                                             std::uint8_t number) noexcept
{
    const std::uint16_t base =
        direction == PdoDirection::receive ? 0x1400U : 0x1800U;
    return static_cast<std::uint16_t>(base + number - 1U);
}

std::uint16_t PdoCodec::mapping_index(PdoDirection direction,
                                       std::uint8_t number) noexcept
{
    const std::uint16_t base =
        direction == PdoDirection::receive ? 0x1600U : 0x1A00U;
    return static_cast<std::uint16_t>(base + number - 1U);
}

std::uint32_t PdoCodec::default_cob_id(PdoDirection direction,
                                        std::uint8_t number,
                                        std::uint8_t node_id) noexcept
{
    const std::uint32_t receive_base[] = {0x200U, 0x300U, 0x400U, 0x500U};
    const std::uint32_t transmit_base[] = {0x180U, 0x280U, 0x380U, 0x480U};
    if (number < 1 || number > 4) {
        return 0;
    }
    const auto index = static_cast<std::size_t>(number - 1U);
    return (direction == PdoDirection::receive ? receive_base[index]
                                                : transmit_base[index]) +
           node_id;
}

}  // namespace iswv::canopen
