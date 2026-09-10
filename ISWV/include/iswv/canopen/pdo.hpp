#pragma once

#include "iswv/canopen/types.hpp"

#include <algorithm>
#include <functional>
#include <vector>

namespace iswv::canopen {

class PdoCodec {
public:
    static Result<void> validate(const PdoConfiguration& configuration);

    static Result<std::vector<std::uint8_t>> encode(
        const std::vector<PdoMappingEntry>& mapping,
        const std::function<Result<MappedValue>(ObjectAddress)>& value_provider);

    static Result<std::vector<MappedValue>> decode(
        const std::vector<PdoMappingEntry>& mapping,
        const std::uint8_t* data,
        std::size_t size);

    static std::uint32_t mapping_value(const PdoMappingEntry& entry) noexcept;
    static std::uint16_t communication_index(PdoDirection direction,
                                             std::uint8_t number) noexcept;
    static std::uint16_t mapping_index(PdoDirection direction,
                                       std::uint8_t number) noexcept;
    static std::uint32_t default_cob_id(PdoDirection direction,
                                        std::uint8_t number,
                                        std::uint8_t node_id) noexcept;
};

template <typename T, typename U>
MappedValue mapped_value(Object<T> object, U value)
{
    static_assert(std::is_convertible_v<U, T>, "mapped value is not convertible");
    const auto bytes = encode_little_endian(static_cast<T>(value));
    MappedValue mapped;
    mapped.object = object.address;
    mapped.size = static_cast<std::uint8_t>(bytes.size());
    std::copy(bytes.begin(), bytes.end(), mapped.data.begin());
    return mapped;
}

}  // namespace iswv::canopen
