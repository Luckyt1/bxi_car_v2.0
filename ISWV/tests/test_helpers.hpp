#pragma once

#include "iswv/canopen/types.hpp"
#include "iswv/fake_transport.hpp"

#include <algorithm>
#include <cstdint>

namespace iswv::test {

inline CanFrame sdo_download_response(const CanFrame& request)
{
    CanFrame response;
    response.id = 0x580U + (request.id - 0x600U);
    response.size = 8;
    response.data[0] = 0x60;
    response.data[1] = request.data[1];
    response.data[2] = request.data[2];
    response.data[3] = request.data[3];
    return response;
}

template <typename T>
CanFrame sdo_upload_response(const CanFrame& request, T value)
{
    const auto bytes = canopen::encode_little_endian(value);
    CanFrame response;
    response.id = 0x580U + (request.id - 0x600U);
    response.size = 8;
    response.data[0] = sizeof(T) == 1 ? 0x4F : (sizeof(T) == 2 ? 0x4B : 0x43);
    response.data[1] = request.data[1];
    response.data[2] = request.data[2];
    response.data[3] = request.data[3];
    std::copy(bytes.begin(), bytes.end(), response.data.begin() + 4);
    return response;
}

inline canopen::ObjectAddress request_object(const CanFrame& request)
{
    return canopen::ObjectAddress{
        static_cast<std::uint16_t>(request.data[1] |
                                   (static_cast<std::uint16_t>(request.data[2]) << 8U)),
        request.data[3]};
}

}  // namespace iswv::test
