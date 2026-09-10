#include "test.hpp"
#include "test_helpers.hpp"

#include "iswv/canopen/master.hpp"

#include <chrono>
#include <future>
#include <memory>

using namespace std::chrono_literals;

TEST_CASE("PDO codec encodes and decodes configured mappings")
{
    using namespace iswv::canopen;
    const std::vector<PdoMappingEntry> mapping{{{0x6040, 0}, 16},
                                                {{0x60FF, 0}, 32}};
    auto encoded = PdoCodec::encode(mapping, [](ObjectAddress object) {
        if (object.index == 0x6040) {
            return iswv::Result<MappedValue>::success(
                mapped_value(Object<std::uint16_t>{object, "control"}, 0x000F));
        }
        return iswv::Result<MappedValue>::success(
            mapped_value(Object<std::int32_t>{object, "velocity"}, -100));
    });
    CHECK(encoded);
    CHECK_EQ(encoded.value().size(), 6U);
    CHECK_EQ(encoded.value()[0], 0x0FU);
    CHECK_EQ(encoded.value()[2], 0x9CU);
    CHECK_EQ(encoded.value()[5], 0xFFU);

    auto decoded = PdoCodec::decode(mapping, encoded.value().data(), encoded.value().size());
    CHECK(decoded);
    CHECK_EQ(decoded.value().size(), 2U);
    auto velocity = decode_little_endian<std::int32_t>(decoded.value()[1].data.data(), 4);
    CHECK(velocity);
    CHECK_EQ(velocity.value(), -100);
}

TEST_CASE("PDO configuration writes standard objects and routes TPDO")
{
    auto transport = std::make_shared<iswv::FakeTransport>();
    iswv::canopen::CanopenMaster master(transport);
    auto node = master.node(4);
    transport->set_send_hook([transport](const iswv::CanFrame& request) {
        if (request.id >= 0x600 && request.id <= 0x67F) {
            transport->inject(iswv::test::sdo_download_response(request));
        }
    });

    iswv::canopen::PdoConfiguration configuration;
    configuration.direction = iswv::canopen::PdoDirection::transmit;
    configuration.number = 1;
    configuration.cob_id = 0x184;
    configuration.mapping = {{{0x6041, 0}, 16}, {{0x6063, 0}, 32}};
    CHECK(node->configure_pdo(configuration));

    std::promise<iswv::canopen::PdoEvent> promise;
    auto future = promise.get_future();
    auto subscription = node->on_tpdo(1, [&promise](const auto& event) {
        promise.set_value(event);
    });
    iswv::CanFrame frame;
    frame.id = 0x184;
    frame.size = 6;
    frame.data[0] = 0x37;
    frame.data[2] = 0xA0;
    frame.data[3] = 0x86;
    frame.data[4] = 0x01;
    transport->inject(frame);
    CHECK(future.wait_for(100ms) == std::future_status::ready);
    auto decoded = node->decode_tpdo(future.get());
    CHECK(decoded);
    CHECK_EQ(decoded.value().size(), 2U);
}
