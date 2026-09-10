#include "test.hpp"
#include "test_helpers.hpp"

#include "iswv/canopen/master.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;

TEST_CASE("SDO upload and download use standard IDs and little endian")
{
    auto transport = std::make_shared<iswv::FakeTransport>();
    iswv::canopen::CanopenMaster master(transport);
    auto node = master.node(1);

    transport->set_send_hook([transport](const iswv::CanFrame& request) {
        const auto object = iswv::test::request_object(request);
        if (request.data[0] == 0x40 && object.index == 0x6041) {
            transport->inject(iswv::test::sdo_upload_response(request, std::uint16_t{0x0037}));
        } else {
            transport->inject(iswv::test::sdo_download_response(request));
        }
    });

    const iswv::canopen::Object<std::uint16_t> status{{0x6041, 0}, "status"};
    auto read = node->read(status);
    CHECK(read);
    CHECK_EQ(read.value(), 0x0037U);

    const iswv::canopen::Object<std::int32_t> target{{0x607A, 0}, "target"};
    auto written = node->write(target, 100000);
    CHECK(written);

    const auto frames = transport->sent_frames();
    CHECK_EQ(frames.size(), 2U);
    CHECK_EQ(frames[0].id, 0x601U);
    CHECK_EQ(frames[0].data[0], 0x40U);
    CHECK_EQ(frames[1].data[0], 0x23U);
    CHECK_EQ(frames[1].data[4], 0xA0U);
    CHECK_EQ(frames[1].data[5], 0x86U);
    CHECK_EQ(frames[1].data[6], 0x01U);
}

TEST_CASE("SDO abort and timeout are surfaced as Result errors")
{
    auto transport = std::make_shared<iswv::FakeTransport>();
    iswv::canopen::CanopenMaster master(transport);
    auto node = master.node(2);

    transport->set_send_hook([transport](const iswv::CanFrame& request) {
        if (iswv::test::request_object(request).index == 0x9999) {
            iswv::CanFrame response;
            response.id = 0x582;
            response.size = 8;
            response.data[0] = 0x80;
            response.data[1] = request.data[1];
            response.data[2] = request.data[2];
            response.data[3] = request.data[3];
            response.data[6] = 0x02;
            response.data[7] = 0x06;  // 0x06020000
            transport->inject(response);
        }
    });

    auto aborted = node->upload_blocking({0x9999, 0});
    CHECK(!aborted);
    CHECK_EQ(aborted.error().code, iswv::ErrorCode::sdo_abort);
    CHECK(aborted.error().sdo_abort_code.has_value());
    CHECK_EQ(*aborted.error().sdo_abort_code, 0x06020000U);

    auto timed_out = node->upload_blocking({0x1234, 0}, {20ms});
    CHECK(!timed_out);
    CHECK_EQ(timed_out.error().code, iswv::ErrorCode::timeout);
}

TEST_CASE("SDO requests serialize per node and run concurrently across nodes")
{
    auto transport = std::make_shared<iswv::FakeTransport>();
    iswv::canopen::CanopenMaster master(transport);
    auto node1 = master.node(1);
    auto node2 = master.node(2);

    auto first = node1->upload({0x2000, 0}, {200ms});
    auto second = node1->upload({0x2001, 0}, {200ms});
    auto other = node2->upload({0x2002, 0}, {200ms});

    std::this_thread::sleep_for(5ms);
    auto frames = transport->sent_frames();
    CHECK_EQ(frames.size(), 2U);
    CHECK_EQ(frames[0].id, 0x601U);
    CHECK_EQ(frames[1].id, 0x602U);

    transport->inject(iswv::test::sdo_upload_response(frames[0], std::uint8_t{1}));
    transport->inject(iswv::test::sdo_upload_response(frames[1], std::uint8_t{2}));
    CHECK(first.get());
    CHECK(other.get());

    std::this_thread::sleep_for(5ms);
    frames = transport->sent_frames();
    CHECK_EQ(frames.size(), 3U);
    transport->inject(iswv::test::sdo_upload_response(frames[2], std::uint8_t{3}));
    CHECK(second.get());
}

TEST_CASE("NMT, heartbeat timeout, and periodic SYNC work")
{
    auto transport = std::make_shared<iswv::FakeTransport>();
    iswv::canopen::CanopenMaster master(transport);
    auto node = master.node(3);

    std::promise<void> timeout_promise;
    auto timeout_future = timeout_promise.get_future();
    auto subscription = node->on_timeout([&timeout_promise](const auto&) {
        timeout_promise.set_value();
    });
    node->set_local_heartbeat_timeout(15ms);

    iswv::CanFrame boot;
    boot.id = 0x703;
    boot.size = 1;
    boot.data[0] = 0;
    transport->inject(boot);
    CHECK(timeout_future.wait_for(100ms) == std::future_status::ready);
    CHECK(!node->network_state().online);

    CHECK(master.send_nmt(iswv::canopen::NmtCommand::start, 3));
    CHECK(master.start_periodic_sync(2ms));
    std::this_thread::sleep_for(12ms);
    master.stop_periodic_sync();
    const auto statistics = master.sync_statistics();
    CHECK(statistics.frames_sent >= 3U);
}

TEST_CASE("SDO queue limit rejects excess work without disturbing active request")
{
    auto transport = std::make_shared<iswv::FakeTransport>();
    iswv::canopen::MasterOptions options;
    options.sdo_queue_limit_per_node = 1;
    iswv::canopen::CanopenMaster master(transport, options);
    auto node = master.node(4);

    auto active = node->upload({0x2000, 0}, {200ms});
    auto rejected = node->upload({0x2001, 0}, {200ms});
    CHECK(rejected.wait_for(100ms) == std::future_status::ready);
    const auto rejected_result = rejected.get();
    CHECK(!rejected_result);
    CHECK_EQ(rejected_result.error().code, iswv::ErrorCode::queue_overflow);

    const auto frames = transport->sent_frames();
    CHECK_EQ(frames.size(), 1U);
    transport->inject(iswv::test::sdo_upload_response(frames.front(), std::uint8_t{7}));
    const auto active_result = active.get();
    CHECK(active_result);
    CHECK_EQ(active_result.value().data[0], 7U);
}

TEST_CASE("Node supervision times out before the first heartbeat")
{
    auto transport = std::make_shared<iswv::FakeTransport>();
    iswv::canopen::CanopenMaster master(transport);
    auto node = master.node(5);

    std::promise<iswv::canopen::NodeTimeoutEvent> timeout_promise;
    auto timeout_future = timeout_promise.get_future();
    auto subscription = node->on_timeout(
        [&timeout_promise](const iswv::canopen::NodeTimeoutEvent& event) {
            timeout_promise.set_value(event);
        });
    node->set_local_heartbeat_timeout(15ms);

    CHECK(timeout_future.wait_for(150ms) == std::future_status::ready);
    const auto event = timeout_future.get();
    CHECK_EQ(event.node_id, 5U);
    CHECK_EQ(event.timeout, 15ms);
    CHECK(!node->network_state().online);
}

TEST_CASE("Node Guard responses are classified and failed requests leave no stale marker")
{
    auto transport = std::make_shared<iswv::FakeTransport>();
    iswv::canopen::CanopenMaster master(transport);
    auto node = master.node(6);

    std::promise<iswv::canopen::HeartbeatEvent> first_promise;
    auto first_future = first_promise.get_future();
    auto first_subscription = node->on_heartbeat(
        [&first_promise](const iswv::canopen::HeartbeatEvent& event) {
            first_promise.set_value(event);
        });
    transport->set_send_hook([transport](const iswv::CanFrame& request) {
        if (request.id == 0x706U && request.remote) {
            iswv::CanFrame response;
            response.id = 0x706U;
            response.size = 1;
            response.data[0] = 0x05U;
            transport->inject(response);
        }
    });

    CHECK(master.request_node_guard(6));
    CHECK(first_future.wait_for(100ms) == std::future_status::ready);
    const auto first = first_future.get();
    CHECK(first.node_guard);
    CHECK_EQ(first.state, iswv::canopen::NmtState::operational);

    first_subscription.reset();
    transport->set_send_hook({});
    transport->close();
    const auto failed = master.request_node_guard(6);
    CHECK(!failed);
    CHECK_EQ(failed.error().code, iswv::ErrorCode::transport_closed);
    transport->open();

    std::promise<iswv::canopen::HeartbeatEvent> second_promise;
    auto second_future = second_promise.get_future();
    auto second_subscription = node->on_heartbeat(
        [&second_promise](const iswv::canopen::HeartbeatEvent& event) {
            second_promise.set_value(event);
        });
    iswv::CanFrame heartbeat;
    heartbeat.id = 0x706U;
    heartbeat.size = 1;
    heartbeat.data[0] = 0x05U;
    transport->inject(heartbeat);

    CHECK(second_future.wait_for(100ms) == std::future_status::ready);
    CHECK(!second_future.get().node_guard);
}

TEST_CASE("periodic Node Guard starts promptly and stop clears local supervision")
{
    auto transport = std::make_shared<iswv::FakeTransport>();
    iswv::canopen::CanopenMaster master(transport);
    auto node = master.node(11);

    std::atomic<unsigned> timeout_count{0};
    auto timeout_subscription = node->on_timeout(
        [&timeout_count](const iswv::canopen::NodeTimeoutEvent&) {
            ++timeout_count;
        });
    CHECK(node->configure_node_guard(10ms, 2, false));

    iswv::CanFrame request;
    CHECK(transport->wait_for_sent(request, 100ms));
    CHECK_EQ(request.id, 0x70BU);
    CHECK(request.remote);

    node->stop_node_guard();
    std::promise<iswv::canopen::HeartbeatEvent> heartbeat_promise;
    auto heartbeat_future = heartbeat_promise.get_future();
    auto heartbeat_subscription = node->on_heartbeat(
        [&heartbeat_promise](const iswv::canopen::HeartbeatEvent& event) {
            heartbeat_promise.set_value(event);
        });
    iswv::CanFrame heartbeat;
    heartbeat.id = 0x70BU;
    heartbeat.size = 1;
    heartbeat.data[0] = 0x05U;
    transport->inject(heartbeat);

    CHECK(heartbeat_future.wait_for(100ms) == std::future_status::ready);
    CHECK(!heartbeat_future.get().node_guard);
    std::this_thread::sleep_for(30ms);
    CHECK_EQ(timeout_count.load(), 0U);
}

TEST_CASE("blocking SDO from a dispatched callback returns would_deadlock")
{
    auto transport = std::make_shared<iswv::FakeTransport>();
    iswv::canopen::CanopenMaster master(transport);
    auto node = master.node(7);

    std::promise<iswv::Result<iswv::canopen::SdoResponse>> result_promise;
    auto result_future = result_promise.get_future();
    auto subscription = master.on_raw_frame(
        [&result_promise, node](const iswv::CanFrame&) {
            result_promise.set_value(node->upload_blocking({0x2000, 0}));
        });
    iswv::CanFrame frame;
    frame.id = 0x123U;
    frame.size = 1;
    transport->inject(frame);

    CHECK(result_future.wait_for(100ms) == std::future_status::ready);
    const auto result = result_future.get();
    CHECK(!result);
    CHECK_EQ(result.error().code, iswv::ErrorCode::would_deadlock);
    CHECK(transport->sent_frames().empty());
}

TEST_CASE("master can be destroyed from its callback thread")
{
    auto transport = std::make_shared<iswv::FakeTransport>();
    auto master = std::make_unique<iswv::canopen::CanopenMaster>(transport);
    auto node = master->node(8);

    std::promise<void> destroyed_promise;
    auto destroyed_future = destroyed_promise.get_future();
    auto subscription = master->on_raw_frame(
        [&master, &destroyed_promise](const iswv::CanFrame&) {
            master.reset();
            destroyed_promise.set_value();
        });
    iswv::CanFrame frame;
    frame.id = 0x124U;
    frame.size = 1;
    transport->inject(frame);

    CHECK(destroyed_future.wait_for(200ms) == std::future_status::ready);
    const auto after_destruction = node->send_nmt(iswv::canopen::NmtCommand::start);
    CHECK(!after_destruction);
    CHECK_EQ(after_destruction.error().code, iswv::ErrorCode::transport_closed);
}

TEST_CASE("Linux error frames retain their error mask and bypass CANopen parsing")
{
    auto transport = std::make_shared<iswv::FakeTransport>();
    iswv::canopen::CanopenMaster master(transport);
    auto node = master.node(5);

    std::atomic<unsigned> heartbeat_count{0};
    auto heartbeat_subscription = node->on_heartbeat(
        [&heartbeat_count](const iswv::canopen::HeartbeatEvent&) {
            ++heartbeat_count;
        });
    std::promise<iswv::CanFrame> raw_promise;
    auto raw_future = raw_promise.get_future();
    auto raw_subscription = master.on_raw_frame(
        [&raw_promise](const iswv::CanFrame& frame) {
            raw_promise.set_value(frame);
        });

    iswv::CanFrame error_frame;
    error_frame.id = 0x00000705U;
    error_frame.error = true;
    error_frame.size = 8;
    error_frame.data[0] = 0x05U;
    CHECK(iswv::valid_frame(error_frame));
    transport->inject(error_frame);

    CHECK(raw_future.wait_for(100ms) == std::future_status::ready);
    const auto received = raw_future.get();
    CHECK(received.error);
    CHECK_EQ(received.id, 0x00000705U);
    CHECK_EQ(heartbeat_count.load(), 0U);
}

TEST_CASE("transport send exceptions are converted to Result errors")
{
    auto transport = std::make_shared<iswv::FakeTransport>();
    iswv::canopen::CanopenMaster master(transport);
    transport->set_send_hook([](const iswv::CanFrame&) {
        throw std::runtime_error("simulated adapter failure");
    });

    const auto sent = master.send_nmt(iswv::canopen::NmtCommand::start, 1);
    CHECK(!sent);
    CHECK_EQ(sent.error().code, iswv::ErrorCode::transport_error);
    CHECK_EQ(sent.error().message, std::string("simulated adapter failure"));
}

TEST_CASE("concurrent shutdown completes an in-flight SDO exactly once")
{
    auto transport = std::make_shared<iswv::FakeTransport>();
    auto master = std::make_unique<iswv::canopen::CanopenMaster>(transport);
    auto node = master->node(10);

    std::mutex mutex;
    std::condition_variable condition;
    bool send_entered = false;
    bool release_send = false;
    transport->set_send_hook([&](const iswv::CanFrame&) {
        std::unique_lock<std::mutex> lock(mutex);
        send_entered = true;
        condition.notify_all();
        condition.wait(lock, [&] { return release_send; });
        throw std::runtime_error("send failed after shutdown");
    });

    std::atomic<unsigned> completion_count{0};
    std::thread caller([&] {
        node->upload_async({0x2000, 0}, {200ms},
                           [&completion_count](const auto&) {
                               ++completion_count;
                           });
    });
    {
        std::unique_lock<std::mutex> lock(mutex);
        CHECK(condition.wait_for(lock, 100ms, [&] { return send_entered; }));
    }

    master.reset();
    CHECK_EQ(completion_count.load(), 1U);
    {
        std::lock_guard<std::mutex> lock(mutex);
        release_send = true;
    }
    condition.notify_all();
    caller.join();
    CHECK_EQ(completion_count.load(), 1U);
}
