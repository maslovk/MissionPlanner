#include <cstring>
#include <string>
#include <vector>

#include "exchange.hpp"
#include "protocol.hpp"
#include "test.hpp"

using namespace mpcore;

namespace {

constexpr int64_t kMs = 1000;

std::vector<uint8_t> param_value(uint32_t sysid, const std::string &name, float value, uint16_t index) {
    uint8_t p[25] = {};
    std::memcpy(p, &value, 4);
    const uint16_t count = 300;
    std::memcpy(p + 4, &count, 2);
    std::memcpy(p + 6, &index, 2);
    std::memcpy(p + 8, name.data(), name.size() < 16 ? name.size() : 16);
    p[24] = 9;
    EncodeRequest req;
    req.msgid = 22;
    req.payload = p;
    req.payload_len = sizeof(p);
    req.src_sysid = sysid;
    req.src_compid = 1;
    std::vector<uint8_t> out(kMaxFrameLen);
    out.resize(static_cast<std::size_t>(encode(req, out.data(), out.size())));
    return out;
}

struct Sent {
    std::vector<uint32_t> msgids;
    std::vector<std::vector<uint8_t>> payloads;
    Exchange::SendFn fn() {
        return [this](uint32_t msgid, const uint8_t *p, std::size_t n) {
            msgids.push_back(msgid);
            payloads.emplace_back(p, p + n);
        };
    }
};

}  // namespace

TEST(exchange_param_set_payload) {
    Sent sent;
    Exchange e(Exchange::param_set(0x80000001u, 1, "WPNAV_SPEED", 750.0f, 9), sent.fn());
    e.start(0);
    CHECK_EQ(sent.msgids.size(), std::size_t{1});
    CHECK_EQ(sent.msgids[0], 23u);  // PARAM_SET
    const auto &p = sent.payloads[0];
    CHECK_EQ(p.size(), std::size_t{23});
    float value;
    std::memcpy(&value, p.data(), 4);
    CHECK_EQ(value, 750.0f);
    CHECK_EQ(p[4], 255);  // a 32-bit target: the sentinel
    CHECK_EQ(p[5], 1);
    CHECK_EQ(std::string(reinterpret_cast<const char *>(p.data() + 6)), std::string("WPNAV_SPEED"));
    CHECK_EQ(p[22], 9);
}

TEST(exchange_param_set_waits_for_its_name) {
    Sent sent;
    Exchange e(Exchange::param_set(1, 1, "WPNAV_SPEED", 750.0f, 9), sent.fn());
    e.start(0);
    const auto other = param_value(1, "WPNAV_ACCEL", 1, 3);
    const auto foreign = param_value(2, "WPNAV_SPEED", 750, 4);
    const auto answer = param_value(1, "WPNAV_SPEED", 750, 4);
    e.on_frame(other.data(), other.size(), 10 * kMs);
    e.on_frame(foreign.data(), foreign.size(), 10 * kMs);
    CHECK(e.tick(20 * kMs) == Exchange::State::Waiting);
    e.on_frame(answer.data(), answer.size(), 30 * kMs);
    CHECK(e.tick(40 * kMs) == Exchange::State::Done);
    CHECK(e.reply() == answer);
    CHECK_EQ(e.sends(), 1u);
}

TEST(exchange_resends_then_times_out) {
    Sent sent;
    Exchange e(Exchange::param_set(1, 1, "X", 1, 9), sent.fn());
    e.start(0);
    // as Mission Planner: 700 ms per try, 3 resends, then a timeout
    CHECK(e.tick(699 * kMs) == Exchange::State::Waiting);
    CHECK_EQ(e.sends(), 1u);
    e.tick(700 * kMs);
    e.tick(1400 * kMs);
    e.tick(2100 * kMs);
    CHECK_EQ(e.sends(), 4u);
    CHECK(e.tick(2799 * kMs) == Exchange::State::Waiting);
    CHECK(e.tick(2800 * kMs) == Exchange::State::TimedOut);
    // a late answer changes nothing
    const auto late = param_value(1, "X", 1, 0);
    e.on_frame(late.data(), late.size(), 2900 * kMs);
    CHECK(e.state() == Exchange::State::TimedOut);
}

TEST(exchange_full_length_name_matches) {
    Sent sent;
    Exchange e(Exchange::param_set(1, 1, "ABCDEFGHIJKLMNOP", 1, 9), sent.fn());
    e.start(0);
    const auto shorter = param_value(1, "ABCDEFGHIJKLMNO", 1, 0);
    const auto exact = param_value(1, "ABCDEFGHIJKLMNOP", 1, 1);
    e.on_frame(shorter.data(), shorter.size(), 0);
    CHECK(e.state() == Exchange::State::Waiting);
    e.on_frame(exact.data(), exact.size(), 0);
    CHECK(e.state() == Exchange::State::Done);
}

TEST(exchange_param_read_by_index_and_by_name) {
    Sent sent;
    Exchange by_index(Exchange::param_read(1, 1, "", 42), sent.fn());
    by_index.start(0);
    CHECK_EQ(sent.msgids[0], 20u);  // PARAM_REQUEST_READ
    int16_t index;
    std::memcpy(&index, sent.payloads[0].data(), 2);
    CHECK_EQ(index, 42);
    const auto wrong = param_value(1, "OTHER", 0, 41);
    const auto right = param_value(1, "WHATEVER", 0, 42);
    by_index.on_frame(wrong.data(), wrong.size(), 0);
    CHECK(by_index.state() == Exchange::State::Waiting);
    by_index.on_frame(right.data(), right.size(), 0);
    CHECK(by_index.state() == Exchange::State::Done);

    Exchange by_name(Exchange::param_read(1, 1, "RTL_ALT", -1), sent.fn());
    by_name.start(0);
    std::memcpy(&index, sent.payloads[1].data(), 2);
    CHECK_EQ(index, -1);
    CHECK_EQ(std::string(reinterpret_cast<const char *>(sent.payloads[1].data() + 4)), std::string("RTL_ALT"));
    const auto rtl = param_value(1, "RTL_ALT", 1500, 7);
    by_name.on_frame(right.data(), right.size(), 0);
    CHECK(by_name.state() == Exchange::State::Waiting);
    by_name.on_frame(rtl.data(), rtl.size(), 0);
    CHECK(by_name.state() == Exchange::State::Done);
}
