#include <cstring>
#include <functional>
#include <set>
#include <string>
#include <vector>

#include "param_download.hpp"
#include "protocol.hpp"
#include "test.hpp"

using namespace mpcore;

namespace {

constexpr int64_t kMs = 1000;
constexpr uint32_t kParamRequestRead = 20;
constexpr uint32_t kParamRequestList = 21;
constexpr uint32_t kParamValue = 22;

// A vehicle with a parameter table that answers the download's requests with PARAM_VALUE frames.
// deliver(index, attempt) decides whether a reply gets through.
struct Vehicle {
    uint32_t sysid = 1;
    uint8_t compid = 1;
    uint16_t count = 0;
    std::function<bool(uint16_t index, int attempt)> deliver = [](uint16_t, int) { return true; };
    std::vector<int> attempts;
    std::vector<std::vector<uint8_t>> outbox;  // frames to the GCS, delivered by the test
    std::vector<uint32_t> requests;            // msgids received
    std::vector<uint16_t> single;              // indexes asked for one by one
    std::vector<uint8_t> last_target;          // payload target system/component of the last request
    uint8_t seq = 0;

    explicit Vehicle(uint16_t n) : count(n), attempts(n, 0) {}

    std::vector<uint8_t> param_value(uint16_t index, uint16_t total) {
        uint8_t p[25] = {};
        const float value = static_cast<float>(index) * 1.5f;
        std::memcpy(p, &value, 4);
        std::memcpy(p + 4, &total, 2);
        std::memcpy(p + 6, &index, 2);
        const std::string name = index == 65535 ? "STANDALONE" : "P" + std::to_string(index);
        std::memcpy(p + 8, name.data(), name.size());
        p[24] = 9;  // REAL32
        EncodeRequest req;
        req.msgid = kParamValue;
        req.payload = p;
        req.payload_len = sizeof(p);
        req.src_sysid = sysid;
        req.src_compid = compid;
        req.seq = seq++;
        std::vector<uint8_t> out(kMaxFrameLen);
        out.resize(static_cast<std::size_t>(encode(req, out.data(), out.size())));
        return out;
    }

    void reply(uint16_t index) {
        if (deliver(index, attempts[index]++))
            outbox.push_back(param_value(index, count));
    }

    void receive(uint32_t msgid, const uint8_t *payload, std::size_t len) {
        requests.push_back(msgid);
        if (msgid == kParamRequestList && len >= 2) {
            last_target.assign(payload, payload + 2);
            for (uint16_t i = 0; i < count; i++)
                reply(i);
        } else if (msgid == kParamRequestRead && len >= 4) {
            last_target.assign(payload + 2, payload + 4);
            int16_t index;
            std::memcpy(&index, payload, 2);
            single.push_back(static_cast<uint16_t>(index));
            if (index >= 0 && index < count)
                reply(static_cast<uint16_t>(index));
        }
    }

    size_t full_lists() const {
        size_t n = 0;
        for (uint32_t m : requests)
            n += m == kParamRequestList ? 1 : 0;
        return n;
    }
};

struct Run {
    Vehicle vehicle;
    std::vector<ParamValue> got;
    std::vector<bool> duplicate;
    ParamDownload download;
    int64_t now = 0;

    explicit Run(uint16_t n, uint32_t sysid = 1)
        : vehicle(n),
          download(sysid, 1,
                   [this](uint32_t msgid, const uint8_t *p, std::size_t len) { vehicle.receive(msgid, p, len); },
                   [this](const ParamValue &p, bool dup) {
                       got.push_back(p);
                       duplicate.push_back(dup);
                   }) {
        vehicle.sysid = sysid;
    }

    // delivers what the vehicle sent, then advances time in 10 ms steps
    void run_for(int64_t us) {
        const int64_t end = now + us;
        while (now < end && !download.done()) {
            auto frames = std::move(vehicle.outbox);
            vehicle.outbox.clear();
            for (const auto &f : frames)
                download.on_frame(f.data(), f.size(), now);
            download.tick(now);
            now += 10 * kMs;
        }
    }
};

}  // namespace

TEST(params_clean_download_needs_one_request) {
    Run r(120);
    r.download.start(0);
    r.run_for(1000 * kMs);
    CHECK(r.download.done());
    CHECK_EQ(r.download.received(), 120u);
    CHECK_EQ(r.vehicle.full_lists(), std::size_t{1});
    CHECK(r.vehicle.single.empty());
    CHECK_EQ(r.got.size(), std::size_t{120});
    CHECK_EQ(r.got[7].name, std::string("P7"));
    CHECK_EQ(r.got[7].value, 7 * 1.5f);
    CHECK_EQ(r.got[7].type, 9);
    CHECK_EQ(r.got[7].count, 120);
}

TEST(params_heavy_loss_retries_the_list_then_one_by_one) {
    Run r(200);
    // only every other parameter gets through the lists (all three), single requests do
    r.vehicle.deliver = [](uint16_t index, int attempt) { return attempt >= 3 || index % 2 == 0; };
    r.download.start(0);
    r.run_for(60'000 * kMs);
    CHECK(r.download.done());
    CHECK_EQ(r.download.received(), 200u);
    // under 75 %: the whole list again, twice at most once the total is known
    CHECK_EQ(r.vehicle.full_lists(), std::size_t{3});
    CHECK(r.download.one_by_one());
    // the odd half, ten per second
    CHECK_EQ(r.vehicle.single.size(), std::size_t{100});
    std::set<uint16_t> asked(r.vehicle.single.begin(), r.vehicle.single.end());
    CHECK_EQ(asked.size(), std::size_t{100});
    for (uint16_t i : asked)
        CHECK(i % 2 == 1);
}

TEST(params_singles_come_ten_per_second) {
    Run r(100);
    // 80 % arrive with the list, the last index among them; single requests are answered
    r.vehicle.deliver = [](uint16_t index, int attempt) { return attempt > 0 || index % 5 != 0; };
    r.download.start(0);
    r.run_for(20 * kMs);
    // the last index came with gaps: 80 % is enough, so straight to single requests
    CHECK_EQ(r.vehicle.full_lists(), std::size_t{1});
    CHECK_EQ(r.vehicle.single.size(), std::size_t{10});
    CHECK(!r.download.done());
    r.run_for(500 * kMs);
    CHECK_EQ(r.vehicle.single.size(), std::size_t{10});  // the next round a second later
    r.run_for(1000 * kMs);
    CHECK_EQ(r.vehicle.single.size(), std::size_t{20});
    CHECK(r.download.done());
}

TEST(params_silent_vehicle_keeps_getting_the_list) {
    Run r(10);
    bool silent = true;
    r.vehicle.deliver = [&silent](uint16_t, int) { return !silent; };
    r.download.start(0);
    r.run_for(20'000 * kMs);
    // no total yet: list retries are not counted; at 0, 4, 8, 12 and 16 s
    CHECK_EQ(r.vehicle.full_lists(), std::size_t{5});
    CHECK(!r.download.one_by_one());
    silent = false;
    r.run_for(10'000 * kMs);
    CHECK(r.download.done());
}

TEST(params_gap_at_the_end_is_requested_at_once) {
    Run r(20);
    r.vehicle.deliver = [](uint16_t index, int attempt) { return attempt > 0 || index != 5; };
    r.download.start(0);
    r.run_for(50 * kMs);
    // index 19 arrived with 5 missing: 95 % so no list, and no 4 s wait
    CHECK(r.download.done());
    CHECK_EQ(r.vehicle.single.size(), std::size_t{1});
    if (!r.vehicle.single.empty())
        CHECK_EQ(r.vehicle.single[0], 5);
    CHECK(r.now < 1000 * kMs);
}

TEST(params_duplicates_and_standalone_and_other_vehicles) {
    Run r(3);
    r.download.start(0);
    const auto p0 = r.vehicle.param_value(0, 3);
    const auto standalone = r.vehicle.param_value(65535, 3);
    Vehicle other(3);
    other.sysid = 2;
    const auto foreign = other.param_value(1, 3);
    r.download.on_frame(p0.data(), p0.size(), 0);
    r.download.on_frame(p0.data(), p0.size(), 0);
    r.download.on_frame(standalone.data(), standalone.size(), 0);
    r.download.on_frame(foreign.data(), foreign.size(), 0);
    CHECK_EQ(r.got.size(), std::size_t{3});
    CHECK(!r.duplicate[0] && r.duplicate[1] && !r.duplicate[2]);
    CHECK_EQ(r.got[2].name, std::string("STANDALONE"));
    CHECK_EQ(r.download.received(), 1u);  // 65535 is not part of the list
    CHECK_EQ(r.download.total(), 3u);
    CHECK(!r.download.done());
}

TEST(params_no_parameters_is_done) {
    Run r(0);
    r.download.start(0);
    const auto none = r.vehicle.param_value(65535, 0);
    r.download.on_frame(none.data(), none.size(), 0);
    CHECK(r.download.done());
}

TEST(params_rounds_continue_after_the_last_index_requested) {
    Run r(100);
    // 70 of 100 arrive with every list; singles are not answered at first
    bool answer = false;
    r.vehicle.deliver = [&answer](uint16_t index, int attempt) { return index < 70 || (answer && attempt > 0); };
    r.download.start(0);
    r.run_for(12'100 * kMs);  // two list retries (70 % < 75 %), then the first round
    CHECK_EQ(r.vehicle.full_lists(), std::size_t{3});
    CHECK(r.download.one_by_one());
    CHECK_EQ(r.vehicle.single.size(), std::size_t{10});
    r.run_for(1000 * kMs);
    CHECK_EQ(r.vehicle.single.size(), std::size_t{20});
    if (r.vehicle.single.size() == 20) {
        CHECK_EQ(r.vehicle.single[0], 70);
        CHECK_EQ(r.vehicle.single[10], 80);  // after the last one asked for
    }
    answer = true;
    r.run_for(10'000 * kMs);
    CHECK(r.download.done());
}

TEST(params_wide_vehicle_payload_target_is_the_sentinel) {
    Run r(5, 0x80000001u);
    r.download.start(0);
    CHECK_EQ(r.vehicle.last_target.size(), std::size_t{2});
    if (r.vehicle.last_target.size() == 2) {
        CHECK_EQ(r.vehicle.last_target[0], 255);
        CHECK_EQ(r.vehicle.last_target[1], 1);
    }
    r.run_for(1000 * kMs);
    CHECK(r.download.done());  // its 32-bit sysid matched in the replies
}
