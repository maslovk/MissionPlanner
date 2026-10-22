#include <cstring>
#include <functional>
#include <vector>

#include "mission_download.hpp"
#include "protocol.hpp"
#include "test.hpp"

using namespace mpcore;

namespace {

constexpr int64_t kMs = 1000;
constexpr uint32_t kRequestList = 43, kCount = 44, kRequest = 40, kRequestInt = 51, kItem = 39,
                   kItemInt = 73, kAck = 47;

std::vector<uint8_t> frame(uint32_t msgid, const std::vector<uint8_t> &payload, uint32_t sysid = 1) {
    EncodeRequest req;
    req.msgid = msgid;
    req.payload = payload.data();
    req.payload_len = payload.size();
    req.src_sysid = sysid;
    req.src_compid = 1;
    std::vector<uint8_t> out(kMaxFrameLen);
    const int n = encode(req, out.data(), out.size());
    CHECK(n > 0);
    out.resize(n > 0 ? static_cast<std::size_t>(n) : 0);
    return out;
}

// A vehicle with a list of items that answers the download's requests.
struct Vehicle {
    uint16_t count;
    uint8_t type = 0;
    uint8_t gcs_sysid = 255, gcs_compid = 190;
    int drop_counts = 0;       // MISSION_COUNT replies to drop first
    int drop_items = 0;        // item replies to drop first
    bool wrong_seq_once = false;
    bool silent = false;
    std::vector<std::vector<uint8_t>> outbox;
    std::vector<uint32_t> got;  // msgids received
    std::vector<uint16_t> asked;
    std::vector<uint8_t> ack;   // the MISSION_ACK payload

    explicit Vehicle(uint16_t n) : count(n) {}

    void reply(std::vector<uint8_t> f, int &drop) {
        if (silent)
            return;
        if (drop > 0) {
            drop--;
            return;
        }
        outbox.push_back(std::move(f));
    }

    std::vector<uint8_t> count_frame(uint16_t n, uint8_t mission_type, uint8_t to_sysid) const {
        std::vector<uint8_t> p(5, 0);  // count, target, mission type (this dialect has no opaque_id)
        std::memcpy(p.data(), &n, 2);
        p[2] = to_sysid;
        p[3] = gcs_compid;
        p[4] = mission_type;
        return frame(kCount, p);
    }

    std::vector<uint8_t> item_frame(uint32_t msgid, uint16_t seq) const {
        std::vector<uint8_t> p(38, 0);
        const float param1 = seq + 0.5f;
        std::memcpy(p.data(), &param1, 4);
        const int32_t x = 100000 * seq, y = -200000 * seq;
        std::memcpy(p.data() + 16, &x, 4);
        std::memcpy(p.data() + 20, &y, 4);
        std::memcpy(p.data() + 28, &seq, 2);
        const uint16_t command = 16;
        std::memcpy(p.data() + 30, &command, 2);
        p[32] = gcs_sysid;
        p[33] = gcs_compid;
        p[37] = type;
        return frame(msgid, p);
    }

    void receive(uint32_t msgid, const uint8_t *payload, std::size_t len) {
        got.push_back(msgid);
        if (msgid == kRequestList) {
            reply(count_frame(count, type, gcs_sysid), drop_counts);
        } else if ((msgid == kRequestInt || msgid == kRequest) && len >= 5) {
            uint16_t seq;
            std::memcpy(&seq, payload, 2);
            asked.push_back(seq);
            uint16_t answer = seq;
            if (wrong_seq_once) {
                wrong_seq_once = false;
                answer = static_cast<uint16_t>(seq + 1);
            }
            reply(item_frame(msgid == kRequestInt ? kItemInt : kItem, answer), drop_items);
        } else if (msgid == kAck) {
            ack.assign(payload, payload + len);
        }
    }

    std::size_t sent(uint32_t msgid) const {
        std::size_t n = 0;
        for (uint32_t m : got)
            n += m == msgid ? 1 : 0;
        return n;
    }
};

struct Run {
    Vehicle vehicle;
    MissionDownload download;
    int64_t now = 0;

    Run(uint16_t n, bool use_int = true, uint8_t type = 0)
        : vehicle(n),
          download(1, 1, 255, 190, type, use_int,
                   [this](uint32_t msgid, const uint8_t *p, std::size_t len) { vehicle.receive(msgid, p, len); }) {
        vehicle.type = type;
    }

    void run_for(int64_t us) {
        const int64_t end = now + us;
        while (now < end) {
            auto frames = std::move(vehicle.outbox);
            vehicle.outbox.clear();
            for (const auto &f : frames)
                download.on_frame(f.data(), f.size(), now);
            const auto s = download.tick(now);
            if (s == MissionDownload::State::Done || s == MissionDownload::State::Failed)
                return;
            now += 10 * kMs;
        }
    }
};

}  // namespace

TEST(mission_download_clean) {
    Run r(12);
    r.download.start(0);
    r.run_for(5000 * kMs);
    CHECK(r.download.state() == MissionDownload::State::Done);
    CHECK_EQ(r.download.count(), 12);
    CHECK_EQ(r.download.items().size(), std::size_t{12});
    CHECK_EQ(r.vehicle.sent(kRequestList), std::size_t{1});
    CHECK_EQ(r.vehicle.sent(kRequestInt), std::size_t{12});
    CHECK_EQ(r.vehicle.sent(kRequest), std::size_t{0});
    // the ACK closes it: accepted, for the mission type
    CHECK_EQ(r.vehicle.ack.size(), std::size_t{4});
    if (r.vehicle.ack.size() == 4) {
        CHECK_EQ(r.vehicle.ack[2], 0);
        CHECK_EQ(r.vehicle.ack[3], 0);
    }
    // the items are the frames received
    FrameView f;
    CHECK(view_frame(r.download.items()[5].data(), r.download.items()[5].size(), f));
    CHECK_EQ(f.msgid, kItemInt);
    uint16_t seq;
    std::memcpy(&seq, f.payload + 28, 2);
    CHECK_EQ(seq, 5);
}

TEST(mission_download_without_mission_int) {
    Run r(3, false);
    r.download.start(0);
    r.run_for(5000 * kMs);
    CHECK(r.download.state() == MissionDownload::State::Done);
    CHECK_EQ(r.vehicle.sent(kRequest), std::size_t{3});
    CHECK_EQ(r.vehicle.sent(kRequestInt), std::size_t{0});
}

TEST(mission_download_empty_list) {
    Run r(0);
    r.download.start(0);
    r.run_for(1000 * kMs);
    CHECK(r.download.state() == MissionDownload::State::Done);
    CHECK_EQ(r.vehicle.sent(kAck), std::size_t{1});
}

TEST(mission_download_resends_on_loss) {
    Run r(4);
    r.vehicle.drop_counts = 1;
    r.vehicle.drop_items = 1;
    r.download.start(0);
    r.run_for(699 * kMs);
    CHECK_EQ(r.vehicle.sent(kRequestList), std::size_t{1});
    r.run_for(20 * kMs);
    CHECK_EQ(r.vehicle.sent(kRequestList), std::size_t{2});  // after 700 ms
    r.run_for(2400 * kMs);
    CHECK_EQ(r.vehicle.sent(kRequestInt), std::size_t{1});  // the lost item: 2.5 s
    r.run_for(5000 * kMs);
    CHECK(r.download.state() == MissionDownload::State::Done);
    CHECK_EQ(r.vehicle.sent(kRequestInt), std::size_t{5});
}

TEST(mission_download_wrong_seq_asks_again_at_once) {
    Run r(3);
    r.vehicle.wrong_seq_once = true;
    r.download.start(0);
    r.run_for(100 * kMs);
    CHECK(r.download.state() == MissionDownload::State::Done);
    CHECK_EQ(r.vehicle.asked.size(), std::size_t{4});
    if (r.vehicle.asked.size() == 4) {
        CHECK_EQ(r.vehicle.asked[0], 0);
        CHECK_EQ(r.vehicle.asked[1], 0);  // asked again, not waiting for the timeout
        CHECK_EQ(r.vehicle.asked[2], 1);
    }
}

TEST(mission_download_ignores_other_gcs_and_types) {
    Run r(2, true, 1);  // a fence
    r.download.start(0);
    // a count for another ground station, and one for the mission rather than the fence
    const auto other_gcs = r.vehicle.count_frame(9, 1, 254);
    const auto other_type = r.vehicle.count_frame(9, 0, 255);
    r.download.on_frame(other_gcs.data(), other_gcs.size(), 0);
    r.download.on_frame(other_type.data(), other_type.size(), 0);
    CHECK(r.download.state() == MissionDownload::State::Counting);
    r.run_for(5000 * kMs);
    CHECK(r.download.state() == MissionDownload::State::Done);
    CHECK_EQ(r.download.count(), 2);
    CHECK_EQ(r.vehicle.ack.size() == 4 ? r.vehicle.ack[3] : 255, 1);
}

TEST(mission_download_gives_up) {
    Run r(5);
    r.vehicle.silent = true;
    r.download.start(0);
    r.run_for(10'000 * kMs);
    CHECK(r.download.state() == MissionDownload::State::Failed);
    CHECK(r.download.failed_counting());
    CHECK_EQ(r.vehicle.sent(kRequestList), std::size_t{7});  // 1 + 6 resends, as getWPCountAsync
    CHECK(r.now >= 4900 * kMs && r.now < 5000 * kMs);
}
