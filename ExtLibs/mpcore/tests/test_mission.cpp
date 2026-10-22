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

// ---- upload ----------------------------------------------------------------------------------

#include "mission_upload.hpp"

namespace {

// A vehicle receiving a list: it asks for each item after the previous one came, keeps what it
// got, and acknowledges the last.
struct Receiver {
    uint8_t type = 0;
    uint8_t gcs_sysid = 255, gcs_compid = 190;
    int lose_counts = 0;   // MISSION_COUNTs that never arrive
    int lose_item = -1;    // this item never arrives the first time
    bool ask_twice = false;  // asks for item 1 again after getting it
    uint8_t refuse = 0;    // a MAV_MISSION_RESULT to answer the count with, 0: accept
    bool silent = false;
    int expected = -1;
    std::vector<std::vector<uint8_t>> stored;
    std::vector<std::vector<uint8_t>> outbox;
    std::vector<uint16_t> asked;
    int counts = 0, items = 0;

    std::vector<uint8_t> request(uint16_t seq) {
        std::vector<uint8_t> p(5, 0);
        std::memcpy(p.data(), &seq, 2);
        p[2] = gcs_sysid;
        p[3] = gcs_compid;
        p[4] = type;
        asked.push_back(seq);
        return frame(kRequestInt, p);
    }

    std::vector<uint8_t> ack(uint8_t result) {
        return frame(kAck, {gcs_sysid, gcs_compid, result, type});
    }

    void receive(uint32_t msgid, const uint8_t *payload, std::size_t len) {
        if (silent)
            return;
        if (msgid == kCount) {
            counts++;
            if (lose_counts > 0) {
                lose_counts--;
                return;
            }
            uint16_t n;
            std::memcpy(&n, payload, 2);
            expected = n;
            stored.assign(n, {});
            if (refuse != 0)
                outbox.push_back(ack(refuse));
            else if (n == 0)
                outbox.push_back(ack(0));
            else
                outbox.push_back(request(0));
        } else if (msgid == kItemInt && len >= 30) {
            items++;
            uint16_t seq;
            std::memcpy(&seq, payload + 28, 2);
            if (static_cast<int>(seq) == lose_item) {
                lose_item = -1;
                return;
            }
            if (seq >= stored.size())
                return;
            stored[seq].assign(payload, payload + len);
            if (ask_twice && seq == 1) {
                ask_twice = false;
                outbox.push_back(request(1));
                return;
            }
            if (static_cast<int>(seq) + 1 < expected)
                outbox.push_back(request(static_cast<uint16_t>(seq + 1)));
            else
                outbox.push_back(ack(0));
        }
    }
};

std::vector<MissionUpload::Item> make_items(int n, uint8_t type = 0) {
    std::vector<MissionUpload::Item> items;
    for (int i = 0; i < n; i++) {
        std::vector<uint8_t> p(38, 0);
        const float param1 = static_cast<float>(i) * 2.0f;
        std::memcpy(p.data(), &param1, 4);
        const uint16_t seq = static_cast<uint16_t>(i);
        std::memcpy(p.data() + 28, &seq, 2);
        p[32] = 1;
        p[33] = 1;
        p[37] = type;
        items.push_back({kItemInt, p});
    }
    return items;
}

struct Upload {
    Receiver vehicle;
    std::vector<MissionUpload::Item> sent_items;
    MissionUpload upload;
    int64_t now = 0;

    explicit Upload(int n, uint8_t type = 0)
        : sent_items(make_items(n, type)),
          upload(1, 1, 255, 190, type, sent_items,
                 [this](uint32_t msgid, const uint8_t *p, std::size_t len) { vehicle.receive(msgid, p, len); }) {
        vehicle.type = type;
    }

    MissionUpload::State run_for(int64_t us) {
        const int64_t end = now + us;
        while (now < end) {
            auto frames = std::move(vehicle.outbox);
            vehicle.outbox.clear();
            for (const auto &f : frames)
                upload.on_frame(f.data(), f.size(), now);
            const auto s = upload.tick(now);
            if (s == MissionUpload::State::Done || s == MissionUpload::State::Failed)
                return s;
            now += 10 * kMs;
        }
        return upload.state();
    }

    bool stored_all() const {
        if (vehicle.stored.size() != sent_items.size())
            return false;
        for (std::size_t i = 0; i < sent_items.size(); i++)
            if (vehicle.stored[i] != sent_items[i].payload)
                return false;
        return true;
    }
};

}  // namespace

TEST(mission_upload_clean) {
    Upload u(8);
    u.upload.start(0);
    CHECK(u.run_for(5000 * kMs) == MissionUpload::State::Done);
    CHECK_EQ(u.upload.result(), 0);
    CHECK(u.stored_all());
    CHECK_EQ(u.vehicle.counts, 1);
    CHECK_EQ(u.upload.items_sent(), 8u);
    CHECK_EQ(u.upload.highest_requested(), 7);
}

TEST(mission_upload_lost_count_and_item) {
    Upload u(5);
    u.vehicle.lose_counts = 1;
    u.vehicle.lose_item = 2;
    u.upload.start(0);
    CHECK(u.run_for(10'000 * kMs) == MissionUpload::State::Done);
    CHECK_EQ(u.upload.result(), 0);
    CHECK(u.stored_all());
    CHECK_EQ(u.vehicle.counts, 2);  // the count again after 700 ms
    CHECK_EQ(u.upload.items_sent(), 6u);  // item 2 again after 450 ms
}

TEST(mission_upload_answers_what_is_asked) {
    Upload u(4);
    u.vehicle.ask_twice = true;
    u.upload.start(0);
    CHECK(u.run_for(5000 * kMs) == MissionUpload::State::Done);
    CHECK(u.stored_all());
    CHECK_EQ(u.upload.items_sent(), 5u);
}

TEST(mission_upload_refused) {
    Upload u(300);
    u.vehicle.refuse = 4;  // MAV_MISSION_NO_SPACE
    u.upload.start(0);
    CHECK(u.run_for(5000 * kMs) == MissionUpload::State::Done);
    CHECK_EQ(u.upload.result(), 4);
    CHECK_EQ(u.upload.items_sent(), 0u);
}

TEST(mission_upload_empty_list) {
    Upload u(0);
    u.upload.start(0);
    CHECK(u.run_for(5000 * kMs) == MissionUpload::State::Done);
    CHECK_EQ(u.upload.result(), 0);
}

TEST(mission_upload_ignores_other_types_and_gcs) {
    Upload u(2, 2);  // rally points
    u.upload.start(0);
    // a request for the mission, and one for another ground station: neither is ours
    Receiver other;
    other.type = 0;
    const auto wrong_type = other.request(0);
    other.type = 2;
    other.gcs_sysid = 254;
    const auto wrong_gcs = other.request(0);
    u.upload.on_frame(wrong_type.data(), wrong_type.size(), 0);
    u.upload.on_frame(wrong_gcs.data(), wrong_gcs.size(), 0);
    CHECK_EQ(u.upload.items_sent(), 0u);
    CHECK(u.run_for(5000 * kMs) == MissionUpload::State::Done);
    CHECK(u.stored_all());
}

TEST(mission_upload_gives_up) {
    Upload u(3);
    u.vehicle.silent = true;
    u.upload.start(0);
    CHECK(u.run_for(10'000 * kMs) == MissionUpload::State::Failed);
    CHECK(!u.upload.count_accepted());
    CHECK_EQ(u.upload.result(), -1);
    CHECK(u.now >= 2790 * kMs && u.now < 2900 * kMs);  // 4 counts, 700 ms each
}
