#include <vector>

#include "link.hpp"
#include "test.hpp"
#include "wire.hpp"

using namespace mpcore;

namespace {

constexpr int64_t kMs = 1000;

// A link with its received frames and written bytes recorded.
struct Endpoint {
    explicit Endpoint(LinkConfig config) : link(config) {
        link.on_frame([this](const mp_frame &f) {
            frames.push_back(f);
            raw.emplace_back(f.frame, f.frame + f.frame_len);
        });
        link.on_write([this](const uint8_t *d, std::size_t n) { out.insert(out.end(), d, d + n); });
    }

    // moves what this endpoint wrote into another endpoint
    void deliver_to(Endpoint &other, int64_t now_us) {
        other.link.feed(out.data(), out.size(), now_us);
        out.clear();
    }

    Link link;
    std::vector<mp_frame> frames;  // pointers inside are stale after the callback; use raw
    std::vector<std::vector<uint8_t>> raw;
    std::vector<uint8_t> out;
};

LinkConfig cfg(uint32_t sysid, uint8_t compid) {
    LinkConfig c;
    c.gcs_sysid = sysid;
    c.gcs_compid = compid;
    return c;
}

int send_heartbeat(Endpoint &from, uint32_t to_sysid, uint8_t to_compid, uint32_t flags = 0) {
    return from.link.send(0, wire::kHeartbeatPayload.data(), wire::kHeartbeatPayload.size(), to_sysid,
                          to_compid, flags);
}

int send_command(Endpoint &from, uint32_t to_sysid, uint8_t to_compid) {
    const auto p = wire::command_payload(to_sysid, to_compid);
    return from.link.send(76, p.data(), p.size(), to_sysid, to_compid);
}

}  // namespace

TEST(link_answers_in_mavlink1_until_the_peer_sends_mavlink2) {
    Endpoint gcs(cfg(255, 190));
    Endpoint vehicle(cfg(1, 1));

    CHECK(send_heartbeat(vehicle, 255, 190) > 0);
    CHECK_EQ(vehicle.out[0], kStxV1);  // never heard the GCS: MAVLink 1
    vehicle.deliver_to(gcs, 0);
    CHECK_EQ(gcs.frames.size(), std::size_t{1});
    CHECK_EQ(gcs.link.peer_count(), std::size_t{1});

    send_heartbeat(gcs, 1, 1);
    CHECK_EQ(gcs.out[0], kStxV1);
    gcs.out.clear();

    send_heartbeat(vehicle, 255, 190, MP_SEND_FORCE_MAVLINK2);
    CHECK_EQ(vehicle.out[0], kStxV2);
    vehicle.deliver_to(gcs, 10 * kMs);
    CHECK(gcs.frames.back().flags & MP_FRAME_MAVLINK2);

    send_heartbeat(gcs, 1, 1);  // latched: MAVLink 2 from now on
    CHECK_EQ(gcs.out[0], kStxV2);
    gcs.out.clear();
    send_heartbeat(vehicle, 255, 190);  // the vehicle has still only heard MAVLink 1
    CHECK_EQ(vehicle.out[0], kStxV1);
    vehicle.out.clear();

    const Peer *p = gcs.link.find_peer(1, 1);
    CHECK(p != nullptr && p->mavlink2);
    CHECK_EQ(p->frames, uint64_t{2});
}

TEST(link_sequence_numbers_increase_per_link) {
    Endpoint gcs(cfg(255, 190));
    for (int i = 0; i < 300; i++)
        send_heartbeat(gcs, 1, 1, MP_SEND_FORCE_MAVLINK2);
    Endpoint vehicle(cfg(1, 1));
    gcs.deliver_to(vehicle, 0);
    CHECK_EQ(vehicle.frames.size(), std::size_t{300});
    CHECK_EQ(vehicle.raw[299][4], static_cast<uint8_t>(299 & 0xFF));
    const Peer *p = vehicle.link.find_peer(255, 190);
    CHECK(p != nullptr);
    if (p != nullptr) {
        CHECK_EQ(p->seq.packets_lost(), 0.0f);
        CHECK_EQ(p->seq.link_quality(), 100.0f);
    }
}

TEST(link_32bit_ids_and_target_filter) {
    const uint32_t wide = 0xffffff01u;
    Endpoint gcs(cfg(255, 190));
    Endpoint vehicle(cfg(wide, 1));
    Endpoint other(cfg(0xffffff02u, 1));

    send_heartbeat(vehicle, 255, 190);  // a 32-bit source is MAVLink 2 even to a new peer
    CHECK_EQ(vehicle.out[0], kStxV2);
    CHECK_EQ(vehicle.out[2] & kIflagSysid32, kIflagSysid32);
    vehicle.deliver_to(gcs, 0);
    CHECK_EQ(gcs.frames.size(), std::size_t{1});
    CHECK_EQ(gcs.frames[0].sysid, wide);
    CHECK(gcs.link.find_peer(wide, 1) != nullptr);
    CHECK(gcs.link.find_peer(wide & 0xFF, 1) == nullptr);  // no aliasing onto 8 bits

    CHECK(send_command(gcs, wide, 1) > 0);
    const std::vector<uint8_t> command = gcs.out;
    vehicle.link.feed(command.data(), command.size(), 0);
    other.link.feed(command.data(), command.size(), 0);

    CHECK_EQ(vehicle.frames.size(), std::size_t{1});
    if (!vehicle.frames.empty()) {
        const mp_frame &f = vehicle.frames[0];
        CHECK(f.flags & MP_FRAME_HAS_TARGET);
        CHECK_EQ(f.target_sysid, wide);
        CHECK_EQ(f.target_compid, 1);
        CHECK(!(f.flags & MP_FRAME_NOT_FOR_US));
    }
    // delivered for logging, flagged, and not counted as traffic from the GCS
    CHECK_EQ(other.frames.size(), std::size_t{1});
    if (!other.frames.empty())
        CHECK(other.frames[0].flags & MP_FRAME_NOT_FOR_US);
    const Peer *seen_by_other = other.link.find_peer(255, 190);
    CHECK(seen_by_other != nullptr && seen_by_other->frames == 0);
}

TEST(link_payload_target_without_target32) {
    Endpoint gcs(cfg(255, 190));
    Endpoint vehicle(cfg(1, 1));
    send_command(gcs, 1, 1);
    gcs.deliver_to(vehicle, 0);
    CHECK_EQ(vehicle.frames.size(), std::size_t{1});
    if (!vehicle.frames.empty()) {
        CHECK(vehicle.frames[0].flags & MP_FRAME_HAS_TARGET);
        CHECK_EQ(vehicle.frames[0].target_sysid, 1u);
        CHECK_EQ(vehicle.frames[0].target_compid, 1);
        // only a TARGET32 header is filtered, as in Mission Planner
        CHECK(!(vehicle.frames[0].flags & MP_FRAME_NOT_FOR_US));
    }
}

TEST(link_signing_with_key_ring) {
    const SigningKey key = wire::test_key(7);
    Endpoint gcs(cfg(255, 190));
    Endpoint vehicle(cfg(1, 1));
    vehicle.link.add_signing_key(wire::test_key(1));  // a wrong key first
    vehicle.link.add_signing_key(key);
    gcs.link.set_peer_signing(1, 1, &key, 3);

    send_heartbeat(gcs, 1, 1);
    CHECK_EQ(gcs.out[0], kStxV2);  // signing implies MAVLink 2
    CHECK_EQ(gcs.out[2] & kIflagSigned, kIflagSigned);
    gcs.deliver_to(vehicle, 0);
    CHECK_EQ(vehicle.frames.size(), std::size_t{1});
    if (!vehicle.frames.empty())
        CHECK(vehicle.frames[0].flags & MP_FRAME_SIGNATURE_OK);

    // the vehicle took the matching key for the GCS and now signs its replies
    const Peer *gcs_peer = vehicle.link.find_peer(255, 190);
    CHECK(gcs_peer != nullptr && gcs_peer->key && *gcs_peer->key == key);
    send_heartbeat(vehicle, 255, 190);
    CHECK_EQ(vehicle.out[2] & kIflagSigned, kIflagSigned);
    vehicle.deliver_to(gcs, 0);
    CHECK_EQ(gcs.frames.size(), std::size_t{1});
    if (!gcs.frames.empty())
        CHECK(gcs.frames[0].flags & MP_FRAME_SIGNATURE_OK);
}

TEST(link_drops_bad_signature_and_keeps_unsigned) {
    const SigningKey key = wire::test_key(7);
    Endpoint gcs(cfg(255, 190));
    Endpoint vehicle(cfg(1, 1));
    vehicle.link.add_signing_key(wire::test_key(8));
    gcs.link.set_peer_signing(1, 1, &key, 3);

    send_heartbeat(gcs, 1, 1);
    gcs.deliver_to(vehicle, 0);
    CHECK(vehicle.frames.empty());
    CHECK_EQ(vehicle.link.stats().signature_failures, uint64_t{1});

    // Unsigned frames are still accepted, as in Mission Planner (MAVLinkInterface only checks
    // signatures that are present).
    gcs.link.set_peer_signing(1, 1, nullptr, 0);
    send_heartbeat(gcs, 1, 1);
    gcs.deliver_to(vehicle, 10 * kMs);
    CHECK_EQ(vehicle.frames.size(), std::size_t{1});
}

TEST(link_ignore_signatures_for_playback) {
    LinkConfig c = cfg(255, 190);
    c.ignore_signatures = true;
    Endpoint player(c);
    const auto f = wire::frame(1, 1);  // signed with a key the player does not have
    player.link.feed(f.data(), f.size(), 0);
    CHECK_EQ(player.frames.size(), std::size_t{1});
    if (!player.frames.empty()) {
        CHECK(player.frames[0].flags & MP_FRAME_SIGNED);
        CHECK(!(player.frames[0].flags & MP_FRAME_SIGNATURE_OK));
    }
}

TEST(link_signing_timestamps_increase) {
    const SigningKey key = wire::test_key(7);
    Endpoint gcs(cfg(255, 190));
    gcs.link.set_wall_clock([] { return uint64_t{1420070400ull * 1000000ull + 1000}; });  // frozen
    gcs.link.set_peer_signing(1, 1, &key, 3);
    send_heartbeat(gcs, 1, 1);
    const std::vector<uint8_t> first = gcs.out;
    gcs.out.clear();
    send_heartbeat(gcs, 1, 1);
    const uint64_t t1 = signature_timestamp(first.data(), first.size());
    const uint64_t t2 = signature_timestamp(gcs.out.data(), gcs.out.size());
    CHECK_EQ(t1, uint64_t{100});  // 1000 us after the epoch, in 10 us units
    CHECK_EQ(t2, t1 + 1);
}

TEST(link_handler_may_reply_during_feed) {
    Endpoint gcs(cfg(255, 190));
    Endpoint vehicle(cfg(1, 1));
    int nested_feed = -1;
    gcs.link.on_frame([&](const mp_frame &f) {
        // e.g. a TIMESYNC reply sent from inside the receive callback
        send_heartbeat(gcs, f.sysid, f.compid);
        const uint8_t junk[] = {0xFD};
        nested_feed = static_cast<int>(gcs.link.feed(junk, 1, 0));
    });
    send_heartbeat(vehicle, 255, 190);
    vehicle.deliver_to(gcs, 0);
    CHECK(!gcs.out.empty());
    CHECK_EQ(nested_feed, 0);  // feed() from a handler is ignored
}

TEST(link_unknown_msgid_creates_no_peer) {
    Endpoint gcs(cfg(255, 190));
    const auto f = wire::frame(0, 5, 255, {1, 2, 3}, 0x123456, 99);
    gcs.link.feed(f.data(), f.size(), 0);
    CHECK_EQ(gcs.frames.size(), std::size_t{1});
    if (!gcs.frames.empty())
        CHECK(gcs.frames[0].flags & MP_FRAME_UNKNOWN_MSGID);
    CHECK_EQ(gcs.link.peer_count(), std::size_t{0});
}

TEST(link_stats) {
    Endpoint gcs(cfg(255, 190));
    Endpoint vehicle(cfg(1, 1));
    send_heartbeat(vehicle, 255, 190);
    send_heartbeat(vehicle, 255, 190, MP_SEND_FORCE_MAVLINK2);
    const std::size_t written = vehicle.out.size();
    vehicle.deliver_to(gcs, 0);
    const mp_link_stats s = gcs.link.stats();
    CHECK_EQ(s.bytes_in, uint64_t{written});
    CHECK_EQ(s.frames_in, uint64_t{2});
    CHECK_EQ(s.mavlink1_in, uint64_t{1});
    CHECK_EQ(s.mavlink2_in, uint64_t{1});
    CHECK_EQ(vehicle.link.stats().frames_out, uint64_t{2});
    CHECK_EQ(vehicle.link.stats().bytes_out, uint64_t{written});
}

TEST(link_poll_releases_frames_held_by_a_false_start) {
    Endpoint gcs(cfg(255, 190));
    // "FE FE": a MAVLink 1 start claiming 254 payload bytes; the heartbeat after it is read as
    // that payload and the link goes quiet
    const auto hb = wire::frame(0, 1);
    std::vector<uint8_t> stream = {0xFE, 0xFE};
    stream.insert(stream.end(), hb.begin(), hb.end());
    CHECK_EQ(gcs.link.feed(stream.data(), stream.size(), 0), std::size_t{0});
    CHECK_EQ(gcs.link.poll(Link::kStallUs), std::size_t{0});  // not stalled yet
    CHECK_EQ(gcs.link.poll(Link::kStallUs + 1), std::size_t{1});
    CHECK_EQ(gcs.frames.size(), std::size_t{1});
    if (!gcs.frames.empty()) {
        CHECK_EQ(gcs.frames[0].sysid, 1u);
        CHECK_EQ(gcs.frames[0].rx_time_us, int64_t{0});  // when its bytes came, not when released
    }
    CHECK_EQ(gcs.link.stats().parse_errors, uint64_t{1});
}

TEST(link_feed_after_a_stall_releases_first) {
    Endpoint gcs(cfg(255, 190));
    const auto hb1 = wire::frame(0, 1);
    const auto hb2 = wire::frame(0, 2);
    std::vector<uint8_t> stream = {0xFE, 0xFE};
    stream.insert(stream.end(), hb1.begin(), hb1.end());
    gcs.link.feed(stream.data(), stream.size(), 0);
    CHECK_EQ(gcs.link.feed(hb2.data(), hb2.size(), 2 * Link::kStallUs), std::size_t{2});
    CHECK_EQ(gcs.frames.size(), std::size_t{2});
    if (gcs.frames.size() == 2) {
        CHECK_EQ(gcs.frames[0].sysid, 1u);
        CHECK_EQ(gcs.frames[1].sysid, 2u);
    }
}
