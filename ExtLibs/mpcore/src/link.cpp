#include "link.hpp"

#include <chrono>
#include <random>

namespace mpcore {

namespace {

// 2015-01-01T00:00:00Z, the MAVLink signing epoch
constexpr uint64_t kSigningEpochUnixUs = 1420070400ull * 1000000ull;

uint64_t system_clock_us() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<microseconds>(system_clock::now().time_since_epoch()).count());
}

uint8_t random_link_id() {
    // links on different threads are independent, so no shared generator
    thread_local std::mt19937 rng{std::random_device{}()};
    return static_cast<uint8_t>(rng() & 0xFF);
}

}  // namespace

Link::Link(LinkConfig config)
    : config_(config),
      parser_([this](const ParsedFrame &f) { handle(f); },
              [this](std::string_view line) {
                  if (on_text_)
                      on_text_(line);
              }),
      wall_clock_(system_clock_us) {}

std::size_t Link::feed(const uint8_t *data, std::size_t len, int64_t now_us) {
    if (feeding_ || (data == nullptr && len != 0))
        return 0;
    feeding_ = true;
    delivered_ = 0;
    release_stalled(now_us);
    now_us_ = now_us;
    parser_.feed(data, len);
    if (len != 0) {
        have_rx_ = true;
        last_rx_us_ = now_us;
    }
    feeding_ = false;
    return delivered_;
}

std::size_t Link::poll(int64_t now_us) {
    if (feeding_)
        return 0;
    feeding_ = true;
    delivered_ = 0;
    release_stalled(now_us);
    feeding_ = false;
    return delivered_;
}

void Link::release_stalled(int64_t now_us) {
    if (!have_rx_ || !parser_.in_frame() || now_us - last_rx_us_ <= kStallUs)
        return;
    // the frames found inside belong to the time their bytes came
    now_us_ = last_rx_us_;
    parser_.abandon_partial_frame();
}

Peer &Link::peer_for(uint32_t sysid, uint8_t compid, uint8_t first_seq) {
    auto it = peer_index_.find(key_of(sysid, compid));
    if (it != peer_index_.end())
        return *it->second;
    peers_.push_back(std::make_unique<Peer>(sysid, compid, first_seq));
    Peer *p = peers_.back().get();
    p->send_link_id = random_link_id();
    peer_index_.emplace(key_of(sysid, compid), p);
    return *p;
}

const Peer *Link::find_peer(uint32_t sysid, uint8_t compid) const {
    auto it = peer_index_.find(key_of(sysid, compid));
    return it == peer_index_.end() ? nullptr : it->second;
}

void Link::set_peer_signing(uint32_t sysid, uint8_t compid, const SigningKey *key, uint8_t link_id) {
    auto it = peer_index_.find(key_of(sysid, compid));
    // a peer configured before it is heard starts at seq 0, like Mission Planner's MAVList indexer
    Peer &peer = it != peer_index_.end() ? *it->second : peer_for(sysid, compid, 0);
    if (key != nullptr) {
        peer.key = *key;
        peer.send_link_id = link_id;
        peer.mavlink2 = true;  // signing needs MAVLink 2 (MAVLinkInterface.enableSigning)
    } else {
        peer.key.reset();
    }
}

bool Link::verify(Peer &peer, const ParsedFrame &parsed) {
    if (peer.key && signature_valid(parsed.raw, parsed.raw_len, *peer.key))
        return true;
    for (const auto &key : keys_) {
        if (signature_valid(parsed.raw, parsed.raw_len, key)) {
            // a key from the ring now belongs to this peer, and replies to it are signed
            peer.key = key;
            peer.mavlink2 = true;
            return true;
        }
    }
    return false;
}

void Link::handle(const ParsedFrame &parsed) {
    mp_frame f{};
    f.struct_size = sizeof(mp_frame);
    f.msgid = parsed.msgid;
    f.sysid = parsed.sysid;
    f.compid = parsed.compid;
    f.seq = parsed.seq;
    f.incompat_flags = parsed.incompat_flags;
    f.compat_flags = parsed.compat_flags;
    f.payload_len = parsed.payload_len;
    f.frame_len = parsed.raw_len;
    f.frame = parsed.raw;
    f.payload = parsed.raw + parsed.payload_offset;
    f.rx_time_us = now_us_;
    if (parsed.mavlink2)
        f.flags |= MP_FRAME_MAVLINK2;
    if (parsed.has_signature)
        f.flags |= MP_FRAME_SIGNED;

    if (!parsed.known_msgid) {
        // The CRC is unverified, so this is not evidence of a peer: no state, no statistics.
        f.flags |= MP_FRAME_UNKNOWN_MSGID;
        delivered_++;
        if (on_frame_)
            on_frame_(f);
        return;
    }

    const MsgInfo *info = find_msg(parsed.msgid);
    bool has_target_component = false;
    if (parsed.header_target) {
        f.flags |= MP_FRAME_HAS_TARGET;
        f.target_sysid = parsed.header_target_sysid;
    } else if (info->target_system_ofs != kNoField) {
        f.flags |= MP_FRAME_HAS_TARGET;
        // a zero-trimmed field is zero (broadcast)
        f.target_sysid = info->target_system_ofs < parsed.payload_len ? f.payload[info->target_system_ofs] : 0;
    }
    if (info->target_component_ofs != kNoField) {
        has_target_component = true;
        f.target_compid =
            info->target_component_ofs < parsed.payload_len ? f.payload[info->target_component_ofs] : 0;
    }

    // Mission Planner creates the vehicle state before checking the signature.
    Peer &peer = peer_for(parsed.sysid, parsed.compid, parsed.seq);
    if (parsed.mavlink2) {
        peer.mavlink2 = true;
        mavlink2_in_++;
    } else {
        mavlink1_in_++;
    }

    // Unsigned frames are accepted even from a signing peer, as Mission Planner does.
    if (parsed.has_signature && !config_.ignore_signatures) {
        signed_in_++;
        if (!verify(peer, parsed)) {
            signature_failures_++;
            return;
        }
        f.flags |= MP_FRAME_SIGNATURE_OK;
    }

    // Only a TARGET32 header can address another 32-bit system on a shared link; such frames
    // are passed on (for logging) but do not count towards that peer's statistics.
    if (parsed.header_target) {
        const bool system_ok = f.target_sysid == 0 || f.target_sysid == config_.gcs_sysid;
        const bool component_ok =
            !has_target_component || f.target_compid == 0 || f.target_compid == config_.gcs_compid;
        if (!system_ok || !component_ok)
            f.flags |= MP_FRAME_NOT_FOR_US;
    }

    if (!(f.flags & MP_FRAME_NOT_FOR_US)) {
        peer.seq.on_packet(parsed.seq, now_us_);
        peer.frames++;
        peer.last_rx_us = now_us_;
    }

    delivered_++;
    if (on_frame_)
        on_frame_(f);
}

uint64_t Link::next_signing_timestamp() {
    const uint64_t unix_us = wall_clock_ ? wall_clock_() : system_clock_us();
    uint64_t ts = unix_us > kSigningEpochUnixUs ? (unix_us - kSigningEpochUnixUs) / 10 : 0;
    // strictly increasing, or the receiver rejects the frame as a replay
    if (ts <= last_signing_timestamp_)
        ts = last_signing_timestamp_ + 1;
    last_signing_timestamp_ = ts;
    return ts;
}

int Link::send(uint32_t msgid, const uint8_t *payload, std::size_t payload_len, uint32_t target_sysid,
               uint8_t target_compid, uint32_t flags) {
    const Peer *peer = find_peer(target_sysid, target_compid);

    // Mission Planner (MAVLinkInterface.generatePacket) answers in MAVLink 1 until the peer has
    // sent MAVLink 2, unless the frame cannot be expressed in MAVLink 1.
    const bool mavlink2 = (flags & MP_SEND_FORCE_MAVLINK2) || (peer != nullptr && peer->mavlink2) ||
                          msgid > 255 || target_sysid > 255 || config_.gcs_sysid > 255;

    EncodeRequest req;
    req.msgid = msgid;
    req.payload = payload;
    req.payload_len = payload_len;
    req.src_sysid = config_.gcs_sysid;
    req.src_compid = config_.gcs_compid;
    req.target_sysid = target_sysid;
    req.seq = tx_seq_;
    req.mavlink1 = !mavlink2;
    if (mavlink2 && peer != nullptr && peer->key) {
        req.key = &*peer->key;
        req.link_id = peer->send_link_id;
        req.timestamp = next_signing_timestamp();
    }

    uint8_t buf[kMaxFrameLen];
    const int len = encode(req, buf, sizeof(buf));
    if (len < 0)
        return len;
    tx_seq_++;
    bytes_out_ += static_cast<uint64_t>(len);
    frames_out_++;
    if (on_write_)
        on_write_(buf, static_cast<std::size_t>(len));
    return len;
}

mp_link_stats Link::stats() const {
    const ParserCounters &c = parser_.counters();
    mp_link_stats s{};
    s.struct_size = sizeof(s);
    s.bytes_in = c.bytes;
    s.bytes_out = bytes_out_;
    s.frames_in = c.frames;
    s.frames_out = frames_out_;
    s.mavlink1_in = mavlink1_in_;
    s.mavlink2_in = mavlink2_in_;
    s.signed_in = signed_in_;
    s.crc_errors = c.crc_errors;
    s.unknown_msgid = c.unknown_msgid;
    s.parse_errors = c.parse_errors;
    s.signature_failures = signature_failures_;
    s.text_lines = c.text_lines;
    return s;
}

}  // namespace mpcore
