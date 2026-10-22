#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "frame_parser.hpp"
#include "mpcore/mpcore.h"
#include "protocol.hpp"
#include "seq_tracker.hpp"

namespace mpcore {

struct LinkConfig {
    uint32_t gcs_sysid = 255;
    uint8_t gcs_compid = 190;  // MAV_COMP_ID_MISSIONPLANNER
    bool ignore_signatures = false;
};

// What the link knows about one sysid/compid it has heard from (Mission Planner's MAVState,
// protocol side only).
struct Peer {
    Peer(uint32_t sysid_, uint8_t compid_, uint8_t first_seq)
        : sysid(sysid_), compid(compid_), seq(first_seq) {}

    uint32_t sysid;
    uint8_t compid;
    SeqTracker seq;
    bool mavlink2 = false;            // latched by the first MAVLink 2 frame
    std::optional<SigningKey> key;    // sign to / verify from this peer
    uint8_t send_link_id = 0;
    uint64_t frames = 0;
    int64_t last_rx_us = 0;
};

// One MAVLink byte stream. No I/O: bytes come in through feed() and go out through the write
// handler. Not thread-safe; handlers run synchronously inside feed() and send().
class Link {
public:
    using FrameFn = std::function<void(const mp_frame &)>;
    using WriteFn = std::function<void(const uint8_t *, std::size_t)>;
    using TextFn = std::function<void(std::string_view)>;
    using ClockFn = std::function<uint64_t()>;  // microseconds since the Unix epoch

    explicit Link(LinkConfig config = {});
    Link(const Link &) = delete;
    Link &operator=(const Link &) = delete;

    void on_frame(FrameFn fn) { on_frame_ = std::move(fn); }
    void on_write(WriteFn fn) { on_write_ = std::move(fn); }
    void on_text(TextFn fn) { on_text_ = std::move(fn); }
    // Wall clock for signing timestamps; the system clock unless replaced (tests).
    void set_wall_clock(ClockFn fn) { wall_clock_ = std::move(fn); }

    // A partly received frame is given up on after this long without bytes (Mission Planner's
    // 1200 ms read timeout), and its bytes are scanned again.
    static constexpr int64_t kStallUs = 1'200'000;

    // Returns the number of frames delivered. Calls from inside a handler are ignored.
    std::size_t feed(const uint8_t *data, std::size_t len, int64_t now_us);
    // Call while no bytes arrive (e.g. from the reader loop) so a stalled frame start does not
    // hold back the frames inside it. Returns the number of frames delivered.
    std::size_t poll(int64_t now_us);

    // Returns the bytes written or a negative MP_ERR_* (see mp_link_send).
    int send(uint32_t msgid, const uint8_t *payload, std::size_t payload_len, uint32_t target_sysid,
             uint8_t target_compid, uint32_t flags = 0);

    void add_signing_key(const SigningKey &key) { keys_.push_back(key); }
    void clear_signing_keys() { keys_.clear(); }
    void set_peer_signing(uint32_t sysid, uint8_t compid, const SigningKey *key, uint8_t link_id);

    const Peer *find_peer(uint32_t sysid, uint8_t compid) const;
    std::size_t peer_count() const { return peers_.size(); }
    const Peer &peer_at(std::size_t i) const { return *peers_[i]; }

    mp_link_stats stats() const;
    const LinkConfig &config() const { return config_; }

private:
    static uint64_t key_of(uint32_t sysid, uint8_t compid) {
        return (uint64_t{sysid} << 8) | compid;
    }
    Peer &peer_for(uint32_t sysid, uint8_t compid, uint8_t first_seq);
    void handle(const ParsedFrame &parsed);
    bool verify(Peer &peer, const ParsedFrame &parsed);
    uint64_t next_signing_timestamp();

    LinkConfig config_;
    FrameParser parser_;
    FrameFn on_frame_;
    WriteFn on_write_;
    TextFn on_text_;
    ClockFn wall_clock_;

    std::vector<std::unique_ptr<Peer>> peers_;  // in the order first heard
    std::unordered_map<uint64_t, Peer *> peer_index_;
    std::vector<SigningKey> keys_;

    void release_stalled(int64_t now_us);

    int64_t now_us_ = 0;
    bool have_rx_ = false;
    int64_t last_rx_us_ = 0;
    std::size_t delivered_ = 0;
    bool feeding_ = false;
    uint8_t tx_seq_ = 0;
    uint64_t last_signing_timestamp_ = 0;

    uint64_t bytes_out_ = 0;
    uint64_t frames_out_ = 0;
    uint64_t mavlink1_in_ = 0;
    uint64_t mavlink2_in_ = 0;
    uint64_t signed_in_ = 0;
    uint64_t signature_failures_ = 0;
};

}  // namespace mpcore
