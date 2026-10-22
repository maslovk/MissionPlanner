#pragma once

#include <cstdint>

namespace mpcore {

// Packet loss from the 8-bit MAVLink sequence number of one sysid/compid, as Mission Planner
// counts it (MAVLinkInterface.readPacketAsync):
//  - a forward jump of delta counts delta - 1 lost packets and one sync loss;
//  - a packet up to 64 behind the newest one is late (reordered, e.g. UDP), not a wrap after
//    191..254 lost ones, if it fills a hole in the mask of recently received seqs and the
//    previous packet came less than a second ago; it gives one lost packet back;
//  - a repeated seq is a duplicate (3DR radios send those) and changes nothing;
//  - both counters decay by 0.8 every 5 s so the link quality follows recent conditions.
// The counters are floats, like Mission Planner's, so the decay matches it exactly.
class SeqTracker {
public:
    static constexpr int kLateWindow = 64;
    static constexpr int64_t kDecayIntervalUs = 5'000'000;
    static constexpr int64_t kLateMaxGapUs = 1'000'000;

    // The first packet of a new peer: taken as a duplicate of itself, so the packet before it,
    // never seen, is not marked as received.
    explicit SeqTracker(uint8_t first_seq) : newest_(first_seq) {}

    // Returns the number of packets this packet showed as lost (0 for in order, late or dup).
    int on_packet(uint8_t seq, int64_t now_us);

    float packets_lost() const { return lost_; }
    float packets_not_lost() const { return not_lost_; }
    float sync_lost() const { return sync_lost_; }
    uint8_t newest_seq() const { return newest_; }
    uint64_t received_mask() const { return mask_; }
    int64_t last_packet_us() const { return last_packet_us_; }

    // 0..100. 0 while nothing has been counted yet.
    float link_quality() const;

private:
    uint8_t newest_;
    uint64_t mask_ = 0;  // bit n set: seq (newest - n - 1) has been received
    float lost_ = 0;
    float not_lost_ = 0;
    float sync_lost_ = 0;
    bool have_packet_ = false;
    int64_t last_packet_us_ = 0;
    bool have_decay_ = false;
    int64_t last_decay_us_ = 0;
};

}  // namespace mpcore
