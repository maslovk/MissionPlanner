#include "seq_tracker.hpp"
#include "test.hpp"

using namespace mpcore;

namespace {

constexpr int64_t kMs = 1000;

// Feeds seqs 10 ms apart starting at t, returns the time after the last one.
int64_t feed(SeqTracker &s, std::initializer_list<int> seqs, int64_t t = 0) {
    for (int seq : seqs) {
        s.on_packet(static_cast<uint8_t>(seq), t);
        t += 10 * kMs;
    }
    return t;
}

}  // namespace

TEST(seq_first_packet_is_its_own_duplicate) {
    SeqTracker s(10);
    CHECK_EQ(s.on_packet(10, 0), 0);
    CHECK_EQ(s.on_packet(11, 10 * kMs), 0);
    CHECK_EQ(s.packets_lost(), 0.0f);
    CHECK_EQ(s.packets_not_lost(), 2.0f);
}

TEST(seq_in_order_and_wrap) {
    SeqTracker s(250);
    feed(s, {250, 251, 252, 253, 254, 255, 0, 1, 2, 3});
    CHECK_EQ(s.packets_lost(), 0.0f);
    CHECK_EQ(s.sync_lost(), 0.0f);
    CHECK_EQ(s.link_quality(), 100.0f);
}

TEST(seq_gap_counts_lost) {
    SeqTracker s(0);
    feed(s, {0, 1, 2, 6});
    CHECK_EQ(s.packets_lost(), 3.0f);
    CHECK_EQ(s.sync_lost(), 1.0f);
    CHECK_EQ(s.newest_seq(), 6);
}

TEST(seq_late_packet_gives_loss_back) {
    SeqTracker s(0);
    feed(s, {0, 1, 2, 4});
    CHECK_EQ(s.packets_lost(), 1.0f);
    CHECK_EQ(s.on_packet(3, 40 * kMs), 0);
    CHECK_EQ(s.packets_lost(), 0.0f);
    CHECK_EQ(s.newest_seq(), 4);  // a late packet does not move the newest seq
}

TEST(seq_late_needs_a_recent_previous_packet) {
    // after a pause the seq has more likely wrapped through an outage
    SeqTracker s(0);
    feed(s, {0, 1, 2, 4});
    CHECK_EQ(s.on_packet(3, 2'000 * kMs), 254);
    CHECK_EQ(s.newest_seq(), 3);
}

// The case fixed by "Tell late MAVLink packets from seq wraps with a mask of received seqs":
// on a fast link the packet after 201 lost ones reads as 55 behind the newest seq.
TEST(seq_fast_link_wrap_is_a_loss_not_a_late_packet) {
    SeqTracker s(0);
    int64_t t = 0;
    for (int i = 0; i < 100; i++) {
        s.on_packet(static_cast<uint8_t>(i), t);
        t += kMs;
    }
    CHECK_EQ(s.packets_lost(), 0.0f);
    CHECK_EQ(s.on_packet(static_cast<uint8_t>((99 + 201) & 0xFF), t), 200);
    CHECK_EQ(s.packets_lost(), 200.0f);
    CHECK_EQ(s.sync_lost(), 1.0f);
}

TEST(seq_duplicate_changes_nothing) {
    SeqTracker s(0);
    feed(s, {0, 1, 1, 2});
    CHECK_EQ(s.packets_lost(), 0.0f);
    CHECK_EQ(s.packets_not_lost(), 4.0f);  // a duplicate still counts as received
}

TEST(seq_counters_decay_every_5_seconds) {
    SeqTracker s(0);
    feed(s, {0, 5});  // 4 lost, 2 received
    CHECK_EQ(s.packets_lost(), 4.0f);
    s.on_packet(6, 6'000 * kMs);  // decay first, then count
    CHECK_EQ(s.packets_lost(), 4.0f * 0.8f);
    CHECK_EQ(s.packets_not_lost(), 2.0f * 0.8f + 1.0f);
    s.on_packet(7, 7'000 * kMs);  // within 5 s of the last decay: none
    CHECK_EQ(s.packets_lost(), 4.0f * 0.8f);
}

TEST(seq_link_quality) {
    SeqTracker s(0);
    CHECK_EQ(s.link_quality(), 0.0f);
    feed(s, {0, 1, 2, 3, 9});  // 5 received, 5 lost
    CHECK_EQ(s.link_quality(), 50.0f);
}
