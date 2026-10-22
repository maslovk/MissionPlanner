#include "seq_tracker.hpp"

namespace mpcore {

int SeqTracker::on_packet(uint8_t seq, int64_t now_us) {
    // Mission Planner's decay timer starts at DateTime.MinValue, so the first packet decays
    // (a no-op on zero counters) and starts the 5 s interval.
    if (!have_decay_ || now_us - last_decay_us_ > kDecayIntervalUs) {
        have_decay_ = true;
        last_decay_us_ = now_us;
        lost_ *= 0.8f;
        not_lost_ *= 0.8f;
    }

    const int delta = (seq - newest_) & 0xFF;
    const int behind = 0x100 - delta;
    const uint64_t late_bit = behind <= kLateWindow ? uint64_t{1} << (behind - 1) : 0;
    const bool recent = have_packet_ && now_us - last_packet_us_ < kLateMaxGapUs;

    int lost = 0;
    if (delta == 0) {
        // duplicate
    } else if (late_bit != 0 && (mask_ & late_bit) == 0 && recent) {
        // a late packet fills a hole that was counted as lost when the newer one came
        mask_ |= late_bit;
        if (lost_ >= 1)
            lost_--;
    } else {
        lost = delta - 1;
        if (lost != 0) {
            sync_lost_++;
            lost_ += static_cast<float>(lost);
        }
        // the previous newest packet is now delta behind
        uint64_t mask = delta < 64 ? mask_ << delta : 0;
        if (delta <= kLateWindow)
            mask |= uint64_t{1} << (delta - 1);
        mask_ = mask;
        newest_ = seq;
    }
    not_lost_++;

    have_packet_ = true;
    last_packet_us_ = now_us;
    return lost;
}

float SeqTracker::link_quality() const {
    if (not_lost_ == 0)
        return 0;
    float q = not_lost_ / (not_lost_ + lost_) * 100.0f;
    return q > 100 ? 100 : q;
}

}  // namespace mpcore
