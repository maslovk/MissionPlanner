#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "protocol.hpp"

namespace mpcore {

// Downloads a mission, fence or rally list, as mav_mission.download with getWPCountAsync and
// getWPAsync does, without I/O (frames in, messages out, tick for the timers):
//  - MISSION_REQUEST_LIST, answered by a MISSION_COUNT for us: 700 ms, 6 resends;
//  - each item by MISSION_REQUEST_INT (MISSION_REQUEST without the vehicle's MISSION_INT
//    capability), answered by its MISSION_ITEM_INT or MISSION_ITEM: 2.5 s, 5 resends; an item
//    with another seq is asked for again at once;
//  - a MISSION_ACK (accepted) at the end.
// Replies must be for us (Mission Planner's IsTargetedTo) and, unlike Mission Planner's loops, of
// the mission type asked for; a request asked again uses the kind it was first sent as (Mission
// Planner falls back to MISSION_REQUEST after a MISSION_ITEM). The items are kept as the frames
// received, for the caller to read.
class MissionDownload {
public:
    using SendFn = std::function<void(uint32_t msgid, const uint8_t *payload, std::size_t len)>;

    enum class State { Idle, Counting, Items, Done, Failed };

    static constexpr int64_t kCountTimeoutUs = 700'000;
    static constexpr int kCountResends = 6;
    static constexpr int64_t kItemTimeoutUs = 2'500'000;
    static constexpr int kItemResends = 5;

    MissionDownload(uint32_t sysid, uint8_t compid, uint32_t gcs_sysid, uint8_t gcs_compid, uint8_t mission_type,
                    bool use_int, SendFn send);

    void start(int64_t now_us);
    void on_frame(const uint8_t *frame, std::size_t len, int64_t now_us);
    State tick(int64_t now_us);

    State state() const { return state_; }
    bool failed_counting() const { return state_ == State::Failed && count_ < 0; }
    int count() const { return count_; }
    std::size_t received() const { return items_.size(); }
    // the item frames, in seq order
    const std::vector<std::vector<uint8_t>> &items() const { return items_; }

private:
    void request_list();
    void request_item();
    void finish();
    bool for_us(const FrameView &f) const;

    uint32_t sysid_;
    uint8_t compid_;
    uint32_t gcs_sysid_;
    uint8_t gcs_compid_;
    uint8_t type_;
    bool use_int_;
    SendFn send_;

    State state_ = State::Idle;
    int count_ = -1;
    std::vector<std::vector<uint8_t>> items_;
    int64_t sent_us_ = 0;
    int resends_left_ = 0;
};

}  // namespace mpcore
