#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include "protocol.hpp"

namespace mpcore {

// Uploads a mission, fence or rally list the way the MAVLink mission protocol has the vehicle drive
// it, without I/O: MISSION_COUNT, then every MISSION_REQUEST_INT / MISSION_REQUEST is answered with
// the item it asks for (in any order, as often as asked), until the vehicle's MISSION_ACK, whose
// result is the outcome. Timeouts as Mission Planner's: 700 ms and 3 resends for the count
// (setWPTotalAsync), 450 ms and 10 resends of the last item sent (setWPAsync). Messages must be from
// the vehicle, for us and of the mission type.
//
// Mission Planner's upload instead sends each item and waits for the next request, and needs
// MISSION_WRITE_PARTIAL_LIST and "ask which item it wants" recovery when a request or an ACK is
// lost (with a bug in uploadPartial's resume index); answering what is asked makes those
// unnecessary.
class MissionUpload {
public:
    using SendFn = std::function<void(uint32_t msgid, const uint8_t *payload, std::size_t len)>;

    enum class State { Idle, Counting, Items, Done, Failed };

    static constexpr int64_t kCountTimeoutUs = 700'000;
    static constexpr int kCountResends = 3;
    static constexpr int64_t kItemTimeoutUs = 450'000;
    static constexpr int kItemResends = 10;

    struct Item {
        uint32_t msgid;                // MISSION_ITEM_INT or MISSION_ITEM
        std::vector<uint8_t> payload;  // with its seq, target and mission type set
    };

    MissionUpload(uint32_t sysid, uint8_t compid, uint32_t gcs_sysid, uint8_t gcs_compid, uint8_t mission_type,
                  std::vector<Item> items, SendFn send);

    void start(int64_t now_us);
    void on_frame(const uint8_t *frame, std::size_t len, int64_t now_us);
    State tick(int64_t now_us);

    State state() const { return state_; }
    // the vehicle's MISSION_ACK result (MAV_MISSION_RESULT, 0 = accepted), -1 without one
    int result() const { return result_; }
    // the vehicle asked for an item (or acknowledged): it took the count
    bool count_accepted() const { return accepted_; }
    // the highest seq asked for, -1 before any
    int highest_requested() const { return highest_; }
    uint32_t items_sent() const { return items_sent_; }

private:
    void send_count();
    void send_item(uint16_t seq);

    uint32_t sysid_;
    uint8_t compid_;
    uint32_t gcs_sysid_;
    uint8_t gcs_compid_;
    uint8_t type_;
    std::vector<Item> items_;
    SendFn send_;

    State state_ = State::Idle;
    bool accepted_ = false;
    int result_ = -1;
    int highest_ = -1;
    int last_sent_ = -1;  // -1: the count
    int64_t sent_us_ = 0;
    int resends_left_ = 0;
    uint32_t items_sent_ = 0;
};

}  // namespace mpcore
