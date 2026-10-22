#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace mpcore {

// The fields of a PARAM_VALUE message.
struct ParamValue {
    std::string name;   // up to 16 characters
    float value = 0;    // as sent; how to read it (float or bytewise) is the caller's business
    uint8_t type = 0;   // MAV_PARAM_TYPE
    uint16_t index = 0; // 65535: sent on its own, not part of the list
    uint16_t count = 0; // the vehicle's total
};

// Downloads a vehicle's parameter list, as Mission Planner's getParamListAsync does, without I/O:
// frames go in through on_frame(), messages to send come out through the send callback, and
// tick() drives the retries.
//  - PARAM_REQUEST_LIST first. A parameter arriving restarts a 4 s quiet timer.
//  - When the timer runs out, or the last index arrives with gaps: request the whole list again
//    while fewer than 75 % have arrived (at most twice once the total is known), after that every
//    second up to 10 missing indexes one by one (PARAM_REQUEST_READ), continuing after the last
//    index requested.
//  - Single requests start only once the list has stopped arriving (the quiet timer restarts with
//    every parameter) or its last index came, so there is no need to wait for a list in flight.
//    (Mission Planner has a check that skips a round after more than 10 new parameters; it never
//    takes effect, and taking it would only delay filling the gaps by a second.)
//  - Done when every index below the total has arrived. There is no overall timeout; the caller
//    cancels.
class ParamDownload {
public:
    static constexpr int64_t kQuietUs = 4'000'000;
    static constexpr int64_t kRoundUs = 1'000'000;
    static constexpr int kFullListRetries = 2;
    static constexpr int kPerRound = 10;

    // msgid and payload (wire order) of a message for the target
    using SendFn = std::function<void(uint32_t msgid, const uint8_t *payload, std::size_t len)>;
    // every PARAM_VALUE from the target, and whether it was one already received
    using ParamFn = std::function<void(const ParamValue &param, bool duplicate)>;

    ParamDownload(uint32_t sysid, uint8_t compid, SendFn send, ParamFn on_param);

    void start(int64_t now_us);
    // Any frame (header to CRC/signature, already checked); PARAM_VALUE from the target counts.
    void on_frame(const uint8_t *frame, std::size_t len, int64_t now_us);
    void tick(int64_t now_us);

    // every index below the total, or a vehicle that reports no parameters at all
    bool done() const { return started_ && (zero_ || (total_ > 0 && received_ >= total_)); }
    uint32_t received() const { return received_; }
    uint32_t total() const { return total_; }
    uint32_t full_list_requests() const { return full_list_requests_; }
    uint32_t single_requests() const { return single_requests_; }
    bool one_by_one() const { return one_by_one_; }

    void on_param_value(const ParamValue &p, int64_t now_us);

private:
    void request_list();
    void request_index(uint16_t index);
    void retry(int64_t now_us);

    uint32_t sysid_;
    uint8_t compid_;
    SendFn send_;
    ParamFn on_param_;

    bool started_ = false;
    bool zero_ = false;
    uint32_t total_ = 0;  // 0 until the first PARAM_VALUE, as Mission Planner's 1
    uint32_t received_ = 0;
    std::vector<bool> have_;
    int64_t quiet_since_us_ = 0;
    bool missing_at_end_ = false;
    int full_list_retries_ = 0;
    bool one_by_one_ = false;
    bool have_round_ = false;
    int64_t last_round_us_ = 0;
    uint32_t next_index_ = 0;
    uint32_t full_list_requests_ = 0;
    uint32_t single_requests_ = 0;
};

}  // namespace mpcore
