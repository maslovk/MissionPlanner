#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "protocol.hpp"

namespace mpcore {

// One request and the reply it waits for, sent again when no reply comes in time: Mission
// Planner's "send, read until the answer or 700 ms, resend up to 3 times, then time out" loops
// (setParamAsync, GetParamAsync, getWPCountAsync, ...). No I/O: frames in, the request out through
// the send callback, tick() for the timer. Only frames from the target sysid/compid are matched.
class Exchange {
public:
    using SendFn = std::function<void(uint32_t msgid, const uint8_t *payload, std::size_t len)>;
    using MatchFn = std::function<bool(const FrameView &)>;

    enum class State { Idle, Waiting, Done, TimedOut };

    struct Request {
        uint32_t sysid = 0;
        uint8_t compid = 0;
        uint32_t msgid = 0;
        std::vector<uint8_t> payload;
        MatchFn match;
        int64_t timeout_us = 700'000;
        int resends = 3;
    };

    Exchange(Request request, SendFn send);

    void start(int64_t now_us);
    void on_frame(const uint8_t *frame, std::size_t len, int64_t now_us);
    State tick(int64_t now_us);

    State state() const { return state_; }
    // the matching frame, once Done
    const std::vector<uint8_t> &reply() const { return reply_; }
    uint32_t sends() const { return sends_; }

    // PARAM_SET, answered by the PARAM_VALUE with the same name
    static Request param_set(uint32_t sysid, uint8_t compid, const std::string &name, float value, uint8_t type);
    // PARAM_REQUEST_READ by name (index -1) or by index, answered by that PARAM_VALUE
    static Request param_read(uint32_t sysid, uint8_t compid, const std::string &name, int16_t index);

private:
    void send();

    Request request_;
    SendFn send_;
    State state_ = State::Idle;
    int64_t sent_us_ = 0;
    int resends_left_ = 0;
    uint32_t sends_ = 0;
    std::vector<uint8_t> reply_;
};

}  // namespace mpcore
