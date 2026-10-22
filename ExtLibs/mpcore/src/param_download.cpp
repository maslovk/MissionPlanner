#include "param_download.hpp"

#include <cstring>

#include "mavlink_c.hpp"
#include "protocol.hpp"

namespace mpcore {

ParamDownload::ParamDownload(uint32_t sysid, uint8_t compid, SendFn send, ParamFn on_param)
    : sysid_(sysid), compid_(compid), send_(std::move(send)), on_param_(std::move(on_param)) {}

void ParamDownload::request_list() {
    mavlink_param_request_list_t req{};
    // a 32-bit target travels in the TARGET32 header; the payload holds the 255 sentinel
    req.target_system = sysid_ > 255 ? 255 : static_cast<uint8_t>(sysid_);
    req.target_component = compid_;
    full_list_requests_++;
    if (send_)
        send_(MAVLINK_MSG_ID_PARAM_REQUEST_LIST, reinterpret_cast<const uint8_t *>(&req),
              MAVLINK_MSG_ID_PARAM_REQUEST_LIST_LEN);
}

void ParamDownload::request_index(uint16_t index) {
    mavlink_param_request_read_t req{};
    req.param_index = static_cast<int16_t>(index);
    req.target_system = sysid_ > 255 ? 255 : static_cast<uint8_t>(sysid_);
    req.target_component = compid_;
    single_requests_++;
    if (send_)
        send_(MAVLINK_MSG_ID_PARAM_REQUEST_READ, reinterpret_cast<const uint8_t *>(&req),
              MAVLINK_MSG_ID_PARAM_REQUEST_READ_LEN);
}

void ParamDownload::start(int64_t now_us) {
    started_ = true;
    quiet_since_us_ = now_us;
    request_list();
}

void ParamDownload::on_frame(const uint8_t *frame, std::size_t len, int64_t now_us) {
    FrameView f;
    if (!view_frame(frame, len, f) || f.msgid != MAVLINK_MSG_ID_PARAM_VALUE || f.sysid != sysid_ ||
        f.compid != compid_)
        return;
    mavlink_param_value_t v{};
    copy_payload(f, reinterpret_cast<uint8_t *>(&v), sizeof(v));
    ParamValue p;
    p.value = v.param_value;
    p.count = v.param_count;
    p.index = v.param_index;
    p.type = v.param_type;
    p.name.assign(v.param_id, strnlen(v.param_id, sizeof(v.param_id)));
    on_param_value(p, now_us);
}

void ParamDownload::on_param_value(const ParamValue &p, int64_t now_us) {
    if (!started_)
        return;
    // while single requests run, only they move things on
    if (!one_by_one_)
        quiet_since_us_ = now_us;

    if (p.count != total_) {
        total_ = p.count;
        if (have_.size() < total_)
            have_.resize(total_, false);
        received_ = 0;
        for (uint32_t i = 0; i < total_; i++)
            received_ += have_[i] ? 1 : 0;
    }
    if (total_ == 0) {
        zero_ = true;
        return;
    }

    const bool listed = p.index != 65535;
    if (listed && p.index < have_.size() && have_[p.index]) {
        if (on_param_)
            on_param_(p, true);
        return;
    }

    if (listed) {
        if (p.index >= have_.size())
            have_.resize(p.index + 1u, false);
        have_[p.index] = true;
        if (p.index < total_)
            received_++;
    }
    if (on_param_)
        on_param_(p, false);

    // the last one arrived and some are missing: no need to wait for the quiet timer
    if (p.index == total_ - 1)
        missing_at_end_ = missing_at_end_ || received_ < total_;
}

void ParamDownload::tick(int64_t now_us) {
    if (!started_ || done())
        return;
    if (missing_at_end_ || now_us - quiet_since_us_ >= kQuietUs)
        retry(now_us);
}

void ParamDownload::retry(int64_t now_us) {
    // fewer than 75 % (in Mission Planner's integer arithmetic): ask for the whole list again
    if (!one_by_one_ && full_list_retries_ < kFullListRetries &&
        (total_ <= 1 || received_ < (total_ / 4) * 3)) {
        // only count a retry once the total is known; until then keep asking
        if (total_ > 1)
            full_list_retries_++;
        request_list();
        quiet_since_us_ = now_us;
        missing_at_end_ = false;
        return;
    }

    one_by_one_ = true;
    if (have_round_ && now_us - last_round_us_ <= kRoundUs)
        return;
    have_round_ = true;
    last_round_us_ = now_us;

    const uint32_t first = total_ > 1 ? next_index_ % (total_ - 1) : 0;
    int queued = 0;
    bool found = false;
    for (uint32_t i = first; i < total_ && queued < kPerRound; i++) {
        if (have_[i])
            continue;
        found = true;
        request_index(static_cast<uint16_t>(i));
        next_index_ = i + 1;
        queued++;
    }
    // nothing missing after the start: begin at the front next time
    if (!found)
        next_index_ = 0;
}

}  // namespace mpcore
