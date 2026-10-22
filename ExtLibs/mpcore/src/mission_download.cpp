#include "mission_download.hpp"

#include "mavlink_c.hpp"

namespace mpcore {

namespace {

uint8_t payload_target(uint32_t sysid) {
    return sysid > 255 ? 255 : static_cast<uint8_t>(sysid);
}

template <typename T>
T read_payload(const FrameView &f) {
    T v{};
    copy_payload(f, reinterpret_cast<uint8_t *>(&v), sizeof(v));
    return v;
}

}  // namespace

MissionDownload::MissionDownload(uint32_t sysid, uint8_t compid, uint32_t gcs_sysid, uint8_t gcs_compid,
                                 uint8_t mission_type, bool use_int, SendFn send)
    : sysid_(sysid), compid_(compid), gcs_sysid_(gcs_sysid), gcs_compid_(gcs_compid), type_(mission_type),
      use_int_(use_int), send_(std::move(send)) {}

void MissionDownload::request_list() {
    mavlink_mission_request_list_t req{};
    req.target_system = payload_target(sysid_);
    req.target_component = compid_;
    req.mission_type = type_;
    if (send_)
        send_(MAVLINK_MSG_ID_MISSION_REQUEST_LIST, reinterpret_cast<const uint8_t *>(&req),
              MAVLINK_MSG_ID_MISSION_REQUEST_LIST_LEN);
}

void MissionDownload::request_item() {
    // MISSION_REQUEST and MISSION_REQUEST_INT have the same fields
    mavlink_mission_request_int_t req{};
    req.seq = static_cast<uint16_t>(items_.size());
    req.target_system = payload_target(sysid_);
    req.target_component = compid_;
    req.mission_type = type_;
    if (send_)
        send_(use_int_ ? MAVLINK_MSG_ID_MISSION_REQUEST_INT : MAVLINK_MSG_ID_MISSION_REQUEST,
              reinterpret_cast<const uint8_t *>(&req),
              use_int_ ? MAVLINK_MSG_ID_MISSION_REQUEST_INT_LEN : MAVLINK_MSG_ID_MISSION_REQUEST_LEN);
}

void MissionDownload::finish() {
    mavlink_mission_ack_t ack{};
    ack.target_system = payload_target(sysid_);
    ack.target_component = compid_;
    ack.type = MAV_MISSION_ACCEPTED;
    ack.mission_type = type_;
    if (send_)
        send_(MAVLINK_MSG_ID_MISSION_ACK, reinterpret_cast<const uint8_t *>(&ack), MAVLINK_MSG_ID_MISSION_ACK_LEN);
    state_ = State::Done;
}

void MissionDownload::start(int64_t now_us) {
    state_ = State::Counting;
    resends_left_ = kCountResends;
    sent_us_ = now_us;
    request_list();
}

bool MissionDownload::for_us(const FrameView &f) const {
    return f.sysid == sysid_ && f.compid == compid_ && frame_targets(f, gcs_sysid_, gcs_compid_);
}

void MissionDownload::on_frame(const uint8_t *frame, std::size_t len, int64_t now_us) {
    FrameView f;
    if (!view_frame(frame, len, f) || !for_us(f))
        return;

    if (state_ == State::Counting && f.msgid == MAVLINK_MSG_ID_MISSION_COUNT) {
        const auto count = read_payload<mavlink_mission_count_t>(f);
        if (count.mission_type != type_)
            return;
        count_ = count.count;
        if (count_ == 0) {
            finish();
            return;
        }
        state_ = State::Items;
        resends_left_ = kItemResends;
        sent_us_ = now_us;
        request_item();
        return;
    }

    if (state_ != State::Items ||
        (f.msgid != MAVLINK_MSG_ID_MISSION_ITEM_INT && f.msgid != MAVLINK_MSG_ID_MISSION_ITEM))
        return;
    uint16_t seq;
    uint8_t type;
    if (f.msgid == MAVLINK_MSG_ID_MISSION_ITEM_INT) {
        const auto item = read_payload<mavlink_mission_item_int_t>(f);
        seq = item.seq;
        type = item.mission_type;
    } else {
        const auto item = read_payload<mavlink_mission_item_t>(f);
        seq = item.seq;
        type = item.mission_type;
    }
    if (type != type_)
        return;
    if (seq != items_.size()) {
        // not the one asked for: ask again, without restarting its timer (as Mission Planner)
        request_item();
        return;
    }
    items_.emplace_back(frame, frame + len);
    if (items_.size() == static_cast<std::size_t>(count_)) {
        finish();
        return;
    }
    resends_left_ = kItemResends;
    sent_us_ = now_us;
    request_item();
}

MissionDownload::State MissionDownload::tick(int64_t now_us) {
    const int64_t timeout = state_ == State::Counting ? kCountTimeoutUs : kItemTimeoutUs;
    if ((state_ == State::Counting || state_ == State::Items) && now_us - sent_us_ >= timeout) {
        if (resends_left_ > 0) {
            resends_left_--;
            sent_us_ = now_us;
            if (state_ == State::Counting)
                request_list();
            else
                request_item();
        } else {
            state_ = State::Failed;
        }
    }
    return state_;
}

}  // namespace mpcore
