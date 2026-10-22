#include "mission_upload.hpp"

#include "mavlink_c.hpp"

namespace mpcore {

namespace {

template <typename T>
T read_payload(const FrameView &f) {
    T v{};
    copy_payload(f, reinterpret_cast<uint8_t *>(&v), sizeof(v));
    return v;
}

}  // namespace

MissionUpload::MissionUpload(uint32_t sysid, uint8_t compid, uint32_t gcs_sysid, uint8_t gcs_compid,
                             uint8_t mission_type, std::vector<Item> items, SendFn send)
    : sysid_(sysid), compid_(compid), gcs_sysid_(gcs_sysid), gcs_compid_(gcs_compid), type_(mission_type),
      items_(std::move(items)), send_(std::move(send)) {}

void MissionUpload::send_count() {
    mavlink_mission_count_t count{};
    count.count = static_cast<uint16_t>(items_.size());
    count.target_system = sysid_ > 255 ? 255 : static_cast<uint8_t>(sysid_);
    count.target_component = compid_;
    count.mission_type = type_;
    last_sent_ = -1;
    if (send_)
        send_(MAVLINK_MSG_ID_MISSION_COUNT, reinterpret_cast<const uint8_t *>(&count), MAVLINK_MSG_ID_MISSION_COUNT_LEN);
}

void MissionUpload::send_item(uint16_t seq) {
    last_sent_ = seq;
    items_sent_++;
    const Item &item = items_[seq];
    if (send_)
        send_(item.msgid, item.payload.data(), item.payload.size());
}

void MissionUpload::start(int64_t now_us) {
    state_ = State::Counting;
    resends_left_ = kCountResends;
    sent_us_ = now_us;
    send_count();
}

void MissionUpload::on_frame(const uint8_t *frame, std::size_t len, int64_t now_us) {
    if (state_ != State::Counting && state_ != State::Items)
        return;
    FrameView f;
    if (!view_frame(frame, len, f) || f.sysid != sysid_ || f.compid != compid_ ||
        !frame_targets(f, gcs_sysid_, gcs_compid_))
        return;

    if (f.msgid == MAVLINK_MSG_ID_MISSION_ACK) {
        const auto ack = read_payload<mavlink_mission_ack_t>(f);
        if (ack.mission_type != type_)
            return;
        result_ = ack.type;
        accepted_ = true;
        state_ = State::Done;
        return;
    }

    if (f.msgid != MAVLINK_MSG_ID_MISSION_REQUEST_INT && f.msgid != MAVLINK_MSG_ID_MISSION_REQUEST)
        return;
    // MISSION_REQUEST and MISSION_REQUEST_INT have the same fields
    const auto request = read_payload<mavlink_mission_request_int_t>(f);
    if (request.mission_type != type_ || request.seq >= items_.size())
        return;
    state_ = State::Items;
    accepted_ = true;
    if (static_cast<int>(request.seq) > highest_)
        highest_ = request.seq;
    resends_left_ = kItemResends;
    sent_us_ = now_us;
    send_item(request.seq);
}

MissionUpload::State MissionUpload::tick(int64_t now_us) {
    const int64_t timeout = state_ == State::Counting ? kCountTimeoutUs : kItemTimeoutUs;
    if ((state_ == State::Counting || state_ == State::Items) && now_us - sent_us_ >= timeout) {
        if (resends_left_ > 0) {
            resends_left_--;
            sent_us_ = now_us;
            if (last_sent_ < 0)
                send_count();
            else
                send_item(static_cast<uint16_t>(last_sent_));
        } else {
            state_ = State::Failed;
        }
    }
    return state_;
}

}  // namespace mpcore
