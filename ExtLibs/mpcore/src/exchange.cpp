#include "exchange.hpp"

#include <cstring>

#include "mavlink_c.hpp"

namespace mpcore {

namespace {

uint8_t payload_target(uint32_t sysid) {
    // a 32-bit target travels in the TARGET32 header; the payload holds the 255 sentinel
    return sysid > 255 ? 255 : static_cast<uint8_t>(sysid);
}

// The generated structs are naturally aligned (MAVLINK_ALIGNED_FIELDS): their fields sit where the
// wire has them, but sizeof can include trailing padding, so the wire length comes from the
// generated *_LEN.
template <typename T>
std::vector<uint8_t> bytes_of(const T &value, std::size_t wire_len) {
    const auto *p = reinterpret_cast<const uint8_t *>(&value);
    return std::vector<uint8_t>(p, p + wire_len);
}

// MAVLink parameter names: up to 16 characters, NUL-terminated only when shorter
void put_name(char (&out)[16], const std::string &name) {
    std::memset(out, 0, sizeof(out));
    std::memcpy(out, name.data(), name.size() < sizeof(out) ? name.size() : sizeof(out));
}

std::string param_name(const mavlink_param_value_t &v) {
    return std::string(v.param_id, strnlen(v.param_id, sizeof(v.param_id)));
}

mavlink_param_value_t param_value(const FrameView &f) {
    mavlink_param_value_t v{};
    copy_payload(f, reinterpret_cast<uint8_t *>(&v), sizeof(v));
    return v;
}

}  // namespace

Exchange::Exchange(Request request, SendFn send) : request_(std::move(request)), send_(std::move(send)) {}

void Exchange::send() {
    sends_++;
    if (send_)
        send_(request_.msgid, request_.payload.data(), request_.payload.size());
}

void Exchange::start(int64_t now_us) {
    state_ = State::Waiting;
    resends_left_ = request_.resends;
    sent_us_ = now_us;
    send();
}

void Exchange::on_frame(const uint8_t *frame, std::size_t len, int64_t now_us) {
    (void)now_us;
    if (state_ != State::Waiting)
        return;
    FrameView f;
    if (!view_frame(frame, len, f) || f.sysid != request_.sysid || f.compid != request_.compid)
        return;
    if (request_.match && !request_.match(f))
        return;
    reply_.assign(frame, frame + len);
    state_ = State::Done;
}

Exchange::State Exchange::tick(int64_t now_us) {
    if (state_ == State::Waiting && now_us - sent_us_ >= request_.timeout_us) {
        if (resends_left_ > 0) {
            resends_left_--;
            sent_us_ = now_us;
            send();
        } else {
            state_ = State::TimedOut;
        }
    }
    return state_;
}

Exchange::Request Exchange::param_set(uint32_t sysid, uint8_t compid, const std::string &name, float value,
                                      uint8_t type) {
    mavlink_param_set_t set{};
    set.param_value = value;
    set.target_system = payload_target(sysid);
    set.target_component = compid;
    put_name(set.param_id, name);
    set.param_type = type;

    Request r;
    r.sysid = sysid;
    r.compid = compid;
    r.msgid = MAVLINK_MSG_ID_PARAM_SET;
    r.payload = bytes_of(set, MAVLINK_MSG_ID_PARAM_SET_LEN);
    const std::string wanted = name.substr(0, sizeof(set.param_id));
    r.match = [wanted](const FrameView &f) {
        return f.msgid == MAVLINK_MSG_ID_PARAM_VALUE && param_name(param_value(f)) == wanted;
    };
    return r;
}

Exchange::Request Exchange::param_read(uint32_t sysid, uint8_t compid, const std::string &name, int16_t index) {
    mavlink_param_request_read_t read{};
    read.param_index = index;
    read.target_system = payload_target(sysid);
    read.target_component = compid;
    if (index == -1)
        put_name(read.param_id, name);

    Request r;
    r.sysid = sysid;
    r.compid = compid;
    r.msgid = MAVLINK_MSG_ID_PARAM_REQUEST_READ;
    r.payload = bytes_of(read, MAVLINK_MSG_ID_PARAM_REQUEST_READ_LEN);
    const std::string wanted = index == -1 ? name.substr(0, sizeof(read.param_id)) : std::string();
    r.match = [wanted, index](const FrameView &f) {
        if (f.msgid != MAVLINK_MSG_ID_PARAM_VALUE)
            return false;
        const mavlink_param_value_t v = param_value(f);
        // another parameter's value (a stream of them may be in flight): not the answer
        return index == -1 ? param_name(v) == wanted : v.param_index == static_cast<uint16_t>(index);
    };
    return r;
}

}  // namespace mpcore
