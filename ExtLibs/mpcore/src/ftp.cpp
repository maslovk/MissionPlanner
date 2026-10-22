#include "ftp.hpp"

#include <algorithm>
#include <cstring>
#include <iterator>

#include "mavlink_c.hpp"

namespace mpcore {

namespace {

constexpr std::size_t kFtpPayloadLen = 251;

uint8_t payload_target(uint32_t sysid) {
    return sysid > 255 ? 255 : static_cast<uint8_t>(sysid);
}

uint16_t get16(const uint8_t *p) {
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

uint32_t get32(const uint8_t *p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) | (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
}

void put16(uint8_t *p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
}

void put32(uint8_t *p, uint32_t v) {
    for (int i = 0; i < 4; i++)
        p[i] = static_cast<uint8_t>(v >> (8 * i));
}

struct Timing {
    int sends;
    int64_t timeout_us;
};

// MAVFtp's RetryTimeout for each request
Timing timing(uint8_t opcode) {
    switch (opcode) {
    case Ftp::kResetSessions:
    case Ftp::kListDirectory:
    case Ftp::kListDirectoryWithTime:
        return {5, Ftp::kSecond};
    case Ftp::kOpenFileRO:
        return {5, 2 * Ftp::kSecond};
    case Ftp::kCalcFileCRC32:
        return {3, 30 * Ftp::kSecond};
    default:
        return {30, Ftp::kSecond};
    }
}

bool parse_u64(const std::string &s, uint64_t &out) {
    if (s.empty())
        return false;
    uint64_t v = 0;
    for (char c : s) {
        if (c < '0' || c > '9')
            return false;
        const uint64_t digit = static_cast<uint64_t>(c - '0');
        if (v > (UINT64_MAX - digit) / 10)
            return false;
        v = v * 10 + digit;
    }
    out = v;
    return true;
}

// a listing's modification time: seconds since 1970, 0 for unknown; anything that is not a
// uint32 is a corrupt entry
uint32_t parse_time(const std::string &s) {
    uint64_t v = 0;
    if (!parse_u64(s, v) || v > UINT32_MAX)
        return 0;
    return static_cast<uint32_t>(v);
}

std::vector<std::string> split_tabs(const std::string &s) {
    std::vector<std::string> out(1);
    for (char c : s) {
        if (c == '\t')
            out.emplace_back();
        else
            out.back() += c;
    }
    return out;
}

}  // namespace

Ftp::Ftp(uint32_t sysid, uint8_t compid, uint32_t gcs_sysid, uint8_t gcs_compid, SendFn send)
    : sysid_(sysid), compid_(compid), gcs_sysid_(gcs_sysid), gcs_compid_(gcs_compid), send_(std::move(send)) {}

bool Ftp::begin(Phase first) {
    if (state_ == State::Running)
        return false;
    state_ = State::Running;
    phase_ = first;
    nak_ = kErrNone;
    nak_errno_ = 0;
    nak_opcode_ = kNone;
    value_ = 0;
    session_ = 0;
    command_ = kNone;
    return true;
}

void Ftp::transmit(const std::vector<uint8_t> &payload) {
    mavlink_file_transfer_protocol_t m{};
    m.target_network = 0;
    m.target_system = payload_target(sysid_);
    m.target_component = compid_;
    std::memcpy(m.payload, payload.data(), std::min(payload.size(), sizeof(m.payload)));
    requests_sent_++;
    if (send_)
        send_(MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL, reinterpret_cast<const uint8_t *>(&m),
              MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN);
}

void Ftp::arm(int64_t now_us, int64_t timeout_us, int sends) {
    sent_us_ = now_us;
    timeout_us_ = timeout_us;
    sends_ = sends;
    resends_left_ = sends - 1;
}

void Ftp::request(uint8_t opcode, uint32_t offset, uint8_t size, const uint8_t *data, std::size_t data_len,
                  int64_t now_us) {
    payload_.assign(kFtpPayloadLen, 0);
    sent_seq_ = next_seq_++;
    put16(&payload_[0], sent_seq_);
    payload_[2] = session_;
    payload_[3] = opcode;
    payload_[4] = size;
    put32(&payload_[8], offset);
    if (data != nullptr && data_len > 0)
        std::memcpy(&payload_[kHeaderLen], data, std::min(data_len, kMaxData));
    opcode_ = opcode;
    const Timing t = timing(opcode);
    arm(now_us, t.timeout_us, t.sends);
    transmit(payload_);
}

void Ftp::reset_sessions(int64_t now_us) {
    request(kResetSessions, 0, 0, nullptr, 0, now_us);
}

// a ResetSessions beside the request in flight, which is sent again when its timer runs out
void Ftp::side_reset() {
    std::vector<uint8_t> p(kFtpPayloadLen, 0);
    put16(&p[0], next_seq_++);
    p[3] = kResetSessions;
    transmit(p);
}

void Ftp::fail(uint8_t nak, uint8_t err, uint8_t opcode) {
    state_ = State::Failed;
    phase_ = Phase::None;
    nak_ = nak;
    nak_errno_ = err;
    nak_opcode_ = opcode;
}

void Ftp::cancel() {
    if (state_ == State::Running)
        fail(kErrNone, 0, opcode_);
}

// the next request follows the vehicle's latest reply (a burst answers one request many times)
void Ftp::see_seq(uint16_t reply_seq) {
    const uint16_t next = static_cast<uint16_t>(reply_seq + 1);
    if (static_cast<int16_t>(static_cast<uint16_t>(next - next_seq_)) > 0)
        next_seq_ = next;
}

bool Ftp::answers(const Reply &r) const {
    return r.req_opcode == opcode_ && r.seq == static_cast<uint16_t>(sent_seq_ + 1);
}

bool Ftp::get_file(const std::string &path, uint8_t read_size, bool burst, int64_t now_us) {
    if (state_ == State::Running || path.size() > kMaxData || read_size == 0)
        return false;
    begin(Phase::ResetBefore);
    command_ = kOpenFileRO;
    path_ = path;
    read_size_ = static_cast<uint8_t>(std::min<std::size_t>(read_size, kMaxData));
    burst_ = burst;
    file_size_ = -1;
    burst_offset_ = 0;
    chunks_.clear();
    buffer_.clear();
    data_.clear();
    reset_sessions(now_us);
    return true;
}

bool Ftp::put_file(const std::string &path, std::vector<uint8_t> data, int64_t now_us) {
    if (state_ == State::Running || path.size() > kMaxData)
        return false;
    begin(Phase::ResetBefore);
    command_ = kCreateFile;
    path_ = path;
    upload_ = std::move(data);
    pending_.clear();
    acked_ = 0;
    reset_sessions(now_us);
    return true;
}

bool Ftp::list(const std::string &dir, int64_t now_us) {
    if (state_ == State::Running || dir.size() > kMaxData)
        return false;
    begin(Phase::List);
    command_ = kListDirectory;
    path_ = dir;
    entries_.clear();
    list_op_ = timed_unsupported_ ? kListDirectory : kListDirectoryWithTime;
    list_from(0, now_us);
    return true;
}

bool Ftp::command(uint8_t opcode, const std::string &path, const std::string &path2, int64_t now_us) {
    std::string data;
    switch (opcode) {
    case kTerminateSession:
    case kResetSessions:
        break;
    case kRename:
        data = path;
        data += '\0';
        data += path2;
        break;
    case kRemoveFile:
    case kCreateDirectory:
    case kRemoveDirectory:
    case kTruncateFile:
    case kCalcFileCRC32:
        data = path;
        break;
    default:
        return false;
    }
    if (state_ == State::Running || data.size() > kMaxData)
        return false;
    begin(Phase::Command);
    command_ = opcode;
    path_ = path;
    path2_ = path2;
    request(opcode, 0, static_cast<uint8_t>(data.size()), reinterpret_cast<const uint8_t *>(data.data()),
            data.size(), now_us);
    return true;
}

void Ftp::after_reset(int64_t now_us) {
    if (phase_ == Phase::ResetAfter) {
        state_ = State::Done;
        phase_ = Phase::None;
        return;
    }
    const auto *path = reinterpret_cast<const uint8_t *>(path_.data());
    const auto len = static_cast<uint8_t>(path_.size());
    if (command_ == kOpenFileRO) {
        phase_ = Phase::Open;
        request(kOpenFileRO, 0, len, path, path_.size(), now_us);
    } else {
        phase_ = Phase::Create;
        request(kCreateFile, 0, len, path, path_.size(), now_us);
    }
}

void Ftp::on_frame(const uint8_t *frame, std::size_t len, int64_t now_us) {
    if (state_ != State::Running)
        return;
    FrameView f;
    if (!view_frame(frame, len, f) || f.msgid != MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL || f.sysid != sysid_ ||
        f.compid != compid_ || !frame_targets(f, gcs_sysid_, gcs_compid_))
        return;
    mavlink_file_transfer_protocol_t m{};
    copy_payload(f, reinterpret_cast<uint8_t *>(&m), MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN);
    const uint8_t *p = m.payload;
    Reply r{get16(p), p[2], p[3], p[4], p[5], p[6], get32(p + 8), p + kHeaderLen};
    if (r.size > kMaxData)
        r.size = static_cast<uint8_t>(kMaxData);
    if (r.opcode != kAck && r.opcode != kNak)
        return;
    const uint8_t code = r.opcode == kNak ? (r.size > 0 ? r.data[0] : static_cast<uint8_t>(kErrFail)) : 0;
    const uint8_t err = r.opcode == kNak && r.size > 1 ? r.data[1] : 0;

    switch (phase_) {
    case Phase::ResetBefore:
    case Phase::ResetAfter:
        // the outcome does not matter, only that the vehicle answered
        if (answers(r)) {
            see_seq(r.seq);
            after_reset(now_us);
        }
        return;
    case Phase::Read:
        on_read(r, now_us);
        return;
    case Phase::Write:
        on_write_ack(r, now_us);
        return;
    case Phase::List:
        on_list(r, now_us);
        return;
    case Phase::Open:
    case Phase::Create:
    case Phase::Command:
        break;
    case Phase::None:
        return;
    }

    if (!answers(r))
        return;
    see_seq(r.seq);
    if (r.opcode == kNak) {
        if (code == kErrNoSessionsAvailable) {
            side_reset();
            arm(now_us, timeout_us_, sends_);
            return;
        }
        if (phase_ == Phase::Command && command_ == kCreateDirectory && code == kErrFailFileExists) {
            state_ = State::Done;
            phase_ = Phase::None;
            return;
        }
        fail(code, err, r.req_opcode);
        return;
    }

    if (phase_ == Phase::Open) {
        const auto size = static_cast<int32_t>(r.size >= 4 ? get32(r.data) : 0);
        file_size_ = size < 0 ? 0 : size;
        session_ = r.session;
        read_op_ = burst_ ? kBurstReadFile : kReadFile;
        phase_ = Phase::Read;
        request(read_op_, 0, read_size_, nullptr, 0, now_us);
    } else if (phase_ == Phase::Create) {
        session_ = r.session;
        if (upload_.empty()) {
            phase_ = Phase::ResetAfter;
            reset_sessions(now_us);
            return;
        }
        for (std::size_t off = 0; off < upload_.size(); off += kWriteChunk)
            pending_.insert(static_cast<uint32_t>(off));
        phase_ = Phase::Write;
        send_chunk(0, now_us);
    } else {
        value_ = r.size >= 4 ? get32(r.data) : 0;
        state_ = State::Done;
        phase_ = Phase::None;
    }
}

void Ftp::read_next(uint32_t offset, int64_t now_us) {
    read_op_ = kReadFile;
    request(kReadFile, offset, read_size_, nullptr, 0, now_us);
}

void Ftp::finish_read() {
    const auto size = static_cast<std::size_t>(std::max<int64_t>(file_size_, 0));
    data_.assign(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(std::min(buffer_.size(), size)));
    state_ = State::Done;
    phase_ = Phase::None;
}

void Ftp::on_read(const Reply &r, int64_t now_us) {
    if (r.req_opcode != read_op_)
        return;
    // a burst answers one request many times; a single read must answer the latest request to
    // move on
    const bool latest = answers(r);
    if (read_op_ == kReadFile && r.opcode == kNak && !latest)
        return;
    see_seq(r.seq);

    if (r.opcode == kNak) {
        const uint8_t code = r.size > 0 ? r.data[0] : static_cast<uint8_t>(kErrFail);
        if (code != kErrEOF) {
            fail(code, r.size > 1 ? r.data[1] : 0, r.req_opcode);
            return;
        }
        if (static_cast<int64_t>(have()) >= file_size_) {
            finish_read();
        } else if (!has_gap()) {
            // the file ended before the size it was opened with
            file_size_ = prefix_end();
            finish_read();
        } else {
            read_next(prefix_end(), now_us);
        }
        return;
    }

    const uint64_t off = r.offset;
    const uint64_t len = r.size;
    const auto size = static_cast<uint64_t>(std::max<int64_t>(file_size_, 0));
    if (off > size || len > size || off + len > size || (chunks_.empty() && off > 0 && size < read_size_))
        return;
    if (len < read_size_)
        file_size_ = static_cast<int64_t>(off + len);  // a short read: the end of the file
    if (len > 0) {
        add_chunk(static_cast<uint32_t>(off), static_cast<uint32_t>(off + len));
        if (buffer_.size() < off + len)
            buffer_.resize(static_cast<std::size_t>(off + len));
        std::memcpy(buffer_.data() + off, r.data, static_cast<std::size_t>(len));
    }
    // data: the count starts again
    arm(now_us, timeout_us_, sends_);
    burst_offset_ = std::max(burst_offset_, static_cast<uint32_t>(off + len));

    if (static_cast<int64_t>(have()) >= file_size_) {
        finish_read();
        return;
    }
    if (read_op_ == kReadFile) {
        if (latest)
            read_next(prefix_end(), now_us);
        return;
    }
    if (static_cast<int64_t>(off + len) >= file_size_) {
        // the end is here and something before it is not: read the holes
        read_next(prefix_end(), now_us);
        return;
    }
    if (r.burst_complete != 0)
        request(kBurstReadFile, burst_offset_, read_size_, nullptr, 0, now_us);
}

void Ftp::send_chunk(uint32_t offset, int64_t now_us) {
    const std::size_t len = std::min(kWriteChunk, upload_.size() - offset);
    request(kWriteFile, offset, static_cast<uint8_t>(len), upload_.data() + offset, len, now_us);
}

void Ftp::on_write_ack(const Reply &r, int64_t now_us) {
    if (r.req_opcode != kWriteFile)
        return;
    see_seq(r.seq);
    if (r.opcode == kNak) {
        fail(r.size > 0 ? r.data[0] : static_cast<uint8_t>(kErrFail), r.size > 1 ? r.data[1] : 0, r.req_opcode);
        return;
    }
    if (pending_.erase(r.offset) > 0)
        acked_ += std::min<uint64_t>(kWriteChunk, upload_.size() - r.offset);
    sent_us_ = now_us;
    // the chunks sent before the latest only count; the latest's answer sends the next ones
    if (r.seq != static_cast<uint16_t>(sent_seq_ + 1))
        return;
    if (pending_.empty()) {
        phase_ = Phase::ResetAfter;
        reset_sessions(now_us);
        return;
    }
    std::vector<uint32_t> next;
    for (auto it = pending_.begin(); it != pending_.end() && next.size() < kWriteWindow; ++it)
        next.push_back(*it);
    for (uint32_t off : next)
        send_chunk(off, now_us);
}

void Ftp::list_from(uint32_t offset, int64_t now_us) {
    request(list_op_, offset, static_cast<uint8_t>(path_.size()), reinterpret_cast<const uint8_t *>(path_.data()),
            path_.size(), now_us);
}

void Ftp::on_list(const Reply &r, int64_t now_us) {
    if (!answers(r))
        return;
    see_seq(r.seq);
    const bool timed = list_op_ == kListDirectoryWithTime;
    if (r.opcode == kNak) {
        const uint8_t code = r.size > 0 ? r.data[0] : static_cast<uint8_t>(kErrFail);
        if (timed && entries_.empty() && (code == kErrUnknownCommand || code == kErrFail)) {
            // the vehicle does not know timed listings
            timed_unsupported_ = true;
            list_op_ = kListDirectory;
            list_from(0, now_us);
            return;
        }
        if (code == kErrEOF) {
            state_ = State::Done;
            phase_ = Phase::None;
            return;
        }
        fail(code, r.size > 1 ? r.data[1] : 0, r.req_opcode);
        return;
    }

    const std::size_t before = entries_.size();
    std::size_t i = 0;
    while (i < r.size) {
        const uint8_t kind = r.data[i++];
        std::string text;
        while (i < r.size && r.data[i] != 0)
            text += static_cast<char>(r.data[i++]);
        i++;  // the terminator
        Entry e;
        if (kind == 'F') {
            const auto fields = split_tabs(text);
            e.name = fields[0];
            if (fields.size() > 1)
                parse_u64(fields[1], e.size);
            if (timed && fields.size() > 2)
                e.mtime = parse_time(fields[2]);
        } else if (kind == 'D') {
            e.kind = 'D';
            if (timed) {
                // "<name>\t<size>\t<mtime>" like a file; a plain listing's tab is part of the name
                const auto fields = split_tabs(text);
                e.name = fields[0];
                if (fields.size() > 2)
                    e.mtime = parse_time(fields[2]);
            } else {
                e.name = text;
            }
        } else if (kind == 'S') {
            e.kind = 'S';
        } else if (!text.empty()) {
            e.name = text;  // an unknown kind: a file
        } else {
            continue;
        }
        entries_.push_back(std::move(e));
    }

    if (entries_.size() == before) {
        state_ = State::Done;
        phase_ = Phase::None;
        return;
    }
    list_from(static_cast<uint32_t>(entries_.size()), now_us);
}

Ftp::State Ftp::tick(int64_t now_us) {
    if (state_ == State::Running && now_us - sent_us_ >= timeout_us_) {
        if (resends_left_ > 0) {
            resends_left_--;
            sent_us_ = now_us;
            transmit(payload_);
        } else if (phase_ == Phase::ResetBefore || phase_ == Phase::ResetAfter) {
            after_reset(now_us);
        } else {
            fail(kErrNone, 0, opcode_);
        }
    }
    return state_;
}

uint64_t Ftp::progress() const {
    return command_ == kCreateFile ? acked_ : have();
}

uint64_t Ftp::have() const {
    uint64_t n = 0;
    for (const auto &c : chunks_)
        n += c.second - c.first;
    return n;
}

uint32_t Ftp::prefix_end() const {
    if (chunks_.empty() || chunks_.begin()->first != 0)
        return 0;
    return chunks_.begin()->second;
}

bool Ftp::has_gap() const {
    return chunks_.size() > 1 || (!chunks_.empty() && chunks_.begin()->first != 0);
}

void Ftp::add_chunk(uint32_t from, uint32_t to) {
    auto it = chunks_.upper_bound(from);
    if (it != chunks_.begin()) {
        auto prev = std::prev(it);
        if (prev->second >= from) {
            from = prev->first;
            to = std::max(to, prev->second);
            chunks_.erase(prev);
        }
    }
    while (it != chunks_.end() && it->first <= to) {
        to = std::max(to, it->second);
        it = chunks_.erase(it);
    }
    chunks_[from] = to;
}

}  // namespace mpcore
