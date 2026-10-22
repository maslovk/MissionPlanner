#include "frame_parser.hpp"

#include <array>
#include <deque>
#include <string>

#include "mavlink_c.hpp"
#include "protocol.hpp"

namespace mpcore {

namespace {

// Mission Planner keeps text lines longer than 3 characters; cap them so a stream of noise
// without line breaks cannot grow the buffer.
constexpr std::size_t kMinTextLine = 4;
constexpr std::size_t kMaxTextLine = 1024;

bool idle(uint8_t state) {
    return state == MAVLINK_PARSE_STATE_UNINIT || state == MAVLINK_PARSE_STATE_IDLE;
}

}  // namespace

struct FrameParser::Impl {
    FrameFn on_frame;
    TextFn on_text;
    ParserCounters counters;

    mavlink_message_t rx{};
    mavlink_status_t status{};
    mavlink_message_t out{};
    mavlink_status_t out_status{};

    std::array<uint8_t, kMaxFrameLen> raw{};
    std::size_t raw_len = 0;
    // bytes of failed frames, parsed again before the next new byte
    std::deque<uint8_t> rescan;
    bool started_in_rescan = false;
    std::string line;

    void feed_byte(uint8_t c) {
        step(c, false);
        while (!rescan.empty()) {
            const uint8_t b = rescan.front();
            rescan.pop_front();
            step(b, true);
        }
    }

    void step(uint8_t c, bool rescanning) {
        const bool was_idle = idle(status.parse_state);
        const uint8_t result = mavlink_frame_char_buffer(&rx, &status, c, &out, &out_status);

        if (was_idle) {
            if (status.parse_state == MAVLINK_PARSE_STATE_GOT_STX) {
                raw[0] = c;
                raw_len = 1;
                started_in_rescan = rescanning;
            } else if (!rescanning) {
                // the inside of a failed frame is not text
                text_byte(c);
            }
            return;
        }

        if (raw_len < raw.size())
            raw[raw_len++] = c;

        // Not finished yet. The generated parser already returns BAD_CRC at the CRC of a signed
        // frame and then still reads its 13 signature bytes; the frame ends when it is idle again.
        if (!idle(status.parse_state))
            return;

        // A start byte inside a failed frame is usually just a 0xFD/0xFE data byte: its failure
        // is not another error, and with an unknown msgid it would pass for a real frame.
        if (result == MAVLINK_FRAMING_OK) {
            deliver(true);
            raw_len = 0;
        } else if (result == MAVLINK_FRAMING_BAD_CRC) {
            // An unknown msgid fails the CRC because its crc_extra is unknown, not because the
            // frame is damaged: deliver it flagged (Mission Planner forwards those to mirrors).
            if (find_msg(out.msgid) == nullptr) {
                if (!started_in_rescan) {
                    counters.unknown_msgid++;
                    deliver(false);
                }
            } else if (!started_in_rescan) {
                counters.crc_errors++;
            }
            restart_after_frame_start();
        } else if (idle(status.parse_state)) {
            // abandoned before the CRC (unknown incompat flags, oversized length)
            if (!started_in_rescan)
                counters.parse_errors++;
            restart_after_frame_start();
        }
    }

    void abandon_partial() {
        // the bytes scanned again may stop inside another false start: repeat until idle
        while (!idle(status.parse_state)) {
            if (!started_in_rescan)
                counters.parse_errors++;
            status.parse_state = MAVLINK_PARSE_STATE_IDLE;
            restart_after_frame_start();
            while (!rescan.empty()) {
                const uint8_t b = rescan.front();
                rescan.pop_front();
                step(b, true);
            }
        }
    }

    void restart_after_frame_start() {
        if (raw_len > 1)
            rescan.insert(rescan.begin(), raw.begin() + 1, raw.begin() + static_cast<std::ptrdiff_t>(raw_len));
        raw_len = 0;
    }

    void deliver(bool known) {
        ParsedFrame f;
        f.raw = raw.data();
        f.raw_len = static_cast<uint16_t>(raw_len);
        f.mavlink2 = out.magic == kStxV2;
        f.incompat_flags = f.mavlink2 ? out.incompat_flags : 0;
        f.compat_flags = f.mavlink2 ? out.compat_flags : 0;
        f.payload_offset = static_cast<uint16_t>(header_length(out.magic, f.incompat_flags));
        f.payload_len = out.len;
        f.seq = out.seq;
        f.sysid = out.sysid;
        f.compid = out.compid;
        f.msgid = out.msgid;
        f.header_target = (f.incompat_flags & kIflagTarget32) != 0;
        f.header_target_sysid = f.header_target ? out.target_sysid : 0;
        f.known_msgid = known;
        f.has_signature = (f.incompat_flags & kIflagSigned) != 0;
        if (known)
            counters.frames++;
        if (on_frame)
            on_frame(f);
    }

    void text_byte(uint8_t c) {
        if (c == '\r' || c == '\n') {
            if (line.size() >= kMinTextLine) {
                counters.text_lines++;
                if (on_text)
                    on_text(line);
            }
            line.clear();
        } else if (c >= 0x20 && c < 0x7f) {
            line.push_back(static_cast<char>(c));
            if (line.size() >= kMaxTextLine) {
                counters.text_lines++;
                if (on_text)
                    on_text(line);
                line.clear();
            }
        }
    }
};

FrameParser::FrameParser(FrameFn on_frame, TextFn on_text) : impl_(std::make_unique<Impl>()) {
    impl_->on_frame = std::move(on_frame);
    impl_->on_text = std::move(on_text);
}

FrameParser::~FrameParser() = default;

void FrameParser::feed(const uint8_t *data, std::size_t len) {
    impl_->counters.bytes += len;
    for (std::size_t i = 0; i < len; i++)
        impl_->feed_byte(data[i]);
}

void FrameParser::abandon_partial_frame() {
    impl_->abandon_partial();
}

bool FrameParser::in_frame() const {
    return !idle(impl_->status.parse_state);
}

const ParserCounters &FrameParser::counters() const {
    return impl_->counters;
}

}  // namespace mpcore
