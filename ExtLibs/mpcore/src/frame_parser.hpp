#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string_view>

namespace mpcore {

struct ParsedFrame {
    const uint8_t *raw = nullptr;  // the whole frame, valid during the callback only
    uint16_t raw_len = 0;
    uint16_t payload_offset = 0;
    uint8_t payload_len = 0;
    bool mavlink2 = false;
    uint8_t incompat_flags = 0;
    uint8_t compat_flags = 0;
    uint8_t seq = 0;
    uint32_t sysid = 0;
    uint8_t compid = 0;
    uint32_t msgid = 0;
    bool header_target = false;  // TARGET32 present
    uint32_t header_target_sysid = 0;
    bool known_msgid = true;     // false: not in the dialect, so the CRC was not checked
    bool has_signature = false;
};

struct ParserCounters {
    uint64_t bytes = 0;
    uint64_t frames = 0;
    uint64_t crc_errors = 0;
    uint64_t unknown_msgid = 0;
    uint64_t parse_errors = 0;
    uint64_t text_lines = 0;
};

// Splits a byte stream into MAVLink 1/2 frames (including 32-bit system IDs) with the
// generated MAVLink parser, plus two things Mission Planner relies on:
//  - after a failed frame the bytes after its start are scanned again, so a corrupted length
//    byte does not swallow the frames that follow it;
//  - bytes outside frames are collected into text lines (bootloader output, NMEA).
// Frames with an unknown msgid are delivered with known_msgid = false.
class FrameParser {
public:
    using FrameFn = std::function<void(const ParsedFrame &)>;
    using TextFn = std::function<void(std::string_view)>;

    FrameParser(FrameFn on_frame, TextFn on_text);
    ~FrameParser();
    FrameParser(const FrameParser &) = delete;
    FrameParser &operator=(const FrameParser &) = delete;

    void feed(const uint8_t *data, std::size_t len);
    // Gives up on a partly received frame (the stream went quiet) and scans its bytes after the
    // start byte again, so a false start cannot hold back the real frames inside it.
    void abandon_partial_frame();
    bool in_frame() const;
    const ParserCounters &counters() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace mpcore
