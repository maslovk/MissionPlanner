#include <string>
#include <vector>

#include "frame_parser.hpp"
#include "test.hpp"
#include "wire.hpp"

using namespace mpcore;

namespace {

struct Seen {
    uint32_t sysid;
    uint8_t compid;
    uint8_t seq;
    uint32_t msgid;
    bool mavlink2;
    bool header_target;
    uint32_t header_target_sysid;
    bool known;
    bool signature;
    std::vector<uint8_t> raw;
    std::vector<uint8_t> payload;
};

struct Harness {
    std::vector<Seen> frames;
    std::vector<std::string> lines;
    FrameParser parser{[this](const ParsedFrame &f) {
                           frames.push_back({f.sysid, f.compid, f.seq, f.msgid, f.mavlink2, f.header_target,
                                             f.header_target_sysid, f.known_msgid, f.has_signature,
                                             std::vector<uint8_t>(f.raw, f.raw + f.raw_len),
                                             std::vector<uint8_t>(f.raw + f.payload_offset,
                                                                  f.raw + f.payload_offset + f.payload_len)});
                       },
                       [this](std::string_view line) { lines.emplace_back(line); }};

    void feed(const std::vector<uint8_t> &bytes) { parser.feed(bytes.data(), bytes.size()); }
    void feed_bytewise(const std::vector<uint8_t> &bytes) {
        for (uint8_t b : bytes)
            parser.feed(&b, 1);
    }
};

}  // namespace

// Port of Sysid32Tests.IndependentFramesDecodeAllFlagCombinations, fed one byte at a time
TEST(decodes_all_flag_combinations) {
    for (uint8_t flags = 0; flags < 8; flags++)
        for (uint32_t id : {1u, 255u, 256u, 0x7fffffffu, 0x80000000u, 0xffffffffu}) {
            if ((flags & 2) == 0 && id > 255)
                continue;
            const uint32_t target = (flags & 4) ? 0xffffffffu : 7u;
            const auto packet = wire::frame(flags, id, target, wire::command_payload(target), 76, 152);
            Harness h;
            h.feed_bytewise(packet);
            CHECK_EQ(h.frames.size(), std::size_t{1});
            if (h.frames.size() != 1)
                continue;
            const Seen &f = h.frames[0];
            CHECK_EQ(f.sysid, id);
            CHECK_EQ(f.compid, 1);
            CHECK_EQ(f.seq, 17);
            CHECK_EQ(f.msgid, 76u);
            CHECK(f.known);
            CHECK(f.raw == packet);
            CHECK_EQ(f.payload.size(), std::size_t{33});
            CHECK_EQ(f.header_target, (flags & 4) != 0);
            if (flags & 4)
                CHECK_EQ(f.header_target_sysid, target);
            CHECK_EQ(f.signature, (flags & 1) != 0);
        }
}

TEST(decodes_mavlink1) {
    Harness h;
    h.feed(wire::frame_v1(9, 4, 0, wire::kHeartbeatPayload, 50, 200));
    CHECK_EQ(h.frames.size(), std::size_t{1});
    CHECK(!h.frames[0].mavlink2);
    CHECK_EQ(h.frames[0].sysid, 9u);
    CHECK_EQ(h.frames[0].compid, 4);
    CHECK_EQ(h.frames[0].seq, 200);
    CHECK(h.frames[0].payload == wire::kHeartbeatPayload);
}

// Port of Sysid32Tests.RejectUnknownFlagsAndTruncatedOrCorruptFrames
TEST(rejects_unknown_incompat_flags_and_recovers) {
    for (int unknown : {128, 129, 134, 135}) {
        const auto flags = static_cast<uint8_t>(unknown);
        Harness h;
        h.feed(wire::concat({wire::frame(flags, 1, 255, wire::frame(0, 42)), wire::frame(2, 0xffffffff)}));
        CHECK(h.parser.counters().parse_errors >= 1);
        // Unlike the C# parser, which drops the whole rejected frame, the bytes after a rejected
        // header are scanned again, so the frame embedded in its payload is found too.
        CHECK_EQ(h.frames.size(), std::size_t{2});
        if (h.frames.size() == 2) {
            CHECK_EQ(h.frames[0].sysid, 42u);
            CHECK_EQ(h.frames[1].sysid, 0xffffffffu);
        }
    }
}

TEST(drops_corrupt_frame) {
    auto valid = wire::frame(7, 0xffffffff);
    valid[valid.size() - 14] ^= 1;  // last CRC byte
    Harness h;
    h.feed(valid);
    CHECK(h.frames.empty());
    CHECK_EQ(h.parser.counters().crc_errors, uint64_t{1});
}

TEST(truncated_frame_does_not_swallow_the_next) {
    auto first = wire::frame(0, 1);
    first.pop_back();
    Harness h;
    h.feed(wire::concat({first, wire::frame(0, 2), wire::frame(0, 3)}));
    CHECK_EQ(h.frames.size(), std::size_t{2});
    if (h.frames.size() == 2) {
        CHECK_EQ(h.frames[0].sysid, 2u);
        CHECK_EQ(h.frames[1].sysid, 3u);
    }
}

TEST(corrupted_length_does_not_swallow_the_next) {
    auto first = wire::frame(0, 1);
    first[1] = 200;  // claims 200 payload bytes: the next frames would be read as its payload
    Harness h;
    std::vector<uint8_t> stream = first;
    for (uint32_t id = 2; id < 12; id++) {
        const auto f = wire::frame(0, id);
        stream.insert(stream.end(), f.begin(), f.end());
    }
    h.feed(stream);
    CHECK_EQ(h.frames.size(), std::size_t{10});
    if (!h.frames.empty()) {
        CHECK_EQ(h.frames.front().sysid, 2u);
        CHECK_EQ(h.frames.back().sysid, 11u);
    }
    CHECK_EQ(h.parser.counters().crc_errors, uint64_t{1});
}

TEST(maximum_and_minimum_length_frames) {
    for (std::size_t length : {std::size_t{0}, std::size_t{255}}) {
        // msgid 0 has a 9 byte payload; any length frames fine, its CRC uses the same extra
        const auto packet = wire::frame(7, 0xffffffff, 255, std::vector<uint8_t>(length, 0));
        if (length == 255)
            CHECK_EQ(packet.size(), kMaxFrameLen);
        Harness h;
        h.feed(packet);
        CHECK_EQ(h.frames.size(), std::size_t{1});
    }
}

TEST(unknown_msgid_is_delivered_flagged) {
    Harness h;
    h.feed(wire::concat({wire::frame(0, 5, 255, {1, 2, 3}, 0x123456, 99), wire::frame(0, 6)}));
    CHECK_EQ(h.frames.size(), std::size_t{2});
    if (h.frames.size() == 2) {
        CHECK(!h.frames[0].known);
        CHECK_EQ(h.frames[0].msgid, 0x123456u);
        CHECK_EQ(h.frames[0].sysid, 5u);
        CHECK(h.frames[1].known);
        CHECK_EQ(h.frames[1].sysid, 6u);
    }
    CHECK_EQ(h.parser.counters().unknown_msgid, uint64_t{1});
    CHECK_EQ(h.parser.counters().frames, uint64_t{1});
}

TEST(collects_text_lines_between_frames) {
    const std::string a = "PreArm: hello\r\n";
    const std::string b = "ab\n";  // 3 characters or fewer: dropped, as in Mission Planner
    const std::string c = "$GPGGA,1\n";
    std::vector<uint8_t> stream(a.begin(), a.end());
    const auto f = wire::frame(0, 1);
    stream.insert(stream.end(), f.begin(), f.end());
    stream.insert(stream.end(), b.begin(), b.end());
    stream.insert(stream.end(), c.begin(), c.end());
    Harness h;
    h.feed_bytewise(stream);
    CHECK_EQ(h.frames.size(), std::size_t{1});
    CHECK_EQ(h.lines.size(), std::size_t{2});
    if (h.lines.size() == 2) {
        CHECK_EQ(h.lines[0], std::string("PreArm: hello"));
        CHECK_EQ(h.lines[1], std::string("$GPGGA,1"));
    }
}

TEST(inside_of_failed_frame_is_not_text) {
    std::vector<uint8_t> payload;
    const std::string inner = "not a line\r\n";
    payload.assign(inner.begin(), inner.end());
    auto bad = wire::frame(0, 1, 255, payload);
    bad[bad.size() - 1] ^= 0xFF;
    Harness h;
    h.feed(bad);
    CHECK(h.lines.empty());
}

TEST(false_start_inside_failed_frame_is_not_delivered) {
    // a damaged frame whose payload looks like an unknown-msgid frame: that is data, not a frame
    auto bad = wire::frame(0, 1, 255, wire::frame(0, 9, 255, {1, 2, 3}, 0x123456, 99));
    bad.back() ^= 0xFF;
    Harness h;
    h.feed(bad);
    CHECK(h.frames.empty());
    CHECK_EQ(h.parser.counters().unknown_msgid, uint64_t{0});
    CHECK_EQ(h.parser.counters().crc_errors, uint64_t{1});
}

// Valid frames mixed with random noise: every valid frame comes out, in order, whatever the
// noise starts (false STX bytes, huge lengths, unknown flags).
TEST(valid_frames_survive_random_noise) {
    uint32_t state = 12345;
    auto rnd = [&state]() {
        state = state * 1664525u + 1013904223u;
        return state >> 8;
    };
    std::vector<uint8_t> stream;
    std::vector<uint32_t> expected;
    for (uint32_t i = 0; i < 500; i++) {
        const std::size_t noise = rnd() % 40;
        for (std::size_t n = 0; n < noise; n++) {
            // plenty of start bytes, so the noise really starts false frames
            const uint32_t r = rnd();
            stream.push_back(r % 4 == 0 ? (r % 8 == 0 ? 0xFD : 0xFE) : static_cast<uint8_t>(r));
        }
        const auto flags = static_cast<uint8_t>(rnd() % 8);
        const uint32_t id = (flags & 2) ? 0x01000000u + i : i % 250 + 1;
        const auto f = wire::frame(flags, id, 0x80000000u, wire::command_payload(0x80000000u), 76, 152);
        stream.insert(stream.end(), f.begin(), f.end());
        expected.push_back(id);
    }
    Harness h;
    h.feed(stream);
    // the stream ends; the noise before the last frame may have opened a frame that is still
    // waiting for bytes (the link does this after 1.2 s without any)
    h.parser.abandon_partial_frame();
    CHECK(!h.parser.in_frame());
    std::vector<uint32_t> seen;
    for (const Seen &f : h.frames)
        if (f.known && f.msgid == 76)
            seen.push_back(f.sysid);
    // the noise could in theory form a frame with a valid CRC; it must never hide a real one
    std::size_t j = 0;
    for (uint32_t id : seen)
        if (j < expected.size() && id == expected[j])
            j++;
    CHECK_EQ(j, expected.size());
}
