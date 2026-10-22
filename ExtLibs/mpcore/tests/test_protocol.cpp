#include <cstring>
#include <string>

#include "mpcore/mpcore.h"
#include "protocol.hpp"
#include "test.hpp"
#include "wire.hpp"

using namespace mpcore;

namespace {

std::vector<uint8_t> encode_vec(const EncodeRequest &req) {
    std::vector<uint8_t> out(kMaxFrameLen);
    const int n = encode(req, out.data(), out.size());
    if (n < 0)
        return {};
    out.resize(static_cast<std::size_t>(n));
    return out;
}

}  // namespace

TEST(sha256_known_answer) {
    const uint8_t abc[] = {'a', 'b', 'c'};
    const auto h = sha256_48(abc, sizeof(abc));
    const std::array<uint8_t, 6> expected = {0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01};
    CHECK(h == expected);
}

TEST(message_table_matches_dialect) {
    const MsgInfo *hb = find_msg(0);
    CHECK(hb != nullptr);
    CHECK_EQ(hb->crc_extra, 50);
    CHECK_EQ(hb->min_len, 9);
    CHECK_EQ(std::string(hb->name), std::string("HEARTBEAT"));
    CHECK_EQ(hb->target_system_ofs, kNoField);

    const MsgInfo *cmd = find_msg(76);
    CHECK(cmd != nullptr);
    CHECK_EQ(std::string(cmd->name), std::string("COMMAND_LONG"));
    CHECK_EQ(cmd->crc_extra, 152);
    CHECK_EQ(cmd->target_system_ofs, 30);
    CHECK_EQ(cmd->target_component_ofs, 31);

    // MAVLink 2 only: extensions make max_len larger than min_len
    const MsgInfo *gps = find_msg(24);
    CHECK(gps != nullptr && gps->max_len > gps->min_len);

    CHECK(find_msg(0xFFFFFF) == nullptr);
}

TEST(header_lengths) {
    CHECK_EQ(header_length(kStxV1, 0), std::size_t{6});
    CHECK_EQ(header_length(kStxV2, 0), std::size_t{10});
    CHECK_EQ(header_length(kStxV2, kIflagSysid32), std::size_t{13});
    CHECK_EQ(header_length(kStxV2, kIflagTarget32), std::size_t{14});
    CHECK_EQ(header_length(kStxV2, kIflagSysid32 | kIflagTarget32 | kIflagSigned), std::size_t{17});
}

// Port of Sysid32Tests.EncoderMatchesIndependentFramesAndSignatures
TEST(encoder_matches_independent_frames_and_signatures) {
    const SigningKey key = wire::test_key();
    for (uint32_t source : {1u, 255u, 256u, 0xffffffffu})
        for (uint32_t target : {0u, 7u, 0x80000000u, 0xffffffffu})
            for (bool sign : {false, true}) {
                const auto payload = wire::command_payload(target);
                EncodeRequest req;
                req.msgid = 76;
                req.payload = payload.data();
                req.payload_len = payload.size();
                req.src_sysid = source;
                req.src_compid = 1;
                req.target_sysid = target;
                req.seq = 17;
                if (sign) {
                    req.key = &key;
                    req.link_id = 3;
                    req.timestamp = 1;
                }
                const uint8_t flags = static_cast<uint8_t>((source > 255 ? 2 : 0) | (target > 255 ? 4 : 0) |
                                                           (sign ? 1 : 0));
                const auto expected = wire::frame(flags, source, target, payload, 76, 152);
                const auto actual = encode_vec(req);
                CHECK(actual == expected);
                if (sign)
                    CHECK(signature_valid(actual.data(), actual.size(), key));
            }
}

TEST(encoder_writes_sentinel_for_wide_payload_target) {
    // the caller's payload holds a stale target; a wide destination replaces it with 255
    auto payload = wire::command_payload(9);
    EncodeRequest req;
    req.msgid = 76;
    req.payload = payload.data();
    req.payload_len = payload.size();
    req.target_sysid = 0x12345678;
    const auto f = encode_vec(req);
    CHECK(!f.empty());
    CHECK_EQ(f[2], kIflagTarget32);
    const std::size_t hl = header_length(f[0], f[2]);
    CHECK_EQ(f[hl + 30], 255);
    CHECK_EQ(f[hl + 31], 190);
}

TEST(encoder_ignores_wide_target_for_untargeted_message) {
    EncodeRequest req;
    req.msgid = 0;
    req.payload = wire::kHeartbeatPayload.data();
    req.payload_len = wire::kHeartbeatPayload.size();
    req.src_sysid = 1;
    req.src_compid = 1;
    req.target_sysid = 0x80000000;  // a destination peer, but HEARTBEAT has no target field
    req.seq = 17;
    CHECK(encode_vec(req) == wire::frame(0, 1));
}

TEST(encoder_mavlink1) {
    EncodeRequest req;
    req.msgid = 0;
    req.payload = wire::kHeartbeatPayload.data();
    req.payload_len = wire::kHeartbeatPayload.size();
    req.src_sysid = 1;
    req.src_compid = 1;
    req.seq = 17;
    req.mavlink1 = true;
    CHECK(encode_vec(req) == wire::frame_v1(1, 1, 0, wire::kHeartbeatPayload, 50));

    // never truncated into MAVLink 1: that would alias another system
    req.src_sysid = 256;
    std::vector<uint8_t> out(kMaxFrameLen);
    CHECK_EQ(encode(req, out.data(), out.size()), MP_ERR_UNREPRESENTABLE);
    req.src_sysid = 1;
    req.msgid = 256;  // no MAVLink 1 msgid above 255
    req.payload_len = 0;
    CHECK_EQ(encode(req, out.data(), out.size()), MP_ERR_UNREPRESENTABLE);
}

TEST(encoder_trims_mavlink2_payload) {
    const std::vector<uint8_t> zeros(9, 0);
    EncodeRequest req;
    req.msgid = 0;
    req.payload = zeros.data();
    req.payload_len = zeros.size();
    const auto f = encode_vec(req);
    CHECK_EQ(f[1], 1);  // MAVLink 2 keeps at least one payload byte
    CHECK_EQ(f.size(), std::size_t{10 + 1 + 2});
}

TEST(encoder_rejects_bad_requests) {
    std::vector<uint8_t> out(kMaxFrameLen);
    EncodeRequest req;
    req.msgid = 0xFFFFFF;
    CHECK_EQ(encode(req, out.data(), out.size()), MP_ERR_UNKNOWN_MSG);

    const std::vector<uint8_t> big(10, 1);
    req.msgid = 0;
    req.payload = big.data();
    req.payload_len = big.size();
    CHECK_EQ(encode(req, out.data(), out.size()), MP_ERR_PAYLOAD_SIZE);

    req.payload = nullptr;
    CHECK_EQ(encode(req, out.data(), out.size()), MP_ERR_ARGUMENT);

    req.payload = wire::kHeartbeatPayload.data();
    req.payload_len = wire::kHeartbeatPayload.size();
    CHECK_EQ(encode(req, out.data(), 5), MP_ERR_ARGUMENT);
}

TEST(signature_timestamp_is_read_back) {
    const auto f = wire::frame(1, 1);
    CHECK_EQ(signature_timestamp(f.data(), f.size()), uint64_t{1});
    CHECK(signature_valid(f.data(), f.size(), wire::test_key()));
    CHECK(!signature_valid(f.data(), f.size(), wire::test_key(43)));
}
