#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace mpcore {

// STX + len + incompat + compat + seq + sysid[4] + compid + msgid[3] + target[4]
constexpr std::size_t kMaxHeaderLen = 17;
constexpr std::size_t kMaxPayloadLen = 255;
constexpr std::size_t kSignatureLen = 13;
constexpr std::size_t kMaxFrameLen = kMaxHeaderLen + kMaxPayloadLen + 2 + kSignatureLen;

constexpr uint8_t kStxV1 = 0xFE;
constexpr uint8_t kStxV2 = 0xFD;
constexpr uint8_t kIflagSigned = 0x01;
constexpr uint8_t kIflagSysid32 = 0x02;
constexpr uint8_t kIflagTarget32 = 0x04;

constexpr uint8_t kNoField = 0xFF;

using SigningKey = std::array<uint8_t, 32>;

struct MsgInfo {
    uint32_t msgid = 0;
    const char *name = nullptr;
    uint8_t crc_extra = 0;
    uint8_t min_len = 0;
    uint8_t max_len = 0;
    uint8_t target_system_ofs = kNoField;
    uint8_t target_component_ofs = kNoField;
};

// nullptr if msgid is not in the compiled dialect
const MsgInfo *find_msg(uint32_t msgid);

// Header length of a frame, from its first bytes: 6 for MAVLink 1, 10..17 for MAVLink 2.
std::size_t header_length(uint8_t stx, uint8_t incompat_flags);

struct EncodeRequest {
    uint32_t msgid = 0;
    const uint8_t *payload = nullptr;
    std::size_t payload_len = 0;
    uint32_t src_sysid = 255;
    uint8_t src_compid = 190;
    uint32_t target_sysid = 0;  // the destination; > 255 adds a TARGET32 header
    uint8_t seq = 0;
    bool mavlink1 = false;
    const SigningKey *key = nullptr;  // sign when set (MAVLink 2 only)
    uint8_t link_id = 0;
    uint64_t timestamp = 0;  // signing timestamp, 10 us units since 2015-01-01
};

// The parts of a frame that has already been checked (by the parser, or Mission Planner's C#).
struct FrameView {
    uint32_t msgid = 0;
    uint32_t sysid = 0;
    uint8_t compid = 0;
    uint8_t seq = 0;
    const uint8_t *payload = nullptr;
    std::size_t payload_len = 0;  // as on the wire: MAVLink 2 may be zero-trimmed
    bool header_target = false;   // TARGET32
    uint32_t header_target_sysid = 0;
};

// false if frame is not a complete MAVLink 1/2 frame
bool view_frame(const uint8_t *frame, std::size_t len, FrameView &out);

// Whether a frame is for sysid/compid, as Mission Planner's IsTargetedTo: its target system (the
// TARGET32 header, else the payload field) is 0 or sysid, and its target component (payload) is 0
// or compid. A message without target fields is for everyone.
bool frame_targets(const FrameView &frame, uint32_t sysid, uint8_t compid);

// Copies a payload into out (out_len bytes, the message's full length), zero-filling what
// MAVLink 2 trimmed.
void copy_payload(const FrameView &frame, uint8_t *out, std::size_t out_len);

// Writes the frame to out and returns its length, or a negative MP_ERR_* code.
int encode(const EncodeRequest &request, uint8_t *out, std::size_t out_capacity);

// Checks the 48-bit signature at the end of a signed MAVLink 2 frame against key.
bool signature_valid(const uint8_t *frame, std::size_t len, const SigningKey &key);

// The 48-bit signing timestamp of a signed frame (10 us units since 2015-01-01).
uint64_t signature_timestamp(const uint8_t *frame, std::size_t len);

// SHA-256 of data, first 6 bytes (the MAVLink signature length).
std::array<uint8_t, 6> sha256_48(const uint8_t *data, std::size_t len);

}  // namespace mpcore
