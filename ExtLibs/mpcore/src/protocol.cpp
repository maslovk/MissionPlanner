#include "protocol.hpp"

#include <algorithm>
#include <cstring>
#include <vector>

#include "mavlink_c.hpp"
#include "mpcore/mpcore.h"

namespace mpcore {

static_assert(kMaxFrameLen == MAVLINK_MAX_PACKET_LEN, "frame length disagrees with the generator");
static_assert(kMaxHeaderLen == MAVLINK_MAX_HEADER_LEN, "header length disagrees with the generator");
static_assert(kSignatureLen == MAVLINK_SIGNATURE_BLOCK_LEN, "signature length disagrees");
static_assert(kStxV2 == MAVLINK_STX && kStxV1 == MAVLINK_STX_MAVLINK1, "STX disagrees");
static_assert(kIflagSigned == MAVLINK_IFLAG_SIGNED && kIflagSysid32 == MAVLINK_IFLAG_SYSID32 &&
                  kIflagTarget32 == MAVLINK_IFLAG_TARGET32,
              "incompat flags disagree with the generator");

namespace {

struct NamedId {
    const char *name;
    uint32_t msgid;
};

// The generated tables, flattened once into one msgid-sorted vector.
std::vector<MsgInfo> build_table() {
    static const mavlink_msg_entry_t entries[] = MAVLINK_MESSAGE_CRCS;
    static const NamedId names[] = MAVLINK_MESSAGE_NAMES;

    std::vector<MsgInfo> table;
    table.reserve(std::size(entries));
    for (const auto &e : entries) {
        MsgInfo info;
        info.msgid = e.msgid;
        info.crc_extra = e.crc_extra;
        info.min_len = e.min_msg_len;
        info.max_len = e.max_msg_len;
        if (e.flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_SYSTEM)
            info.target_system_ofs = e.target_system_ofs;
        if (e.flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_COMPONENT)
            info.target_component_ofs = e.target_component_ofs;
        table.push_back(info);
    }
    std::sort(table.begin(), table.end(),
              [](const MsgInfo &a, const MsgInfo &b) { return a.msgid < b.msgid; });
    for (const auto &n : names) {
        auto it = std::lower_bound(table.begin(), table.end(), n.msgid,
                                   [](const MsgInfo &a, uint32_t id) { return a.msgid < id; });
        if (it != table.end() && it->msgid == n.msgid)
            it->name = n.name;
    }
    return table;
}

const std::vector<MsgInfo> &table() {
    static const std::vector<MsgInfo> t = build_table();
    return t;
}

}  // namespace

const MsgInfo *find_msg(uint32_t msgid) {
    const auto &t = table();
    auto it = std::lower_bound(t.begin(), t.end(), msgid,
                               [](const MsgInfo &a, uint32_t id) { return a.msgid < id; });
    return it != t.end() && it->msgid == msgid ? &*it : nullptr;
}

std::size_t header_length(uint8_t stx, uint8_t incompat_flags) {
    if (stx == kStxV1)
        return MAVLINK_CORE_HEADER_MAVLINK1_LEN + 1;
    return MAVLINK_NUM_HEADER_BYTES + ((incompat_flags & kIflagSysid32) ? 3 : 0) +
           ((incompat_flags & kIflagTarget32) ? 4 : 0);
}

int encode(const EncodeRequest &req, uint8_t *out, std::size_t out_capacity) {
    if ((req.payload == nullptr && req.payload_len != 0) || out == nullptr)
        return MP_ERR_ARGUMENT;
    const MsgInfo *info = find_msg(req.msgid);
    if (info == nullptr)
        return MP_ERR_UNKNOWN_MSG;
    if (req.payload_len > info->max_len)
        return MP_ERR_PAYLOAD_SIZE;

    mavlink_message_t msg{};
    msg.msgid = req.msgid & 0xFFFFFF;  // a 24-bit field; find_msg() above rejected larger ids
    auto *payload = reinterpret_cast<uint8_t *>(_MAV_PAYLOAD_NON_CONST(&msg));
    if (req.payload_len != 0)
        std::memcpy(payload, req.payload, req.payload_len);

    // A wide destination only exists for messages that have a target, and is carried in the
    // TARGET32 header; the 8-bit payload field then holds the 255 sentinel.
    uint32_t header_target = 0;
    if (info->target_system_ofs != kNoField && req.target_sysid > 255) {
        payload[info->target_system_ofs] = MAVLINK_TARGET_SYSTEM_SENTINEL;
        header_target = req.target_sysid;
    }

    mavlink_status_t status{};
    status.current_tx_seq = req.seq;
    if (req.mavlink1)
        status.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    mavlink_signing_t signing{};
    if (req.key != nullptr && !req.mavlink1) {
        signing.flags = MAVLINK_SIGNING_FLAG_SIGN_OUTGOING;
        signing.link_id = req.link_id;
        signing.timestamp = req.timestamp;
        std::memcpy(signing.secret_key, req.key->data(), req.key->size());
        status.signing = &signing;
    }

    if (mavlink_finalize_message_buffer_target(&msg, req.src_sysid, req.src_compid, &status,
                                                        info->min_len, info->max_len,
                                                        info->crc_extra, header_target) == 0)
        return MP_ERR_UNREPRESENTABLE;

    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    if (len == 0)
        return MP_ERR_UNREPRESENTABLE;
    if (len > out_capacity)
        return MP_ERR_ARGUMENT;
    std::memcpy(out, buf, len);
    return len;
}

std::array<uint8_t, 6> sha256_48(const uint8_t *data, std::size_t len) {
    mavlink_sha256_ctx ctx;
    mavlink_sha256_init(&ctx);
    mavlink_sha256_update(&ctx, data, static_cast<uint32_t>(len));
    std::array<uint8_t, 6> out{};
    mavlink_sha256_final_48(&ctx, out.data());
    return out;
}

bool signature_valid(const uint8_t *frame, std::size_t len, const SigningKey &key) {
    // signature = sha256_48(key + header + payload + crc + link id + timestamp)
    if (len < MAVLINK_NUM_HEADER_BYTES + 2 + kSignatureLen)
        return false;
    mavlink_sha256_ctx ctx;
    mavlink_sha256_init(&ctx);
    mavlink_sha256_update(&ctx, key.data(), static_cast<uint32_t>(key.size()));
    mavlink_sha256_update(&ctx, frame, static_cast<uint32_t>(len - 6));
    uint8_t hash[6];
    mavlink_sha256_final_48(&ctx, hash);
    return std::memcmp(hash, frame + len - 6, 6) == 0;
}

uint64_t signature_timestamp(const uint8_t *frame, std::size_t len) {
    if (len < kSignatureLen)
        return 0;
    const uint8_t *ts = frame + len - kSignatureLen + 1;
    uint64_t value = 0;
    for (int i = 5; i >= 0; i--)
        value = (value << 8) | ts[i];
    return value;
}

}  // namespace mpcore
