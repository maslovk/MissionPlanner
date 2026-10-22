// The exported C ABI: thin, exception-free wrappers over mpcore::Link.

#include <cstddef>
#include <cstring>
#include <new>

#include "link.hpp"
#include "mpcore/mpcore.h"
#include "protocol.hpp"

struct mp_link {
    explicit mp_link(const mpcore::LinkConfig &config) : link(config) {}
    mpcore::Link link;
    mp_link_callbacks callbacks{};
};

// Layouts the C# binding relies on (MissionPlanner.Mavlink.Native)
static_assert(offsetof(mp_encode_args, timestamp) == 32, "mp_encode_args.timestamp moved");
static_assert(offsetof(mp_encode_args, key) == 40, "mp_encode_args.key moved");
static_assert(MPCORE_MAX_FRAME_LEN == mpcore::kMaxFrameLen, "MPCORE_MAX_FRAME_LEN is wrong");

namespace {

// Accept a struct from an older caller (smaller, with a prefix of today's fields) but never
// read past what the caller says it passed.
template <typename T>
bool copy_in(const T *in, T &out) {
    if (in == nullptr || in->struct_size < sizeof(uint32_t) * 2 || in->struct_size > sizeof(T))
        return false;
    out = T{};
    std::memcpy(&out, in, in->struct_size);
    return true;
}

template <typename T>
int copy_out(const T &value, T *out) {
    if (out == nullptr || out->struct_size < sizeof(uint32_t) || out->struct_size > sizeof(T))
        return MP_ERR_ARGUMENT;
    const uint32_t size = out->struct_size;
    std::memcpy(out, &value, size);
    out->struct_size = size;
    return MP_OK;
}

void install(mp_link *l) {
    const mp_link_callbacks cb = l->callbacks;
    l->link.on_frame(cb.on_frame ? mpcore::Link::FrameFn([cb](const mp_frame &f) { cb.on_frame(cb.user, &f); })
                                 : mpcore::Link::FrameFn());
    l->link.on_write(cb.on_write
                         ? mpcore::Link::WriteFn([cb](const uint8_t *d, std::size_t n) { cb.on_write(cb.user, d, n); })
                         : mpcore::Link::WriteFn());
    l->link.on_text(cb.on_text ? mpcore::Link::TextFn([cb](std::string_view s) { cb.on_text(cb.user, s.data(), s.size()); })
                               : mpcore::Link::TextFn());
}

mp_peer_stats to_stats(const mpcore::Peer &p) {
    mp_peer_stats s{};
    s.struct_size = sizeof(s);
    s.sysid = p.sysid;
    s.compid = p.compid;
    s.mavlink2 = p.mavlink2 ? 1 : 0;
    s.signing = p.key ? 1 : 0;
    s.last_seq = p.seq.newest_seq();
    s.packets_lost = p.seq.packets_lost();
    s.packets_not_lost = p.seq.packets_not_lost();
    s.sync_lost = p.seq.sync_lost();
    s.link_quality = p.seq.link_quality();
    s.frames = p.frames;
    s.last_rx_us = p.last_rx_us;
    return s;
}

}  // namespace

extern "C" {

MPCORE_API uint32_t MPCORE_CALL mp_abi_version(void) {
    return MPCORE_ABI_VERSION;
}

MPCORE_API const char *MPCORE_CALL mp_version_string(void) {
    return "mpcore 0.1.0";
}

MPCORE_API int MPCORE_CALL mp_msg_info_get(uint32_t msgid, mp_msg_info *out) {
    const mpcore::MsgInfo *info = mpcore::find_msg(msgid);
    if (info == nullptr)
        return MP_ERR_UNKNOWN_MSG;
    mp_msg_info m{};
    m.struct_size = sizeof(m);
    m.msgid = info->msgid;
    m.name = info->name;
    m.crc_extra = info->crc_extra;
    m.min_len = info->min_len;
    m.max_len = info->max_len;
    m.target_system_ofs = info->target_system_ofs;
    m.target_component_ofs = info->target_component_ofs;
    return copy_out(m, out);
}

MPCORE_API int MPCORE_CALL mp_frame_encode(const mp_encode_args *args, const void *payload,
                                           size_t payload_len, uint8_t *out, size_t out_capacity) {
    mp_encode_args a{};
    if (!copy_in(args, a) || out == nullptr)
        return MP_ERR_ARGUMENT;
    mpcore::SigningKey key{};
    mpcore::EncodeRequest req;
    req.msgid = a.msgid;
    req.payload = static_cast<const uint8_t *>(payload);
    req.payload_len = payload_len;
    req.src_sysid = a.src_sysid;
    req.src_compid = a.src_compid;
    req.target_sysid = a.target_sysid;
    req.seq = a.seq;
    req.mavlink1 = a.mavlink1 != 0;
    if (a.sign != 0 && a.mavlink1 == 0) {
        if (a.key == nullptr)
            return MP_ERR_ARGUMENT;
        std::memcpy(key.data(), a.key, key.size());
        req.key = &key;
        req.link_id = a.link_id;
        req.timestamp = a.timestamp;
    }
    return mpcore::encode(req, out, out_capacity);
}

MPCORE_API void MPCORE_CALL mp_link_config_init(mp_link_config *config) {
    if (config == nullptr)
        return;
    *config = mp_link_config{};
    config->struct_size = sizeof(*config);
    config->gcs_sysid = 255;
    config->gcs_compid = 190;
}

MPCORE_API mp_link *MPCORE_CALL mp_link_create(const mp_link_config *config,
                                               const mp_link_callbacks *callbacks) {
    mp_link_config c{};
    mp_link_config_init(&c);
    if (config != nullptr && !copy_in(config, c))
        return nullptr;
    mpcore::LinkConfig lc;
    lc.gcs_sysid = c.gcs_sysid;
    lc.gcs_compid = c.gcs_compid;
    lc.ignore_signatures = c.ignore_signatures != 0;

    auto *l = new (std::nothrow) mp_link(lc);
    if (l == nullptr)
        return nullptr;
    if (callbacks != nullptr && mp_link_set_callbacks(l, callbacks) != MP_OK) {
        delete l;
        return nullptr;
    }
    return l;
}

MPCORE_API void MPCORE_CALL mp_link_destroy(mp_link *link) {
    delete link;
}

MPCORE_API int MPCORE_CALL mp_link_set_callbacks(mp_link *link, const mp_link_callbacks *callbacks) {
    if (link == nullptr)
        return MP_ERR_ARGUMENT;
    mp_link_callbacks cb{};
    if (callbacks != nullptr && !copy_in(callbacks, cb))
        return MP_ERR_ARGUMENT;
    link->callbacks = cb;
    install(link);
    return MP_OK;
}

MPCORE_API size_t MPCORE_CALL mp_link_feed(mp_link *link, const uint8_t *data, size_t len,
                                           int64_t now_us) {
    if (link == nullptr)
        return 0;
    try {
        return link->link.feed(data, len, now_us);
    } catch (...) {
        return 0;  // never let an exception cross the ABI (only std::bad_alloc is possible)
    }
}

MPCORE_API size_t MPCORE_CALL mp_link_poll(mp_link *link, int64_t now_us) {
    if (link == nullptr)
        return 0;
    try {
        return link->link.poll(now_us);
    } catch (...) {
        return 0;
    }
}

MPCORE_API int MPCORE_CALL mp_link_send(mp_link *link, uint32_t msgid, const void *payload,
                                        size_t payload_len, uint32_t target_sysid,
                                        uint8_t target_compid, uint32_t send_flags) {
    if (link == nullptr)
        return MP_ERR_ARGUMENT;
    try {
        return link->link.send(msgid, static_cast<const uint8_t *>(payload), payload_len, target_sysid,
                               target_compid, send_flags);
    } catch (...) {
        return MP_ERR_ARGUMENT;
    }
}

MPCORE_API int MPCORE_CALL mp_link_add_signing_key(mp_link *link, const uint8_t key[32]) {
    if (link == nullptr || key == nullptr)
        return MP_ERR_ARGUMENT;
    mpcore::SigningKey k{};
    std::memcpy(k.data(), key, k.size());
    try {
        link->link.add_signing_key(k);
    } catch (...) {
        return MP_ERR_ARGUMENT;
    }
    return MP_OK;
}

MPCORE_API void MPCORE_CALL mp_link_clear_signing_keys(mp_link *link) {
    if (link != nullptr)
        link->link.clear_signing_keys();
}

MPCORE_API int MPCORE_CALL mp_link_set_peer_signing(mp_link *link, uint32_t sysid, uint8_t compid,
                                                    const uint8_t key[32], uint8_t link_id) {
    if (link == nullptr)
        return MP_ERR_ARGUMENT;
    mpcore::SigningKey k{};
    if (key != nullptr)
        std::memcpy(k.data(), key, k.size());
    try {
        link->link.set_peer_signing(sysid, compid, key != nullptr ? &k : nullptr, link_id);
    } catch (...) {
        return MP_ERR_ARGUMENT;
    }
    return MP_OK;
}

MPCORE_API int MPCORE_CALL mp_link_peer_stats(mp_link *link, uint32_t sysid, uint8_t compid,
                                              mp_peer_stats *out) {
    if (link == nullptr)
        return MP_ERR_ARGUMENT;
    const mpcore::Peer *p = link->link.find_peer(sysid, compid);
    if (p == nullptr)
        return MP_ERR_NOT_FOUND;
    return copy_out(to_stats(*p), out);
}

MPCORE_API size_t MPCORE_CALL mp_link_peer_count(mp_link *link) {
    return link == nullptr ? 0 : link->link.peer_count();
}

MPCORE_API int MPCORE_CALL mp_link_peer_at(mp_link *link, size_t index, mp_peer_stats *out) {
    if (link == nullptr)
        return MP_ERR_ARGUMENT;
    if (index >= link->link.peer_count())
        return MP_ERR_NOT_FOUND;
    return copy_out(to_stats(link->link.peer_at(index)), out);
}

MPCORE_API int MPCORE_CALL mp_link_stats_get(mp_link *link, mp_link_stats *out) {
    if (link == nullptr)
        return MP_ERR_ARGUMENT;
    return copy_out(link->link.stats(), out);
}

}  // extern "C"
