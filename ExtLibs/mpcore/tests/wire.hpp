#pragma once

// Builds MAVLink frames byte by byte, independently of the code under test (its own CRC, its
// own header layout). Ported from MissionPlannerTests/Mavlink/Sysid32Tests.cs Frame(), so the
// C# and C++ implementations are checked against the same fixtures.

#include <cstdint>
#include <vector>

#include "protocol.hpp"

namespace wire {

inline const std::vector<uint8_t> kHeartbeatPayload = {0, 0, 0, 0, 2, 3, 81, 4, 3};

inline mpcore::SigningKey test_key(uint8_t fill = 42) {
    mpcore::SigningKey k;
    k.fill(fill);
    return k;
}

inline uint16_t x25(const std::vector<uint8_t> &bytes, std::size_t from, uint8_t crc_extra) {
    uint16_t crc = 0xffff;
    auto add = [&crc](uint8_t value) {
        crc = static_cast<uint16_t>(crc ^ value);
        for (int i = 0; i < 8; i++)
            crc = static_cast<uint16_t>((crc >> 1) ^ ((crc & 1) != 0 ? 0x8408 : 0));
    };
    for (std::size_t i = from; i < bytes.size(); i++)
        add(bytes[i]);
    add(crc_extra);
    return crc;
}

inline void put_le32(std::vector<uint8_t> &b, uint32_t v) {
    for (int i = 0; i < 4; i++)
        b.push_back(static_cast<uint8_t>(v >> (8 * i)));
}

// flags: 1 SIGNED (key 42*32, link id 3, timestamp 1), 2 SYSID32, 4 TARGET32
inline std::vector<uint8_t> frame(uint8_t flags, uint32_t source, uint32_t target = 255,
                                  const std::vector<uint8_t> &payload = kHeartbeatPayload,
                                  uint32_t message = 0, uint8_t crc_extra = 50,
                                  uint8_t component = 1, uint8_t seq = 17) {
    std::vector<uint8_t> b = {0xFD, static_cast<uint8_t>(payload.size()), flags, 0, seq};
    if (flags & 2)
        put_le32(b, source);
    else
        b.push_back(static_cast<uint8_t>(source));
    b.push_back(component);
    b.push_back(static_cast<uint8_t>(message));
    b.push_back(static_cast<uint8_t>(message >> 8));
    b.push_back(static_cast<uint8_t>(message >> 16));
    if (flags & 4)
        put_le32(b, target);
    b.insert(b.end(), payload.begin(), payload.end());
    const uint16_t crc = x25(b, 1, crc_extra);
    b.push_back(static_cast<uint8_t>(crc));
    b.push_back(static_cast<uint8_t>(crc >> 8));
    if (flags & 1) {
        static const uint8_t link_id_and_timestamp[] = {3, 1, 0, 0, 0, 0, 0};
        b.insert(b.end(), std::begin(link_id_and_timestamp), std::end(link_id_and_timestamp));
        std::vector<uint8_t> hashed(32, 42);
        hashed.insert(hashed.end(), b.begin(), b.end());
        const auto sig = mpcore::sha256_48(hashed.data(), hashed.size());
        b.insert(b.end(), sig.begin(), sig.end());
    }
    return b;
}

inline std::vector<uint8_t> frame_v1(uint8_t sysid, uint8_t compid, uint8_t msgid,
                                     const std::vector<uint8_t> &payload, uint8_t crc_extra,
                                     uint8_t seq = 17) {
    std::vector<uint8_t> b = {0xFE, static_cast<uint8_t>(payload.size()), seq, sysid, compid, msgid};
    b.insert(b.end(), payload.begin(), payload.end());
    const uint16_t crc = x25(b, 1, crc_extra);
    b.push_back(static_cast<uint8_t>(crc));
    b.push_back(static_cast<uint8_t>(crc >> 8));
    return b;
}

// COMMAND_LONG (76, crc_extra 152): command 300, target system/component, confirmation 1
inline std::vector<uint8_t> command_payload(uint32_t target, uint8_t component = 190) {
    std::vector<uint8_t> p(33, 0);
    p[28] = 44;
    p[29] = 1;
    p[30] = target > 255 ? 255 : static_cast<uint8_t>(target);
    p[31] = component;
    p[32] = 1;  // keeps the full payload after zero trimming
    return p;
}

inline std::vector<uint8_t> concat(std::initializer_list<std::vector<uint8_t>> parts) {
    std::vector<uint8_t> out;
    for (const auto &p : parts)
        out.insert(out.end(), p.begin(), p.end());
    return out;
}

}  // namespace wire
