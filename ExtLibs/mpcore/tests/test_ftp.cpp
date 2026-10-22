#include <cstring>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "ftp.hpp"
#include "protocol.hpp"
#include "test.hpp"

using namespace mpcore;

namespace {

constexpr int64_t kMs = 1000;
constexpr uint32_t kFtpMsg = 110;
constexpr std::size_t kPayload = 251;

std::vector<uint8_t> encode_frame(const std::vector<uint8_t> &payload, uint32_t sysid, uint8_t compid) {
    EncodeRequest req;
    req.msgid = kFtpMsg;
    req.payload = payload.data();
    req.payload_len = payload.size();
    req.src_sysid = sysid;
    req.src_compid = compid;
    std::vector<uint8_t> out(kMaxFrameLen);
    const int n = encode(req, out.data(), out.size());
    CHECK(n > 0);
    out.resize(n > 0 ? static_cast<std::size_t>(n) : 0);
    return out;
}

uint32_t crc32(const std::vector<uint8_t> &data) {
    uint32_t crc = 0;
    for (uint8_t b : data) {
        crc ^= b;
        for (int i = 0; i < 8; i++)
            crc = (crc >> 1) ^ ((crc & 1) != 0 ? 0xEDB88320u : 0);
    }
    return crc;
}

std::vector<uint8_t> pattern(std::size_t n, uint8_t salt = 7) {
    std::vector<uint8_t> v(n);
    for (std::size_t i = 0; i < n; i++)
        v[i] = static_cast<uint8_t>((i * 31 + salt) ^ (i >> 8));
    return v;
}

// A vehicle's FTP server, as ArduPilot's GCS_FTP: one session, burst reads of burst_len packets,
// a request with the seq of the last one answered gets that answer again.
struct Vehicle {
    struct Req {
        uint16_t seq;
        uint8_t session, opcode, size, req_opcode, burst;
        uint32_t offset;
        std::vector<uint8_t> data;
    };

    std::map<std::string, std::vector<uint8_t>> files;
    std::set<std::string> dirs{"/"};
    std::map<std::string, uint32_t> mtimes;
    bool timed_list = true;
    bool not_found_code = false;   // missing files NAK FileNotFound, else FailErrno ENOENT
    bool exists_code = true;       // mkdir of a directory there NAKs FailFileExists, else errno EEXIST
    int64_t open_extra = 0;        // OpenFileRO reports this many bytes more than the file has
    int no_sessions = 0;           // opens to NAK with NoSessionsAvailable
    int burst_len = 10;
    bool silent = false;
    uint32_t reply_sysid = 1;
    uint8_t reply_to_sysid = 255, reply_to_compid = 190;
    std::function<bool(const Req &)> drop_in;       // lose a request
    std::function<bool(const Req &, int)> drop_out;  // lose a reply (the reply, its number)

    bool open = false, writing = false;
    std::string open_path;
    std::vector<uint8_t> last_reply;
    uint16_t last_seq = 0;
    int replies = 0;
    std::vector<std::vector<uint8_t>> outbox;
    std::vector<uint8_t> opcodes;  // requests received, in order
    std::vector<Req> requests;

    static Req parse(const uint8_t *p) {
        Req r{static_cast<uint16_t>(p[0] | (p[1] << 8)), p[2], p[3], p[4], p[5], p[6],
              static_cast<uint32_t>(p[8] | (p[9] << 8) | (p[10] << 16) | (static_cast<uint32_t>(p[11]) << 24)),
              {}};
        r.data.assign(p + 12, p + 12 + std::min<std::size_t>(r.size, 239));
        return r;
    }

    std::string text(const Req &r) const { return std::string(r.data.begin(), r.data.end()); }

    void send(std::vector<uint8_t> ftp) {
        Req r = parse(ftp.data());
        last_reply = ftp;
        last_seq = r.seq;
        const int n = replies++;
        if (drop_out && drop_out(r, n))
            return;
        std::vector<uint8_t> msg(3 + kPayload, 0);
        msg[1] = reply_to_sysid;
        msg[2] = reply_to_compid;
        std::memcpy(&msg[3], ftp.data(), kPayload);
        outbox.push_back(encode_frame(msg, reply_sysid, 1));
    }

    std::vector<uint8_t> reply_to(const Req &q, uint16_t seq, uint8_t opcode, const std::vector<uint8_t> &data = {},
                                  uint32_t offset = 0) const {
        std::vector<uint8_t> f(kPayload, 0);
        f[0] = static_cast<uint8_t>(seq);
        f[1] = static_cast<uint8_t>(seq >> 8);
        f[2] = q.session;
        f[3] = opcode;
        f[4] = static_cast<uint8_t>(data.size());
        f[5] = q.opcode;
        for (int i = 0; i < 4; i++)
            f[8 + i] = static_cast<uint8_t>(offset >> (8 * i));
        std::memcpy(&f[12], data.data(), data.size());
        return f;
    }

    void ack(const Req &q, const std::vector<uint8_t> &data = {}, uint32_t offset = 0) {
        send(reply_to(q, static_cast<uint16_t>(q.seq + 1), 128, data, offset));
    }

    void nak(const Req &q, uint8_t code, uint8_t err = 0) {
        std::vector<uint8_t> d{code};
        if (code == 2)
            d.push_back(err);
        send(reply_to(q, static_cast<uint16_t>(q.seq + 1), 129, d));
    }

    void missing(const Req &q) {
        if (not_found_code)
            nak(q, 10);
        else
            nak(q, 2, 2);  // ENOENT
    }

    static std::string parent(const std::string &path) {
        const auto slash = path.rfind('/');
        return slash == 0 || slash == std::string::npos ? "/" : path.substr(0, slash);
    }

    static std::string leaf(const std::string &path) { return path.substr(path.rfind('/') + 1); }

    void receive(const std::vector<uint8_t> &msg) {
        CHECK_EQ(msg.size(), 3 + kPayload);
        CHECK_EQ(msg[1], 1);
        CHECK_EQ(msg[2], 1);
        const Req q = parse(msg.data() + 3);
        requests.push_back(q);
        opcodes.push_back(q.opcode);
        if (silent || (drop_in && drop_in(q)))
            return;
        if (!last_reply.empty() && static_cast<uint16_t>(q.seq + 1) == last_seq) {
            std::vector<uint8_t> again = last_reply;
            send(again);
            return;
        }
        handle(q);
    }

    void handle(const Req &q) {
        switch (q.opcode) {
        case 1:  // TerminateSession
        case 2:  // ResetSessions
            open = false;
            ack(q);
            return;
        case 4: {  // OpenFileRO
            if (no_sessions > 0 || open) {
                no_sessions = no_sessions > 0 ? no_sessions - 1 : 0;
                nak(q, 5);
                return;
            }
            const auto it = files.find(text(q));
            if (it == files.end()) {
                missing(q);
                return;
            }
            open = true;
            writing = false;
            open_path = it->first;
            const auto size = static_cast<uint32_t>(it->second.size() + open_extra);
            ack(q, {static_cast<uint8_t>(size), static_cast<uint8_t>(size >> 8), static_cast<uint8_t>(size >> 16),
                    static_cast<uint8_t>(size >> 24)});
            return;
        }
        case 5:     // ReadFile
        case 15: {  // BurstReadFile
            if (!open || writing) {
                nak(q, 4);
                return;
            }
            const auto &file = files[open_path];
            const int packets = q.opcode == 15 ? burst_len : 1;
            auto seq = static_cast<uint16_t>(q.seq + 1);
            for (int i = 0; i < packets; i++, seq++) {
                const std::size_t off = q.offset + static_cast<std::size_t>(i) * q.size;
                if (off >= file.size()) {
                    std::vector<uint8_t> f = reply_to(q, seq, 129, {6});
                    send(f);
                    return;
                }
                const std::size_t n = std::min<std::size_t>(q.size, file.size() - off);
                std::vector<uint8_t> chunk(file.begin() + static_cast<std::ptrdiff_t>(off),
                                           file.begin() + static_cast<std::ptrdiff_t>(off + n));
                std::vector<uint8_t> f = reply_to(q, seq, 128, chunk, static_cast<uint32_t>(off));
                f[6] = i == packets - 1 ? 1 : 0;
                send(f);
            }
            return;
        }
        case 6:  // CreateFile
            if (open) {
                nak(q, 5);
                return;
            }
            files[text(q)] = {};
            open = true;
            writing = true;
            open_path = text(q);
            ack(q);
            return;
        case 7: {  // WriteFile
            if (!open || !writing) {
                nak(q, 4);
                return;
            }
            auto &file = files[open_path];
            if (file.size() < q.offset + q.data.size())
                file.resize(q.offset + q.data.size());
            std::memcpy(file.data() + q.offset, q.data.data(), q.data.size());
            ack(q, {}, q.offset);
            return;
        }
        case 3:     // ListDirectory
        case 16: {  // ListDirectoryWithTime
            if (q.opcode == 16 && !timed_list) {
                nak(q, 7);
                return;
            }
            const std::string dir = text(q);
            if (dirs.count(dir) == 0) {
                missing(q);
                return;
            }
            std::vector<std::string> entries;
            for (const auto &d : dirs)
                if (d != "/" && parent(d) == dir)
                    entries.push_back("D" + leaf(d) +
                                      (q.opcode == 16 ? "\t0\t" + std::to_string(mtimes[d]) : std::string()));
            for (const auto &f : files)
                if (parent(f.first) == dir)
                    entries.push_back("F" + leaf(f.first) + "\t" + std::to_string(f.second.size()) +
                                      (q.opcode == 16 ? "\t" + std::to_string(mtimes[f.first]) : std::string()));
            if (q.offset >= entries.size()) {
                nak(q, 6);
                return;
            }
            std::vector<uint8_t> data;
            for (std::size_t i = q.offset; i < entries.size(); i++) {
                if (data.size() + entries[i].size() + 1 > 239)
                    break;
                data.insert(data.end(), entries[i].begin(), entries[i].end());
                data.push_back(0);
            }
            ack(q, data, q.offset);
            return;
        }
        case 8:  // RemoveFile
            if (files.erase(text(q)) == 0)
                missing(q);
            else
                ack(q);
            return;
        case 9:  // CreateDirectory
            if (dirs.count(text(q)) != 0) {
                if (exists_code)
                    nak(q, 8);
                else
                    nak(q, 2, 17);  // EEXIST
                return;
            }
            dirs.insert(text(q));
            ack(q);
            return;
        case 10:  // RemoveDirectory
            if (dirs.erase(text(q)) == 0)
                missing(q);
            else
                ack(q);
            return;
        case 12:  // TruncateFile
            if (files.count(text(q)) == 0) {
                missing(q);
                return;
            }
            files[text(q)].resize(q.offset);
            ack(q);
            return;
        case 13: {  // Rename
            const std::string all = text(q);
            const auto nul = all.find('\0');
            const std::string from = all.substr(0, nul), to = nul == std::string::npos ? "" : all.substr(nul + 1);
            if (files.count(from) == 0) {
                missing(q);
                return;
            }
            files[to] = files[from];
            files.erase(from);
            ack(q);
            return;
        }
        case 14: {  // CalcFileCRC32
            const auto it = files.find(text(q));
            if (it == files.end()) {
                missing(q);
                return;
            }
            const uint32_t c = crc32(it->second);
            ack(q, {static_cast<uint8_t>(c), static_cast<uint8_t>(c >> 8), static_cast<uint8_t>(c >> 16),
                    static_cast<uint8_t>(c >> 24)});
            return;
        }
        default:
            nak(q, 7);
        }
    }

    int count(uint8_t opcode) const {
        int n = 0;
        for (uint8_t o : opcodes)
            n += o == opcode ? 1 : 0;
        return n;
    }
};

struct Harness {
    Vehicle v;
    std::vector<std::vector<uint8_t>> to_vehicle;
    int64_t now = 0;
    Ftp ftp{1, 1, 255, 190, [this](uint32_t msgid, const uint8_t *p, std::size_t n) {
                CHECK_EQ(msgid, kFtpMsg);
                to_vehicle.emplace_back(p, p + n);
            }};

    void deliver() {
        for (int rounds = 0; rounds < 100000 && (!to_vehicle.empty() || !v.outbox.empty()); rounds++) {
            auto requests = std::move(to_vehicle);
            to_vehicle.clear();
            for (const auto &r : requests)
                v.receive(r);
            auto replies = std::move(v.outbox);
            v.outbox.clear();
            for (const auto &f : replies)
                ftp.on_frame(f.data(), f.size(), now);
        }
    }

    Ftp::State run(int64_t limit_us = 300'000 * kMs) {
        const int64_t until = now + limit_us;
        while (now < until) {
            deliver();
            const auto s = ftp.tick(now);
            if (s != Ftp::State::Running)
                return s;
            now += 10 * kMs;
        }
        return ftp.state();
    }
};

}  // namespace

TEST(ftp_get_file_burst) {
    Harness h;
    const auto content = pattern(5000);
    h.v.files["/APM/log.bin"] = content;
    CHECK(h.ftp.get_file("/APM/log.bin", 110, true, h.now));
    CHECK(!h.ftp.get_file("/APM/log.bin", 110, true, h.now));  // one at a time
    CHECK(h.run() == Ftp::State::Done);
    CHECK(h.ftp.data() == content);
    CHECK_EQ(h.ftp.file_size(), 5000);
    CHECK_EQ(h.v.opcodes[0], Ftp::kResetSessions);
    CHECK_EQ(h.v.opcodes[1], Ftp::kOpenFileRO);
    CHECK_EQ(h.v.count(Ftp::kReadFile), 0);
    // 46 packets of 110 bytes, 10 to a burst
    CHECK_EQ(h.v.count(Ftp::kBurstReadFile), 5);
    CHECK_EQ(h.ftp.progress(), 5000u);
}

TEST(ftp_get_file_burst_fills_holes) {
    Harness h;
    const auto content = pattern(3000);
    h.v.files["/f"] = content;
    // the first pass loses every seventh data packet
    std::set<uint32_t> lost;
    h.v.drop_out = [&lost](const Vehicle::Req &r, int) {
        if (r.opcode == 128 && r.req_opcode == Ftp::kBurstReadFile && (r.offset / 100) % 7 == 3 &&
            lost.insert(r.offset).second)
            return true;
        return false;
    };
    CHECK(h.ftp.get_file("/f", 100, true, h.now));
    CHECK(h.run() == Ftp::State::Done);
    CHECK(h.ftp.data() == content);
    CHECK_EQ(h.v.count(Ftp::kReadFile), static_cast<int>(lost.size()));
    CHECK(h.now < 1000 * kMs);  // no waiting for timeouts
}

TEST(ftp_get_file_lost_burst_end) {
    Harness h;
    const auto content = pattern(2500);
    h.v.files["/f"] = content;
    // the burst-complete packet of the first burst is lost: the request goes again on the timer
    bool once = true;
    h.v.drop_out = [&once](const Vehicle::Req &r, int) {
        if (once && r.opcode == 128 && r.burst == 1) {
            once = false;
            return true;
        }
        return false;
    };
    CHECK(h.ftp.get_file("/f", 100, true, h.now));
    CHECK(h.run() == Ftp::State::Done);
    CHECK(h.ftp.data() == content);
    CHECK(h.now >= 1000 * kMs);
}

TEST(ftp_get_file_plain_reads_with_loss) {
    Harness h;
    const auto content = pattern(1234);
    h.v.files["/f"] = content;
    h.v.drop_out = [](const Vehicle::Req &r, int n) { return r.req_opcode == Ftp::kReadFile && n % 4 == 2; };
    CHECK(h.ftp.get_file("/f", 80, false, h.now));
    CHECK(h.run() == Ftp::State::Done);
    CHECK(h.ftp.data() == content);
    CHECK_EQ(h.v.count(Ftp::kBurstReadFile), 0);
}

TEST(ftp_get_file_shorter_than_opened) {
    for (bool burst : {true, false}) {
        Harness h;
        const auto content = pattern(990);  // a multiple of the read size: no short read
        h.v.files["/@PARAM/param.pck"] = content;
        h.v.open_extra = 500;
        CHECK(h.ftp.get_file("/@PARAM/param.pck", 110, burst, h.now));
        CHECK(h.run() == Ftp::State::Done);
        CHECK(h.ftp.data() == content);
        CHECK_EQ(h.ftp.file_size(), 990);
    }
}

TEST(ftp_get_file_empty_and_exact) {
    {
        Harness h;
        h.v.files["/empty"] = {};
        CHECK(h.ftp.get_file("/empty", 110, true, h.now));
        CHECK(h.run() == Ftp::State::Done);
        CHECK(h.ftp.data().empty());
        CHECK_EQ(h.ftp.file_size(), 0);
    }
    {
        Harness h;
        const auto content = pattern(1100);
        h.v.files["/exact"] = content;
        CHECK(h.ftp.get_file("/exact", 110, true, h.now));
        CHECK(h.run() == Ftp::State::Done);
        CHECK(h.ftp.data() == content);
    }
}

TEST(ftp_get_file_not_found) {
    for (bool code : {true, false}) {
        Harness h;
        h.v.not_found_code = code;
        CHECK(h.ftp.get_file("/nope", 110, true, h.now));
        CHECK(h.run() == Ftp::State::Failed);
        CHECK_EQ(h.ftp.nak_opcode(), Ftp::kOpenFileRO);
        if (code) {
            CHECK_EQ(h.ftp.nak(), Ftp::kErrFileNotFound);
        } else {
            CHECK_EQ(h.ftp.nak(), Ftp::kErrFailErrno);
            CHECK_EQ(h.ftp.nak_errno(), 2);
        }
        CHECK(h.now < 100 * kMs);
    }
}

TEST(ftp_get_file_no_sessions) {
    Harness h;
    const auto content = pattern(700);
    h.v.files["/f"] = content;
    h.v.no_sessions = 1;
    CHECK(h.ftp.get_file("/f", 110, true, h.now));
    CHECK(h.run() == Ftp::State::Done);
    CHECK(h.ftp.data() == content);
    // reset, open (NAK), reset, open again on the timer (2 s)
    CHECK_EQ(h.v.count(Ftp::kResetSessions), 2);
    CHECK_EQ(h.v.count(Ftp::kOpenFileRO), 2);
}

TEST(ftp_get_file_silent_vehicle) {
    Harness h;
    h.v.silent = true;
    CHECK(h.ftp.get_file("/f", 110, true, h.now));
    CHECK(h.run() == Ftp::State::Failed);
    CHECK_EQ(h.ftp.nak(), Ftp::kErrNone);
    CHECK_EQ(h.ftp.nak_opcode(), Ftp::kOpenFileRO);
    // ResetSessions 5 x 1 s (ignored), then OpenFileRO 5 x 2 s
    CHECK_EQ(h.v.count(Ftp::kResetSessions), 5);
    CHECK_EQ(h.v.count(Ftp::kOpenFileRO), 5);
    CHECK(h.now >= 15'000 * kMs && h.now < 15'100 * kMs);
}

TEST(ftp_get_file_data_stops) {
    Harness h;
    h.v.files["/f"] = pattern(5000);
    h.v.drop_out = [](const Vehicle::Req &r, int) { return r.opcode == 128 && r.offset >= 2000; };
    CHECK(h.ftp.get_file("/f", 100, true, h.now));
    CHECK(h.run() == Ftp::State::Failed);
    CHECK_EQ(h.ftp.nak(), Ftp::kErrNone);
    CHECK_EQ(h.ftp.progress(), 2000u);
    CHECK(h.ftp.data().empty());
}

TEST(ftp_get_file_errno_while_reading) {
    Harness h;
    h.v.files["/f"] = pattern(500);
    h.v.drop_in = [&h](const Vehicle::Req &q) {
        if (q.opcode == Ftp::kBurstReadFile) {
            h.v.nak(q, 2, 5);  // EIO
            return true;
        }
        return false;
    };
    CHECK(h.ftp.get_file("/f", 100, true, h.now));
    CHECK(h.run() == Ftp::State::Failed);
    CHECK_EQ(h.ftp.nak(), Ftp::kErrFailErrno);
    CHECK_EQ(h.ftp.nak_errno(), 5);
    CHECK_EQ(h.ftp.nak_opcode(), Ftp::kBurstReadFile);
}

TEST(ftp_ignores_others) {
    Harness h;
    h.v.files["/f"] = pattern(300);
    h.v.reply_to_sysid = 7;  // for another GCS
    CHECK(h.ftp.get_file("/f", 100, true, h.now));
    h.run(500 * kMs);
    CHECK(h.ftp.state() == Ftp::State::Running);
    h.v.reply_to_sysid = 255;
    h.v.reply_sysid = 2;  // from another vehicle
    h.run(500 * kMs);
    CHECK(h.ftp.state() == Ftp::State::Running);
    h.v.reply_sysid = 1;
    h.v.reply_to_sysid = 0;  // broadcast
    CHECK(h.run() == Ftp::State::Done);
    CHECK(h.ftp.data() == h.v.files["/f"]);
}

TEST(ftp_seq_follows_bursts) {
    Harness h;
    h.v.files["/f"] = pattern(2000);
    h.ftp.set_next_seq(65530);  // wraps
    CHECK(h.ftp.get_file("/f", 100, true, h.now));
    CHECK(h.run() == Ftp::State::Done);
    // every request is new to the vehicle: none answered from its last reply
    const auto &reqs = h.v.requests;
    for (std::size_t i = 1; i < reqs.size(); i++)
        CHECK(static_cast<int16_t>(static_cast<uint16_t>(reqs[i].seq - reqs[i - 1].seq)) > 0);
    CHECK_EQ(h.v.replies, 2 + 20);  // reset, open, two bursts of 10
    CHECK(static_cast<int16_t>(static_cast<uint16_t>(h.ftp.next_seq() - h.v.last_seq)) > 0);
}

TEST(ftp_put_file) {
    Harness h;
    const auto content = pattern(1000, 3);
    CHECK(h.ftp.put_file("/APM/scripts/a.lua", content, h.now));
    CHECK(h.run() == Ftp::State::Done);
    CHECK(h.v.files["/APM/scripts/a.lua"] == content);
    CHECK(!h.v.open);  // the sessions reset at the end
    CHECK_EQ(h.v.opcodes.front(), Ftp::kResetSessions);
    CHECK_EQ(h.v.opcodes[1], Ftp::kCreateFile);
    CHECK_EQ(h.v.opcodes.back(), Ftp::kResetSessions);
    CHECK_EQ(h.v.count(Ftp::kWriteFile), 13);  // 1000 bytes in 80-byte chunks
    CHECK_EQ(h.ftp.progress(), 1000u);
}

TEST(ftp_put_file_with_loss) {
    Harness h;
    const auto content = pattern(2000, 9);
    int writes = 0;
    h.v.drop_in = [&writes](const Vehicle::Req &q) { return q.opcode == Ftp::kWriteFile && writes++ % 6 == 4; };
    h.v.drop_out = [](const Vehicle::Req &r, int n) { return r.req_opcode == Ftp::kWriteFile && n % 9 == 5; };
    CHECK(h.ftp.put_file("/f", content, h.now));
    CHECK(h.run() == Ftp::State::Done);
    CHECK(h.v.files["/f"] == content);
}

TEST(ftp_put_file_lost_chunk_before_the_last) {
    // MAVFtp stops at the last chunk's ACK; every chunk must be acknowledged
    Harness h;
    const auto content = pattern(800, 1);
    bool once = true;
    h.v.drop_in = [&once](const Vehicle::Req &q) {
        if (once && q.opcode == Ftp::kWriteFile && q.offset == 640) {
            once = false;
            return true;
        }
        return false;
    };
    CHECK(h.ftp.put_file("/f", content, h.now));
    CHECK(h.run() == Ftp::State::Done);
    CHECK(h.v.files["/f"] == content);
}

TEST(ftp_put_file_empty) {
    Harness h;
    h.v.files["/f"] = pattern(10);
    CHECK(h.ftp.put_file("/f", {}, h.now));
    CHECK(h.run() == Ftp::State::Done);
    CHECK(h.v.files["/f"].empty());
    CHECK_EQ(h.v.count(Ftp::kWriteFile), 0);
}

TEST(ftp_put_file_silent_after_create) {
    Harness h;
    h.v.drop_in = [](const Vehicle::Req &q) { return q.opcode == Ftp::kWriteFile; };
    CHECK(h.ftp.put_file("/f", pattern(100), h.now));
    CHECK(h.run() == Ftp::State::Failed);
    CHECK_EQ(h.ftp.nak_opcode(), Ftp::kWriteFile);
    CHECK_EQ(h.v.count(Ftp::kWriteFile), 30);
}

TEST(ftp_list_timed_and_plain) {
    Harness h;
    h.v.dirs.insert("/APM");
    h.v.dirs.insert("/APM/LOGS");
    h.v.mtimes["/APM/LOGS"] = 1700000000;
    for (int i = 0; i < 30; i++) {
        const std::string name = "/APM/file" + std::to_string(i) + ".txt";
        h.v.files[name] = pattern(static_cast<std::size_t>(i * 10));
        h.v.mtimes[name] = 1600000000u + static_cast<uint32_t>(i);
    }
    CHECK(h.ftp.list("/APM", h.now));
    CHECK(h.run() == Ftp::State::Done);
    const auto &e = h.ftp.entries();
    CHECK_EQ(e.size(), 31u);
    CHECK_EQ(e[0].kind, 'D');
    CHECK_EQ(e[0].name, std::string("LOGS"));
    CHECK_EQ(e[0].mtime, 1700000000u);
    std::size_t files = 0;
    for (const auto &x : e) {
        if (x.kind != 'F')
            continue;
        files++;
        const int i = std::stoi(x.name.substr(4));
        CHECK_EQ(x.size, static_cast<uint64_t>(i * 10));
        CHECK_EQ(x.mtime, 1600000000u + static_cast<uint32_t>(i));
    }
    CHECK_EQ(files, 30u);
    CHECK(h.v.count(Ftp::kListDirectoryWithTime) > 2);  // several pages, then the EOF
    CHECK_EQ(h.v.count(Ftp::kListDirectory), 0);

    // a vehicle without timed listings: the plain one, from then on straight away
    h.v.timed_list = false;
    h.v.opcodes.clear();
    CHECK(h.ftp.list("/APM", h.now));
    CHECK(h.run() == Ftp::State::Done);
    CHECK_EQ(h.ftp.entries().size(), 31u);
    CHECK_EQ(h.ftp.entries()[0].mtime, 0u);
    CHECK_EQ(h.v.count(Ftp::kListDirectoryWithTime), 1);
    h.v.opcodes.clear();
    CHECK(h.ftp.list("/APM", h.now));
    CHECK(h.run() == Ftp::State::Done);
    CHECK_EQ(h.ftp.entries().size(), 31u);
    CHECK_EQ(h.v.count(Ftp::kListDirectoryWithTime), 0);
    CHECK(h.ftp.plain_listing());

    // kept by the caller across objects
    Harness other;
    other.v.dirs.insert("/x");
    other.ftp.set_plain_listing(true);
    CHECK(other.ftp.list("/x", other.now));
    CHECK(other.run() == Ftp::State::Done);
    CHECK_EQ(other.v.count(Ftp::kListDirectoryWithTime), 0);
    CHECK_EQ(other.v.count(Ftp::kListDirectory), 1);
}

TEST(ftp_list_empty_missing_and_lossy) {
    Harness h;
    h.v.dirs.insert("/empty");
    CHECK(h.ftp.list("/empty", h.now));
    CHECK(h.run() == Ftp::State::Done);
    CHECK(h.ftp.entries().empty());

    CHECK(h.ftp.list("/missing", h.now));
    CHECK(h.run() == Ftp::State::Failed);
    CHECK_EQ(h.ftp.nak(), Ftp::kErrFailErrno);

    h.v.dirs.insert("/d");
    for (int i = 0; i < 20; i++)
        h.v.files["/d/longer_file_name_" + std::to_string(i)] = {};
    h.v.drop_out = [](const Vehicle::Req &r, int n) { return r.opcode == 128 && n % 3 == 1; };
    CHECK(h.ftp.list("/d", h.now));
    CHECK(h.run() == Ftp::State::Done);
    CHECK_EQ(h.ftp.entries().size(), 20u);
}

TEST(ftp_commands) {
    Harness h;
    h.v.files["/a"] = pattern(321);
    CHECK(h.ftp.command(Ftp::kCalcFileCRC32, "/a", "", h.now));
    CHECK(h.run() == Ftp::State::Done);
    CHECK_EQ(h.ftp.value(), crc32(h.v.files["/a"]));

    CHECK(h.ftp.command(Ftp::kRename, "/a", "/b", h.now));
    CHECK(h.run() == Ftp::State::Done);
    CHECK(h.v.files.count("/b") == 1 && h.v.files.count("/a") == 0);

    CHECK(h.ftp.command(Ftp::kRemoveFile, "/a", "", h.now));
    CHECK(h.run() == Ftp::State::Failed);
    CHECK_EQ(h.ftp.nak(), Ftp::kErrFailErrno);
    CHECK_EQ(h.ftp.nak_errno(), 2);
    CHECK_EQ(h.ftp.nak_opcode(), Ftp::kRemoveFile);

    CHECK(h.ftp.command(Ftp::kCreateDirectory, "/dir", "", h.now));
    CHECK(h.run() == Ftp::State::Done);
    CHECK(h.ftp.command(Ftp::kCreateDirectory, "/dir", "", h.now));
    CHECK(h.run() == Ftp::State::Done);  // FailFileExists: there already
    h.v.exists_code = false;
    CHECK(h.ftp.command(Ftp::kCreateDirectory, "/dir", "", h.now));
    CHECK(h.run() == Ftp::State::Failed);  // errno EEXIST
    CHECK_EQ(h.ftp.nak_errno(), 17);

    CHECK(h.ftp.command(Ftp::kRemoveDirectory, "/dir", "", h.now));
    CHECK(h.run() == Ftp::State::Done);
    CHECK(h.v.dirs.count("/dir") == 0);

    CHECK(h.ftp.command(Ftp::kTruncateFile, "/b", "", h.now));
    CHECK(h.run() == Ftp::State::Done);
    CHECK(h.v.files["/b"].empty());

    CHECK(h.ftp.command(Ftp::kTerminateSession, "", "", h.now));
    CHECK(h.run() == Ftp::State::Done);

    CHECK(!h.ftp.command(Ftp::kOpenFileRO, "/b", "", h.now));  // not a command
    CHECK(!h.ftp.command(Ftp::kRemoveFile, std::string(240, 'x'), "", h.now));
    CHECK(!h.ftp.command(Ftp::kRename, std::string(120, 'x'), std::string(120, 'y'), h.now));
}

TEST(ftp_command_timeouts) {
    Harness h;
    h.v.silent = true;
    CHECK(h.ftp.command(Ftp::kCalcFileCRC32, "/a", "", h.now));
    CHECK(h.run() == Ftp::State::Failed);
    CHECK_EQ(h.v.count(Ftp::kCalcFileCRC32), 3);
    CHECK(h.now >= 90'000 * kMs && h.now < 90'100 * kMs);

    const int64_t start = h.now;
    CHECK(h.ftp.command(Ftp::kRemoveFile, "/a", "", h.now));
    CHECK(h.run() == Ftp::State::Failed);
    CHECK_EQ(h.v.count(Ftp::kRemoveFile), 30);
    CHECK(h.now - start >= 30'000 * kMs && h.now - start < 30'100 * kMs);

    // a lost reply: the request again, answered from the vehicle's last reply
    h.v.silent = false;
    h.v.files["/a"] = pattern(10);
    bool once = true;
    h.v.drop_out = [&once](const Vehicle::Req &, int) {
        const bool drop = once;
        once = false;
        return drop;
    };
    CHECK(h.ftp.command(Ftp::kRemoveFile, "/a", "", h.now));
    CHECK(h.run() == Ftp::State::Done);
    CHECK_EQ(h.v.count(Ftp::kRemoveFile), 32);
}

TEST(ftp_cancel) {
    Harness h;
    h.v.silent = true;
    CHECK(h.ftp.get_file("/f", 100, true, h.now));
    h.run(300 * kMs);
    h.ftp.cancel();
    CHECK(h.ftp.state() == Ftp::State::Failed);
    CHECK_EQ(h.ftp.nak(), Ftp::kErrNone);
    const auto sent = h.ftp.requests_sent();
    CHECK(h.ftp.tick(h.now + 100'000 * kMs) == Ftp::State::Failed);
    CHECK_EQ(h.ftp.requests_sent(), sent);
    h.v.silent = false;
    h.v.files["/f"] = pattern(10);
    CHECK(h.ftp.get_file("/f", 100, true, h.now));  // the next operation
    CHECK(h.run() == Ftp::State::Done);
    CHECK(h.ftp.data() == h.v.files["/f"]);
}
