#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "protocol.hpp"

namespace mpcore {

// MAVLink FTP (FILE_TRANSFER_PROTOCOL) as Mission Planner's MAVFtp runs it, without I/O: one
// operation at a time, frames in, messages out, tick for the timers.
//  - get_file (MAVFtp.GetFile): ResetSessions, OpenFileRO, then BurstReadFile (or ReadFile)
//    from offset 0; a burst continues where the furthest data ended, and once the end of the file
//    is seen the holes are read one ReadFile at a time. A short read or an EOF NAK ends the file.
//  - put_file (MAVFtp.UploadFile): ResetSessions, CreateFile, WriteFile in 80-byte chunks (the
//    first alone, then up to 5 outstanding, lowest offsets first), ResetSessions.
//  - list (kCmdListDirectory): ListDirectoryWithTime from offset 0, each reply's entries counted
//    into the next offset, until a reply without entries or an EOF NAK; a vehicle that NAKs the
//    timed listing (unknown command or fail, on its first reply) is listed plainly from then on.
//  - command: one request and its reply (RemoveFile, RemoveDirectory, CreateDirectory, Rename,
//    TruncateFile, CalcFileCRC32, ResetSessions, TerminateSession).
// Timeouts as MAVFtp's RetryTimeout: a request is sent up to N times, each followed by a wait;
// ResetSessions 5 x 1 s, OpenFileRO 5 x 2 s, a listing 5 x 1 s, CalcFileCRC32 3 x 30 s,
// everything else 30 x 1 s, and data that arrives restarts the count. A NAK for the request (its
// opcode and, but for a burst, its seq) ends the operation, except EOF (the end of a file or a
// listing), NoSessionsAvailable (the sessions are reset and the request sent again) and, for
// CreateDirectory, FailFileExists (done). A NAK of the ResetSessions around a transfer is ignored,
// as is its timing out. Replies must be from the vehicle and for us (IsTargetedTo).
// Unlike MAVFtp: replies and NAKs must answer the request, an upload is done once every chunk is
// acknowledged (MAVFtp stops at the last chunk's), a hole is read even at the end of a file, the
// session the vehicle opened is used (MAVFtp always uses 0, the only one ArduPilot has), and the
// sessions are reset without waiting inside a reply (MAVFtp blocks its packet handler).
class Ftp {
public:
    using SendFn = std::function<void(uint32_t msgid, const uint8_t *payload, std::size_t len)>;

    enum class State { Idle, Running, Done, Failed };

    enum Opcode : uint8_t {
        kNone = 0,
        kTerminateSession = 1,
        kResetSessions = 2,
        kListDirectory = 3,
        kOpenFileRO = 4,
        kReadFile = 5,
        kCreateFile = 6,
        kWriteFile = 7,
        kRemoveFile = 8,
        kCreateDirectory = 9,
        kRemoveDirectory = 10,
        kOpenFileWO = 11,
        kTruncateFile = 12,
        kRename = 13,
        kCalcFileCRC32 = 14,
        kBurstReadFile = 15,
        kListDirectoryWithTime = 16,
        kAck = 128,
        kNak = 129,
    };

    enum Nak : uint8_t {
        kErrNone = 0,
        kErrFail = 1,
        kErrFailErrno = 2,
        kErrInvalidDataSize = 3,
        kErrInvalidSession = 4,
        kErrNoSessionsAvailable = 5,
        kErrEOF = 6,
        kErrUnknownCommand = 7,
        kErrFailFileExists = 8,
        kErrFailFileProtected = 9,
        kErrFileNotFound = 10,
    };

    static constexpr std::size_t kHeaderLen = 12;
    static constexpr std::size_t kMaxData = 239;
    static constexpr std::size_t kWriteChunk = 80;
    static constexpr std::size_t kWriteWindow = 5;
    static constexpr int64_t kSecond = 1'000'000;

    struct Entry {
        char kind = 'F';  // 'F' file, 'D' directory, 'S' an entry the vehicle skipped
        std::string name;
        uint64_t size = 0;
        uint32_t mtime = 0;  // seconds since 1970 (UTC), 0 if not known
    };

    Ftp(uint32_t sysid, uint8_t compid, uint32_t gcs_sysid, uint8_t gcs_compid, SendFn send);

    // Each starts an operation; false (and nothing sent) while one is running or for a bad
    // argument (a path longer than kMaxData).
    bool get_file(const std::string &path, uint8_t read_size, bool burst, int64_t now_us);
    bool put_file(const std::string &path, std::vector<uint8_t> data, int64_t now_us);
    bool list(const std::string &dir, int64_t now_us);
    bool command(uint8_t opcode, const std::string &path, const std::string &path2, int64_t now_us);
    // stops the operation: Failed, without a NAK
    void cancel();

    void on_frame(const uint8_t *frame, std::size_t len, int64_t now_us);
    State tick(int64_t now_us);

    State state() const { return state_; }
    // once Failed: the NAK that ended it (kErrNone: timed out or cancelled), its errno and the
    // request opcode it answered
    uint8_t nak() const { return nak_; }
    uint8_t nak_errno() const { return nak_errno_; }
    uint8_t nak_opcode() const { return nak_opcode_; }
    // the request being sent (the last one once finished)
    uint8_t opcode() const { return opcode_; }
    // a command's reply value: CalcFileCRC32's CRC
    uint32_t value() const { return value_; }
    // get_file: the size the vehicle opened (-1 before), cut to where the file ended
    int64_t file_size() const { return file_size_; }
    // get_file: bytes received; put_file: bytes acknowledged
    uint64_t progress() const;
    // get_file: the file once done
    const std::vector<uint8_t> &data() const { return data_; }
    // list: the entries so far
    const std::vector<Entry> &entries() const { return entries_; }

    // the seq of the next request, shared with a caller that sends FTP requests itself
    uint16_t next_seq() const { return next_seq_; }
    void set_next_seq(uint16_t seq) { next_seq_ = seq; }
    uint32_t requests_sent() const { return requests_sent_; }
    // whether listings are plain (the vehicle NAKed a timed one), shared with a caller that keeps it
    bool plain_listing() const { return timed_unsupported_; }
    void set_plain_listing(bool plain) { timed_unsupported_ = plain; }

private:
    enum class Phase { None, ResetBefore, Open, Read, Create, Write, ResetAfter, List, Command };

    struct Reply {
        uint16_t seq;
        uint8_t session;
        uint8_t opcode;
        uint8_t size;
        uint8_t req_opcode;
        uint8_t burst_complete;
        uint32_t offset;
        const uint8_t *data;  // size bytes, at most kMaxData
    };

    bool begin(Phase first);
    void request(uint8_t opcode, uint32_t offset, uint8_t size, const uint8_t *data, std::size_t data_len,
                 int64_t now_us);
    void transmit(const std::vector<uint8_t> &payload);
    void arm(int64_t now_us, int64_t timeout_us, int sends);
    void reset_sessions(int64_t now_us);
    void side_reset();
    void fail(uint8_t nak, uint8_t err, uint8_t opcode);
    void see_seq(uint16_t reply_seq);
    bool answers(const Reply &r) const;

    void after_reset(int64_t now_us);
    void on_read(const Reply &r, int64_t now_us);
    void read_next(uint32_t offset, int64_t now_us);
    void finish_read();
    void on_write_ack(const Reply &r, int64_t now_us);
    void send_chunk(uint32_t offset, int64_t now_us);
    void on_list(const Reply &r, int64_t now_us);
    void list_from(uint32_t offset, int64_t now_us);

    uint64_t have() const;
    uint32_t prefix_end() const;
    bool has_gap() const;
    void add_chunk(uint32_t from, uint32_t to);

    uint32_t sysid_;
    uint8_t compid_;
    uint32_t gcs_sysid_;
    uint8_t gcs_compid_;
    SendFn send_;

    State state_ = State::Idle;
    Phase phase_ = Phase::None;
    uint16_t next_seq_ = 0;
    uint32_t requests_sent_ = 0;

    // the request in flight
    std::vector<uint8_t> payload_;  // the FTP payload, header and data
    uint8_t opcode_ = kNone;
    uint16_t sent_seq_ = 0;
    int64_t sent_us_ = 0;
    int64_t timeout_us_ = 0;
    int resends_left_ = 0;
    int sends_ = 0;

    uint8_t nak_ = kErrNone;
    uint8_t nak_errno_ = 0;
    uint8_t nak_opcode_ = kNone;
    uint32_t value_ = 0;

    // the operation
    std::string path_, path2_;
    uint8_t command_ = kNone;
    uint8_t session_ = 0;

    // get_file
    uint8_t read_size_ = 0;
    bool burst_ = true;
    uint8_t read_op_ = kBurstReadFile;
    int64_t file_size_ = -1;
    uint32_t burst_offset_ = 0;
    std::map<uint32_t, uint32_t> chunks_;  // [from, to), joined
    std::vector<uint8_t> buffer_;
    std::vector<uint8_t> data_;

    // put_file
    std::vector<uint8_t> upload_;
    std::set<uint32_t> pending_;
    uint64_t acked_ = 0;

    // list
    bool timed_unsupported_ = false;
    uint8_t list_op_ = kListDirectoryWithTime;
    std::vector<Entry> entries_;
};

}  // namespace mpcore
