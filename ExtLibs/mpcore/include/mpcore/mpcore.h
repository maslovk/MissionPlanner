/*
 * mpcore - Mission Planner MAVLink core, C ABI.
 *
 * One mp_link is one MAVLink byte stream (serial port, TCP/UDP socket, log file). The link does
 * no I/O of its own: the caller feeds received bytes with mp_link_feed() and the link hands
 * outgoing bytes to the on_write callback. Transports, threads and timers stay with the caller.
 *
 * Threading: an mp_link is not thread-safe. Call every function for one link from one thread at
 * a time (the caller serialises). Callbacks run synchronously inside mp_link_feed()/mp_link_send()
 * on the calling thread. Pointers passed to callbacks are valid only until the callback returns.
 * Different links are independent and may be used from different threads.
 *
 * ABI: every struct starts with struct_size, which the caller sets to sizeof(struct). Fields are
 * only ever appended, so an older caller keeps working with a newer library.
 */
#ifndef MPCORE_MPCORE_H
#define MPCORE_MPCORE_H

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#  define MPCORE_CALL __cdecl
#  if defined(MPCORE_BUILDING)
#    define MPCORE_API __declspec(dllexport)
#  else
#    define MPCORE_API __declspec(dllimport)
#  endif
#else
#  define MPCORE_CALL
#  if defined(MPCORE_BUILDING)
#    define MPCORE_API __attribute__((visibility("default")))
#  else
#    define MPCORE_API
#  endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Incremented on any incompatible change to this header. */
#define MPCORE_ABI_VERSION 1

/* Return codes. Functions returning int give 0 (or a positive count) on success. */
#define MP_OK                  0
#define MP_ERR_ARGUMENT       -1  /* NULL pointer, bad struct_size or out-of-range value */
#define MP_ERR_UNKNOWN_MSG    -2  /* msgid is not in the compiled dialect */
#define MP_ERR_PAYLOAD_SIZE   -3  /* payload longer than the message's maximum length */
#define MP_ERR_UNREPRESENTABLE -4 /* the frame cannot be encoded (e.g. 32-bit IDs in MAVLink 1) */
#define MP_ERR_NOT_FOUND      -5  /* no such peer */

/* mp_frame.flags */
#define MP_FRAME_MAVLINK2       0x0001u /* MAVLink 2 framing (else MAVLink 1) */
#define MP_FRAME_SIGNED         0x0002u /* frame carries a signature block */
#define MP_FRAME_SIGNATURE_OK   0x0004u /* the signature was verified against a key */
#define MP_FRAME_UNKNOWN_MSGID  0x0008u /* msgid not in the dialect: the CRC could not be checked */
#define MP_FRAME_NOT_FOR_US     0x0010u /* TARGET32 header names another system/component */
#define MP_FRAME_HAS_TARGET     0x0020u /* target_sysid/target_compid are valid */

/* mp_link_send() flags */
#define MP_SEND_FORCE_MAVLINK2  0x0001u /* use MAVLink 2 even if the peer has only sent MAVLink 1 */

/* The longest frame: 17 header bytes, 255 payload, 2 CRC, 13 signature. */
#define MPCORE_MAX_FRAME_LEN    287

typedef struct mp_link mp_link;

typedef struct mp_frame {
    uint32_t struct_size;
    uint32_t flags;          /* MP_FRAME_* */
    uint32_t msgid;
    uint32_t sysid;          /* full 32-bit source system ID */
    uint32_t target_sysid;   /* header TARGET32 target, else the payload target_system field */
    uint8_t compid;
    uint8_t target_compid;
    uint8_t seq;
    uint8_t incompat_flags;
    uint8_t compat_flags;
    uint8_t payload_len;     /* as on the wire: MAVLink 2 payloads may be zero-trimmed */
    uint16_t frame_len;
    const uint8_t *frame;    /* the complete frame as received (header .. signature) */
    const uint8_t *payload;  /* points into frame */
    int64_t rx_time_us;      /* the now_us given to mp_link_feed() */
} mp_frame;

typedef void (MPCORE_CALL *mp_frame_fn)(void *user, const mp_frame *frame);
typedef void (MPCORE_CALL *mp_write_fn)(void *user, const uint8_t *data, size_t len);
/* A line of non-MAVLink text seen on the link (e.g. a bootloader or NMEA), without CR/LF. */
typedef void (MPCORE_CALL *mp_text_fn)(void *user, const char *line, size_t len);

typedef struct mp_link_callbacks {
    uint32_t struct_size;
    void *user;
    mp_frame_fn on_frame;    /* every valid frame, plus unknown-msgid frames (flagged) */
    mp_write_fn on_write;    /* bytes to transmit; required for mp_link_send() */
    mp_text_fn on_text;      /* optional */
} mp_link_callbacks;

typedef struct mp_link_config {
    uint32_t struct_size;
    uint32_t gcs_sysid;      /* our system ID (default 255) */
    uint8_t gcs_compid;      /* our component ID (default 190, MAV_COMP_ID_MISSIONPLANNER) */
    uint8_t ignore_signatures; /* accept signed frames unchecked, e.g. for log playback */
    uint8_t reserved[2];
} mp_link_config;

typedef struct mp_peer_stats {
    uint32_t struct_size;
    uint32_t sysid;
    uint8_t compid;
    uint8_t mavlink2;        /* has sent MAVLink 2; once set it stays set */
    uint8_t signing;         /* outgoing frames to this peer are signed */
    uint8_t last_seq;
    float packets_lost;      /* decays by 0.8 every 5 s, like packets_not_lost */
    float packets_not_lost;
    float sync_lost;         /* number of gaps, not packets */
    float link_quality;      /* 0..100 */
    uint64_t frames;
    int64_t last_rx_us;
} mp_peer_stats;

typedef struct mp_link_stats {
    uint32_t struct_size;
    uint32_t reserved;
    uint64_t bytes_in;
    uint64_t bytes_out;
    uint64_t frames_in;
    uint64_t frames_out;
    uint64_t mavlink1_in;
    uint64_t mavlink2_in;
    uint64_t signed_in;
    uint64_t crc_errors;
    uint64_t unknown_msgid;
    uint64_t parse_errors;   /* frames abandoned before the CRC, e.g. unknown incompat flags */
    uint64_t signature_failures;
    uint64_t text_lines;
} mp_link_stats;

typedef struct mp_msg_info {
    uint32_t struct_size;
    uint32_t msgid;
    const char *name;        /* e.g. "HEARTBEAT"; static storage */
    uint8_t crc_extra;
    uint8_t min_len;         /* MAVLink 1 length, without extensions */
    uint8_t max_len;         /* with extensions */
    uint8_t target_system_ofs;    /* 0xFF if the message has no target_system field */
    uint8_t target_component_ofs; /* 0xFF if the message has no target_component field */
    uint8_t reserved[3];
} mp_msg_info;

/* Everything mp_frame_encode() needs: the caller makes every decision, the core only encodes. */
typedef struct mp_encode_args {
    uint32_t struct_size;
    uint32_t msgid;
    uint32_t src_sysid;
    uint32_t target_sysid;   /* > 255 with a target_system field: payload 255, TARGET32 header */
    uint8_t src_compid;
    uint8_t seq;
    uint8_t mavlink1;        /* 1: MAVLink 1 framing, payload cut to the base length */
    uint8_t sign;            /* 1: append a signature (MAVLink 2 only) */
    uint8_t link_id;         /* signature link ID */
    uint8_t reserved[11];    /* keeps timestamp at offset 32 on every ABI */
    uint64_t timestamp;      /* signature timestamp, 10 us units since 2015-01-01 */
    const uint8_t *key;      /* 32-byte signing key, required when sign is 1 */
} mp_encode_args;

MPCORE_API uint32_t MPCORE_CALL mp_abi_version(void);
MPCORE_API const char *MPCORE_CALL mp_version_string(void);

MPCORE_API int MPCORE_CALL mp_msg_info_get(uint32_t msgid, mp_msg_info *out);

/*
 * Encodes one frame without any link state. payload is the message struct in wire order, up to the
 * message's maximum length (shorter is zero-extended); MAVLink 2 payloads are zero-trimmed. Writes
 * the frame to out (MPCORE_MAX_FRAME_LEN bytes always suffice) and returns its length, or a
 * negative MP_ERR_*.
 */
MPCORE_API int MPCORE_CALL mp_frame_encode(const mp_encode_args *args, const void *payload,
                                           size_t payload_len, uint8_t *out, size_t out_capacity);

MPCORE_API void MPCORE_CALL mp_link_config_init(mp_link_config *config);
/* Returns NULL on bad arguments. callbacks may be NULL and set later. */
MPCORE_API mp_link *MPCORE_CALL mp_link_create(const mp_link_config *config,
                                               const mp_link_callbacks *callbacks);
MPCORE_API void MPCORE_CALL mp_link_destroy(mp_link *link);
MPCORE_API int MPCORE_CALL mp_link_set_callbacks(mp_link *link, const mp_link_callbacks *callbacks);

/* Parses bytes; now_us is a monotonic time in microseconds. Returns the frames delivered. */
MPCORE_API size_t MPCORE_CALL mp_link_feed(mp_link *link, const uint8_t *data, size_t len,
                                           int64_t now_us);
/*
 * Call periodically while no bytes arrive (e.g. each idle pass of the reader loop). A frame
 * start that has had no bytes for 1.2 s is given up on and the bytes after it are scanned
 * again, so a false start in line noise cannot hold back the real frames inside it.
 * Returns the frames delivered.
 */
MPCORE_API size_t MPCORE_CALL mp_link_poll(mp_link *link, int64_t now_us);

/*
 * Encodes and writes one message. payload is the message struct in wire order (as generated),
 * up to the message's maximum length; a shorter payload is zero-extended. (target_sysid,
 * target_compid) name the destination peer: its state picks MAVLink 1 or 2 and signing. If the
 * message has a target_system field and target_sysid > 255, the payload field becomes 255 and a
 * TARGET32 header carries the full ID; otherwise the payload's own target fields are sent as given.
 * Returns the number of bytes written, or a negative MP_ERR_*.
 */
MPCORE_API int MPCORE_CALL mp_link_send(mp_link *link, uint32_t msgid, const void *payload,
                                        size_t payload_len, uint32_t target_sysid,
                                        uint8_t target_compid, uint32_t send_flags);

/* Key ring tried for signed frames from peers without a key yet (Mission Planner's AuthKeys). */
MPCORE_API int MPCORE_CALL mp_link_add_signing_key(mp_link *link, const uint8_t key[32]);
MPCORE_API void MPCORE_CALL mp_link_clear_signing_keys(mp_link *link);
/* Sets (key != NULL) or clears the key used to sign frames to, and verify frames from, a peer. */
MPCORE_API int MPCORE_CALL mp_link_set_peer_signing(mp_link *link, uint32_t sysid, uint8_t compid,
                                                    const uint8_t key[32], uint8_t link_id);

MPCORE_API int MPCORE_CALL mp_link_peer_stats(mp_link *link, uint32_t sysid, uint8_t compid,
                                              mp_peer_stats *out);
MPCORE_API size_t MPCORE_CALL mp_link_peer_count(mp_link *link);
/* Peers in the order they were first heard. */
MPCORE_API int MPCORE_CALL mp_link_peer_at(mp_link *link, size_t index, mp_peer_stats *out);
MPCORE_API int MPCORE_CALL mp_link_stats_get(mp_link *link, mp_link_stats *out);

/*
 * Parameter list download (PARAM_REQUEST_LIST / PARAM_VALUE, with Mission Planner's retries:
 * the whole list again below 75 %, then up to 10 missing indexes a second). Like mp_link it does
 * no I/O: feed it frames, send what it asks for, call tick regularly. One vehicle per download;
 * not thread-safe; callbacks run inside the calls.
 */
typedef struct mp_param_download mp_param_download;

typedef struct mp_param {
    uint32_t struct_size;
    const char *name;        /* NUL-terminated, valid during the callback */
    float value;             /* PARAM_VALUE.param_value as sent */
    uint8_t type;            /* MAV_PARAM_TYPE */
    uint8_t duplicate;       /* 1: this index had already arrived */
    uint16_t index;          /* 65535: a parameter sent on its own, not part of the list */
    uint16_t count;          /* the vehicle's total */
    uint16_t reserved;
} mp_param;

typedef struct mp_param_download_status {
    uint32_t struct_size;
    uint32_t received;       /* distinct indexes below the total */
    uint32_t total;          /* 0 until the vehicle has answered */
    uint32_t full_list_requests;
    uint32_t single_requests;
    uint8_t done;
    uint8_t one_by_one;
    uint8_t reserved[2];
} mp_param_download_status;

/* A message to send to the download's vehicle: msgid and payload in wire order. */
typedef void (MPCORE_CALL *mp_message_fn)(void *user, uint32_t msgid, const uint8_t *payload, size_t len);
typedef void (MPCORE_CALL *mp_param_fn)(void *user, const mp_param *param);

MPCORE_API mp_param_download *MPCORE_CALL mp_param_download_create(uint32_t sysid, uint8_t compid,
                                                                    mp_message_fn send, mp_param_fn on_param,
                                                                    void *user);
MPCORE_API void MPCORE_CALL mp_param_download_destroy(mp_param_download *download);
/* Sends PARAM_REQUEST_LIST. */
MPCORE_API int MPCORE_CALL mp_param_download_start(mp_param_download *download, int64_t now_us);
/* Any received frame; PARAM_VALUE from the vehicle is used, the rest ignored. */
MPCORE_API int MPCORE_CALL mp_param_download_frame(mp_param_download *download, const uint8_t *frame,
                                                   size_t len, int64_t now_us);
/* Retries when due. Returns 1 once every parameter has arrived, else 0 (or a negative MP_ERR_*). */
MPCORE_API int MPCORE_CALL mp_param_download_tick(mp_param_download *download, int64_t now_us);
MPCORE_API int MPCORE_CALL mp_param_download_status_get(mp_param_download *download,
                                                        mp_param_download_status *out);

/*
 * One request and its reply, sent again when the reply is late (700 ms, 3 resends, then a
 * timeout, as Mission Planner's request loops). Feed it frames, send what it asks for, call tick.
 * One per request; not thread-safe; send runs inside the calls.
 */
typedef struct mp_exchange mp_exchange;

/* mp_exchange_tick() results */
#define MP_EXCHANGE_WAITING   0
#define MP_EXCHANGE_DONE      1
#define MP_EXCHANGE_TIMED_OUT 2

/* PARAM_SET, answered by the vehicle's PARAM_VALUE with the same name. value as sent on the wire. */
MPCORE_API mp_exchange *MPCORE_CALL mp_param_set_create(uint32_t sysid, uint8_t compid, const char *name,
                                                         float value, uint8_t type, mp_message_fn send,
                                                         void *user);
/* PARAM_REQUEST_READ by name (index -1) or by index, answered by that PARAM_VALUE. */
MPCORE_API mp_exchange *MPCORE_CALL mp_param_read_create(uint32_t sysid, uint8_t compid, const char *name,
                                                          int16_t index, mp_message_fn send, void *user);
MPCORE_API void MPCORE_CALL mp_exchange_destroy(mp_exchange *exchange);
/* Sends the request. */
MPCORE_API int MPCORE_CALL mp_exchange_start(mp_exchange *exchange, int64_t now_us);
/* Any received frame; the reply from the target is recognised, the rest ignored. */
MPCORE_API int MPCORE_CALL mp_exchange_frame(mp_exchange *exchange, const uint8_t *frame, size_t len,
                                             int64_t now_us);
/* Resends when due. Returns MP_EXCHANGE_* (or a negative MP_ERR_*). */
MPCORE_API int MPCORE_CALL mp_exchange_tick(mp_exchange *exchange, int64_t now_us);
/* Copies the reply frame once done; returns its length, 0 before, or a negative MP_ERR_*. */
MPCORE_API int MPCORE_CALL mp_exchange_reply(mp_exchange *exchange, uint8_t *out, size_t capacity);
/* How many times the request was sent. */
MPCORE_API uint32_t MPCORE_CALL mp_exchange_sends(mp_exchange *exchange);

/*
 * Mission, fence or rally list download (MISSION_REQUEST_LIST, MISSION_COUNT, the items one by one,
 * MISSION_ACK), with Mission Planner's timeouts: 700 ms and 6 resends for the count, 2.5 s and 5
 * per item. Replies must be for gcs_sysid/gcs_compid and of the mission type. Same shape as the
 * other protocol objects: no I/O, not thread-safe, send runs inside the calls.
 */
typedef struct mp_mission_download mp_mission_download;

/* mp_mission_download_tick() results */
#define MP_MISSION_RUNNING 0
#define MP_MISSION_DONE    1
#define MP_MISSION_FAILED  2 /* timed out; mp_mission_download_count() < 0: while counting */

MPCORE_API mp_mission_download *MPCORE_CALL mp_mission_download_create(uint32_t sysid, uint8_t compid,
                                                                        uint32_t gcs_sysid, uint8_t gcs_compid,
                                                                        uint8_t mission_type, uint8_t use_int,
                                                                        mp_message_fn send, void *user);
MPCORE_API void MPCORE_CALL mp_mission_download_destroy(mp_mission_download *download);
MPCORE_API int MPCORE_CALL mp_mission_download_start(mp_mission_download *download, int64_t now_us);
MPCORE_API int MPCORE_CALL mp_mission_download_frame(mp_mission_download *download, const uint8_t *frame,
                                                     size_t len, int64_t now_us);
/* Resends when due. Returns MP_MISSION_* (or a negative MP_ERR_*). */
MPCORE_API int MPCORE_CALL mp_mission_download_tick(mp_mission_download *download, int64_t now_us);
/* The vehicle's count, or -1 before it answered. */
MPCORE_API int MPCORE_CALL mp_mission_download_count(mp_mission_download *download);
/* Items received so far. */
MPCORE_API size_t MPCORE_CALL mp_mission_download_received(mp_mission_download *download);
/* Copies item index (its MISSION_ITEM_INT or MISSION_ITEM frame); returns its length. */
MPCORE_API int MPCORE_CALL mp_mission_download_item(mp_mission_download *download, size_t index, uint8_t *out,
                                                    size_t capacity);

/*
 * Mission, fence or rally list upload: MISSION_COUNT, then each MISSION_REQUEST(_INT) answered with
 * the item it asks for, until the vehicle's MISSION_ACK (its result is the outcome). 700 ms and 3
 * resends for the count, 450 ms and 10 for an item. Add the items (MISSION_ITEM_INT or MISSION_ITEM
 * payloads with their seq, target and mission type set) before mp_mission_upload_start. Ticks
 * return MP_MISSION_*.
 */
typedef struct mp_mission_upload mp_mission_upload;

MPCORE_API mp_mission_upload *MPCORE_CALL mp_mission_upload_create(uint32_t sysid, uint8_t compid,
                                                                    uint32_t gcs_sysid, uint8_t gcs_compid,
                                                                    uint8_t mission_type, mp_message_fn send,
                                                                    void *user);
MPCORE_API void MPCORE_CALL mp_mission_upload_destroy(mp_mission_upload *upload);
MPCORE_API int MPCORE_CALL mp_mission_upload_add(mp_mission_upload *upload, uint32_t msgid, const void *payload,
                                                 size_t len);
MPCORE_API int MPCORE_CALL mp_mission_upload_start(mp_mission_upload *upload, int64_t now_us);
MPCORE_API int MPCORE_CALL mp_mission_upload_frame(mp_mission_upload *upload, const uint8_t *frame, size_t len,
                                                   int64_t now_us);
MPCORE_API int MPCORE_CALL mp_mission_upload_tick(mp_mission_upload *upload, int64_t now_us);
/* The vehicle's MISSION_ACK result (MAV_MISSION_RESULT), or -1 without one. */
MPCORE_API int MPCORE_CALL mp_mission_upload_result(mp_mission_upload *upload);
/* 1 once the vehicle asked for an item or answered the count. */
MPCORE_API int MPCORE_CALL mp_mission_upload_count_accepted(mp_mission_upload *upload);
/* The highest seq the vehicle asked for, -1 before any. */
MPCORE_API int MPCORE_CALL mp_mission_upload_highest_requested(mp_mission_upload *upload);

/*
 * MAVLink FTP (FILE_TRANSFER_PROTOCOL) with Mission Planner's MAVFtp timeouts, one operation at a
 * time: a file download (sessions reset, open, burst or plain reads, the holes read again), a file
 * upload (sessions reset, create, writes, sessions reset), a directory listing (timed, else plain)
 * or a single command. Same shape as the other protocol objects: no I/O, not thread-safe, send
 * runs inside the calls. Paths are UTF-8, at most MP_FTP_MAX_DATA bytes (a rename's two with the
 * NUL between them).
 */
typedef struct mp_ftp mp_ftp;

#define MP_FTP_MAX_DATA 239

/* mp_ftp_tick() results */
#define MP_FTP_RUNNING 0
#define MP_FTP_DONE    1
#define MP_FTP_FAILED  2 /* see mp_ftp_nak(): a NAK, or MP_FTP_ERR_NONE for a timeout or a cancel */

/* FTP request opcodes (the MAVLink FTP specification) */
#define MP_FTP_TERMINATE_SESSION 1
#define MP_FTP_RESET_SESSIONS    2
#define MP_FTP_LIST_DIRECTORY    3
#define MP_FTP_OPEN_FILE_RO      4
#define MP_FTP_READ_FILE         5
#define MP_FTP_CREATE_FILE       6
#define MP_FTP_WRITE_FILE        7
#define MP_FTP_REMOVE_FILE       8
#define MP_FTP_CREATE_DIRECTORY  9
#define MP_FTP_REMOVE_DIRECTORY  10
#define MP_FTP_OPEN_FILE_WO      11
#define MP_FTP_TRUNCATE_FILE     12
#define MP_FTP_RENAME            13
#define MP_FTP_CALC_FILE_CRC32   14
#define MP_FTP_BURST_READ_FILE   15
#define MP_FTP_LIST_DIRECTORY_WITH_TIME 16

/* NAK codes */
#define MP_FTP_ERR_NONE                 0
#define MP_FTP_ERR_FAIL                 1
#define MP_FTP_ERR_FAIL_ERRNO           2
#define MP_FTP_ERR_INVALID_DATA_SIZE    3
#define MP_FTP_ERR_INVALID_SESSION      4
#define MP_FTP_ERR_NO_SESSIONS_AVAILABLE 5
#define MP_FTP_ERR_EOF                  6
#define MP_FTP_ERR_UNKNOWN_COMMAND      7
#define MP_FTP_ERR_FAIL_FILE_EXISTS     8
#define MP_FTP_ERR_FAIL_FILE_PROTECTED  9
#define MP_FTP_ERR_FILE_NOT_FOUND       10

typedef struct mp_ftp_entry {
    uint32_t struct_size; /* set to sizeof(mp_ftp_entry) */
    uint8_t kind;         /* 'F' file, 'D' directory, 'S' an entry the vehicle skipped */
    uint8_t reserved[3];
    uint32_t mtime;       /* seconds since 1970 (UTC), 0 if not known */
    uint32_t reserved2;
    uint64_t size;
} mp_ftp_entry;

MPCORE_API mp_ftp *MPCORE_CALL mp_ftp_create(uint32_t sysid, uint8_t compid, uint32_t gcs_sysid,
                                             uint8_t gcs_compid, mp_message_fn send, void *user);
MPCORE_API void MPCORE_CALL mp_ftp_destroy(mp_ftp *ftp);
/* Starts an operation; MP_ERR_ARGUMENT while one runs or for a bad argument. read_size: bytes per
   read (1..MP_FTP_MAX_DATA); burst: BurstReadFile, else ReadFile. */
MPCORE_API int MPCORE_CALL mp_ftp_get_file(mp_ftp *ftp, const char *path, uint8_t read_size, int burst,
                                           int64_t now_us);
MPCORE_API int MPCORE_CALL mp_ftp_put_file(mp_ftp *ftp, const char *path, const void *data, size_t len,
                                           int64_t now_us);
MPCORE_API int MPCORE_CALL mp_ftp_list(mp_ftp *ftp, const char *dir, int64_t now_us);
/* One of TERMINATE_SESSION, RESET_SESSIONS, REMOVE_FILE, CREATE_DIRECTORY, REMOVE_DIRECTORY,
   TRUNCATE_FILE, RENAME (path to path2) or CALC_FILE_CRC32; path2 may be NULL otherwise. */
MPCORE_API int MPCORE_CALL mp_ftp_command(mp_ftp *ftp, uint8_t opcode, const char *path, const char *path2,
                                          int64_t now_us);
/* Stops the operation (MP_FTP_FAILED, no NAK). */
MPCORE_API int MPCORE_CALL mp_ftp_cancel(mp_ftp *ftp);
MPCORE_API int MPCORE_CALL mp_ftp_frame(mp_ftp *ftp, const uint8_t *frame, size_t len, int64_t now_us);
/* Resends when due. Returns MP_FTP_* (or a negative MP_ERR_*). */
MPCORE_API int MPCORE_CALL mp_ftp_tick(mp_ftp *ftp, int64_t now_us);
/* Once failed: the NAK code (MP_FTP_ERR_*), its errno, and the request opcode it answered. */
MPCORE_API int MPCORE_CALL mp_ftp_nak(mp_ftp *ftp);
MPCORE_API int MPCORE_CALL mp_ftp_nak_errno(mp_ftp *ftp);
MPCORE_API int MPCORE_CALL mp_ftp_nak_opcode(mp_ftp *ftp);
/* The request being sent (the last one once finished). */
MPCORE_API int MPCORE_CALL mp_ftp_opcode(mp_ftp *ftp);
/* CALC_FILE_CRC32's CRC. */
MPCORE_API uint32_t MPCORE_CALL mp_ftp_value(mp_ftp *ftp);
/* A download: the size opened (-1 before), cut to where the file ended. */
MPCORE_API int64_t MPCORE_CALL mp_ftp_file_size(mp_ftp *ftp);
/* A download: bytes received; an upload: bytes acknowledged. */
MPCORE_API uint64_t MPCORE_CALL mp_ftp_progress(mp_ftp *ftp);
/* A finished download: copies up to capacity bytes (out may be NULL); returns the file's length. */
MPCORE_API size_t MPCORE_CALL mp_ftp_data(mp_ftp *ftp, void *out, size_t capacity);
/* A listing's entries so far. */
MPCORE_API size_t MPCORE_CALL mp_ftp_entry_count(mp_ftp *ftp);
MPCORE_API int MPCORE_CALL mp_ftp_entry_get(mp_ftp *ftp, size_t index, mp_ftp_entry *out);
/* Copies an entry's UTF-8 name, without a NUL (out may be NULL); returns its length, or a negative
   MP_ERR_*. */
MPCORE_API int MPCORE_CALL mp_ftp_entry_name(mp_ftp *ftp, size_t index, char *out, size_t capacity);
/* The seq of the next request, to share with code that sends FTP requests itself. */
MPCORE_API uint16_t MPCORE_CALL mp_ftp_next_seq(mp_ftp *ftp);
MPCORE_API void MPCORE_CALL mp_ftp_set_next_seq(mp_ftp *ftp, uint16_t seq);
/* 1 once the vehicle NAKed a timed listing: listings are plain from then on. Settable, to keep it
   across objects. */
MPCORE_API int MPCORE_CALL mp_ftp_plain_listing(mp_ftp *ftp);
MPCORE_API void MPCORE_CALL mp_ftp_set_plain_listing(mp_ftp *ftp, int plain);
/* FILE_TRANSFER_PROTOCOL messages sent, resends included. */
MPCORE_API uint32_t MPCORE_CALL mp_ftp_requests_sent(mp_ftp *ftp);

/*
 * One vehicle's telemetry, decoded as Mission Planner's CurrentState decodes it (the stored values,
 * before display units). Feed it every frame of the link; frames of other vehicles are ignored
 * (but RADIO/RADIO_STATUS, which are for everyone). Not thread-safe.
 */
typedef struct mp_vehicle mp_vehicle;

#define MP_VEHICLE_BATTERIES 9

/* Fields by size (8, 4, 2, 1 bytes), so the layout has no padding to get wrong. Grows at the end
   of each size group only in a new ABI version; read it with mp_vehicle_state_get. */
typedef struct mp_vehicle_state {
    uint32_t struct_size; /* set to sizeof(mp_vehicle_state) */
    uint32_t messages;    /* frames that changed the state */
    double lat, lng;      /* deg: GLOBAL_POSITION_INT, else GPS_RAW_INT */
    double vx, vy, vz;    /* m/s */
    double battery_voltage[MP_VEHICLE_BATTERIES]; /* V, BATTERY_STATUS by id; [0] smoothed as CurrentState */
    double battery_current[MP_VEHICLE_BATTERIES]; /* A; [0] also SYS_STATUS, [1] also BATTERY2 */
    double battery_used_mah[MP_VEHICLE_BATTERIES];
    double battery_temp[MP_VEHICLE_BATTERIES];    /* degC, kept while not reported */
    double battery_cells[14];                      /* V, battery 0 */
    double lat2, lng2;                             /* deg, GPS2_RAW */
    double ahrs2_lat, ahrs2_lng;                   /* deg, AHRS2 */
    uint64_t uid;                                  /* AUTOPILOT_VERSION */
    uint64_t time_unix_usec;                       /* SYSTEM_TIME */
    float battery_remain_min[MP_VEHICLE_BATTERIES];
    float roll, pitch, yaw;                        /* deg, yaw 0..360 */
    float alt, altasl;                             /* m, relative to home and AMSL */
    float groundspeed, groundcourse, airspeed, climbrate, throttle;
    float gps_hdop, satcount, gps_h_acc, gps_v_acc, gps_vel_acc, gps_hdg_acc, gps_yaw;
    float load;
    float nav_roll, nav_pitch, nav_bearing, target_bearing, wp_dist, alt_error, aspd_error, xtrack_error;
    float vibe_x, vibe_y, vibe_z;
    float ekf_status, ekf_velv, ekf_compv, ekf_poshor, ekf_posvert, ekf_teralt;
    float press_abs, press_temp, airspeed1_temp;
    float ter_curalt, ter_alt;
    float press_abs2, press_temp2, airspeed2_temp; /* SCALED_PRESSURE2 */
    float altasl2, gps_hdop2, satcount2, groundspeed2, groundcourse2;  /* GPS2_RAW */
    float gps_h_acc2, gps_v_acc2, gps_vel_acc2, gps_hdg_acc2, gps_yaw2;
    float rpm[2];
    float esc_voltage[16], esc_current[16], esc_rpm[16], esc_temp[16]; /* ESC_TELEMETRY_* */
    float imu_accel[9], imu_gyro[9], imu_mag[9];   /* x, y, z of IMUs 1..3 (RAW/SCALED_IMU*, HIGHRES_IMU) */
    float imu_temp[3];                             /* degC */
    float pid_ff, pid_p, pid_i, pid_d, pid_desired, pid_achieved, pid_srate, pid_pdmod; /* PID_TUNING */
    float pid_srate_axis[6];                       /* by PID_TUNING_AXIS, roll first */
    float asratio, hwvoltage, sonarrange, sonarvoltage;
    float efi_baro, efi_headtemp, efi_load, efi_exhausttemp, efi_intaketemp, efi_rpm; /* EFI_STATUS, ECU 0 */
    float efi_fuelflow, efi_fuelconsumed, efi_fuelpressure;
    float boardvoltage, servovoltage;              /* POWER_STATUS, mV */
    float wind_dir, wind_speed;                    /* WIND: deg, m/s */
    float freemem, brklevel, aoa, ssa;
    float campoint[3];                             /* MOUNT_STATUS, deg */
    float opt_m_x, opt_m_y, posn, pose, posd;
    float gen_status, gen_speed, gen_current, gen_voltage; /* GENERATOR_STATUS */
    float mcu_temp, mcu_voltage, mcu_voltage_min, mcu_voltage_max; /* MCU_STATUS 0 */
    float ahrs2_roll, ahrs2_pitch, ahrs2_yaw, ahrs2_alt;
    uint32_t custom_mode;
    uint32_t sensors_present, sensors_enabled, sensors_health;
    uint32_t errors_count[4];
    uint32_t vibe_clip[3];
    int32_t battery_remaining[MP_VEHICLE_BATTERIES];
    int32_t rxrssi;
    int32_t ekf_flags;
    uint32_t flight_sw_version, capabilities;     /* AUTOPILOT_VERSION (the low 32 capability bits) */
    uint32_t rangefinder[10];                      /* cm, DISTANCE_SENSOR by id */
    uint32_t voltageflag, gen_runtime;
    int32_t gen_maint_time;
    uint16_t wpno, drop_rate_comm, rxerrors, fixed;
    uint16_t ter_load, ter_pend, ter_space;
    uint16_t rc_in[16];
    uint16_t servo_out[32];
    uint16_t i2cerrors, fenceb_count;
    uint16_t hygro_humi[2];
    int16_t hygro_temp[2], opt_x, opt_y;
    uint8_t heartbeat_seen, type, autopilot, base_mode, system_status, armed, landed, failsafe;
    uint8_t custom_mode_seen, sensors_seen, vtol_state, landed_state, gps_fix_type, use_location, got_vfr;
    uint8_t rssi, remrssi, txbuf, noise, remnoise;
    uint8_t pid_axis, gps_fix_type2, efi_health, opt_qua, fenceb_type, fenceb_status, got_wind;
    uint8_t reserved[7]; /* to a multiple of 8: no tail padding */
} mp_vehicle_state;

MPCORE_API mp_vehicle *MPCORE_CALL mp_vehicle_create(uint32_t sysid, uint8_t compid);
MPCORE_API void MPCORE_CALL mp_vehicle_destroy(mp_vehicle *vehicle);
/* Any frame of the link. Returns 1 if it changed the state, 0 if not, or a negative MP_ERR_*.
   now_us: the caller's clock (battery mAh is integrated from SYS_STATUS current as CurrentState does). */
MPCORE_API int MPCORE_CALL mp_vehicle_frame(mp_vehicle *vehicle, const uint8_t *frame, size_t len, int64_t now_us);
/* Copies the state; out->struct_size must be set. */
MPCORE_API int MPCORE_CALL mp_vehicle_state_get(mp_vehicle *vehicle, mp_vehicle_state *out);

#ifdef __cplusplus
}
#endif

#endif /* MPCORE_MPCORE_H */
