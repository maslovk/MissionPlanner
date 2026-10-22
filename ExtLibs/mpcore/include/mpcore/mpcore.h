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

#ifdef __cplusplus
}
#endif

#endif /* MPCORE_MPCORE_H */
