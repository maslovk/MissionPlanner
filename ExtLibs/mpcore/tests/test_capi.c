/* Exercises the C ABI from plain C, through the shared library only. */

#include <stdio.h>
#include <string.h>

#include "mpcore/mpcore.h"

static int failures = 0;

#define CHECK(cond)                                                     \
    do {                                                                \
        if (!(cond)) {                                                  \
            failures++;                                                 \
            printf("    %s:%d: CHECK failed: %s\n", __FILE__, __LINE__, #cond); \
        }                                                               \
    } while (0)

typedef struct capture {
    uint8_t bytes[4096];
    size_t len;
    int frames;
    uint32_t last_sysid;
    uint32_t last_msgid;
    uint32_t last_flags;
    int lines;
} capture;

static void MPCORE_CALL on_write(void *user, const uint8_t *data, size_t len) {
    capture *c = (capture *)user;
    if (c->len + len <= sizeof(c->bytes)) {
        memcpy(c->bytes + c->len, data, len);
        c->len += len;
    }
}

static void MPCORE_CALL on_frame(void *user, const mp_frame *f) {
    capture *c = (capture *)user;
    c->frames++;
    c->last_sysid = f->sysid;
    c->last_msgid = f->msgid;
    c->last_flags = f->flags;
}

static void MPCORE_CALL on_text(void *user, const char *line, size_t len) {
    capture *c = (capture *)user;
    (void)line;
    (void)len;
    c->lines++;
}

static mp_link *make_link(uint32_t sysid, uint8_t compid, capture *c) {
    mp_link_config cfg;
    mp_link_callbacks cb;
    mp_link_config_init(&cfg);
    cfg.gcs_sysid = sysid;
    cfg.gcs_compid = compid;
    memset(&cb, 0, sizeof(cb));
    cb.struct_size = sizeof(cb);
    cb.user = c;
    cb.on_frame = on_frame;
    cb.on_write = on_write;
    cb.on_text = on_text;
    return mp_link_create(&cfg, &cb);
}

static void test_version_and_messages(void) {
    mp_msg_info info;
    CHECK(mp_abi_version() == MPCORE_ABI_VERSION);
    CHECK(mp_version_string() != NULL);

    memset(&info, 0, sizeof(info));
    info.struct_size = sizeof(info);
    CHECK(mp_msg_info_get(0, &info) == MP_OK);
    CHECK(info.name != NULL && strcmp(info.name, "HEARTBEAT") == 0);
    CHECK(info.crc_extra == 50 && info.min_len == 9);
    CHECK(info.target_system_ofs == 0xFF);
    CHECK(mp_msg_info_get(76, &info) == MP_OK);
    CHECK(info.target_system_ofs == 30 && info.target_component_ofs == 31);
    CHECK(mp_msg_info_get(0xFFFFFF, &info) == MP_ERR_UNKNOWN_MSG);
    info.struct_size = 0;
    CHECK(mp_msg_info_get(0, &info) == MP_ERR_ARGUMENT);
}

static void test_round_trip(void) {
    static const uint8_t heartbeat[9] = {0, 0, 0, 0, 2, 3, 81, 4, 3};
    static const char text[] = "Hello from the bootloader\r\n";
    capture vc, gc;
    mp_link *vehicle, *gcs;
    mp_peer_stats ps;
    mp_link_stats ls;
    int i;

    memset(&vc, 0, sizeof(vc));
    memset(&gc, 0, sizeof(gc));
    vehicle = make_link(0x80000001u, 1, &vc);
    gcs = make_link(255, 190, &gc);
    CHECK(vehicle != NULL && gcs != NULL);

    for (i = 0; i < 5; i++)
        CHECK(mp_link_send(vehicle, 0, heartbeat, sizeof(heartbeat), 255, 190, 0) > 0);
    CHECK(mp_link_feed(gcs, (const uint8_t *)text, sizeof(text) - 1, 0) == 0);
    CHECK(mp_link_feed(gcs, vc.bytes, vc.len, 1000) == 5);
    CHECK(gc.frames == 5);
    CHECK(gc.lines == 1);
    CHECK(gc.last_sysid == 0x80000001u);
    CHECK(gc.last_flags & MP_FRAME_MAVLINK2);

    CHECK(mp_link_peer_count(gcs) == 1);
    memset(&ps, 0, sizeof(ps));
    ps.struct_size = sizeof(ps);
    CHECK(mp_link_peer_stats(gcs, 0x80000001u, 1, &ps) == MP_OK);
    CHECK(ps.sysid == 0x80000001u && ps.compid == 1 && ps.mavlink2 == 1);
    CHECK(ps.frames == 5 && ps.packets_lost == 0.0f && ps.link_quality == 100.0f);
    CHECK(ps.last_seq == 4);
    CHECK(mp_link_peer_stats(gcs, 1, 1, &ps) == MP_ERR_NOT_FOUND);
    CHECK(mp_link_peer_at(gcs, 0, &ps) == MP_OK && ps.sysid == 0x80000001u);
    CHECK(mp_link_peer_at(gcs, 1, &ps) == MP_ERR_NOT_FOUND);

    /* an older caller with a shorter struct gets only the fields it knows */
    memset(&ps, 0xEE, sizeof(ps));
    ps.struct_size = (uint32_t)offsetof(mp_peer_stats, packets_lost);
    CHECK(mp_link_peer_stats(gcs, 0x80000001u, 1, &ps) == MP_OK);
    CHECK(ps.struct_size == offsetof(mp_peer_stats, packets_lost));
    CHECK(ps.sysid == 0x80000001u);
    {
        const uint8_t *tail = (const uint8_t *)&ps.packets_lost;
        CHECK(tail[0] == 0xEE);
    }

    memset(&ls, 0, sizeof(ls));
    ls.struct_size = sizeof(ls);
    CHECK(mp_link_stats_get(gcs, &ls) == MP_OK);
    CHECK(ls.frames_in == 5 && ls.mavlink2_in == 5 && ls.text_lines == 1);
    CHECK(ls.bytes_in == vc.len + sizeof(text) - 1);

    mp_link_destroy(vehicle);
    mp_link_destroy(gcs);
}

static void test_errors(void) {
    static const uint8_t heartbeat[9] = {0, 0, 0, 0, 2, 3, 81, 4, 3};
    static const uint8_t too_long[10] = {0};
    capture c;
    mp_link_config cfg;
    mp_link *l;

    memset(&c, 0, sizeof(c));
    l = make_link(255, 190, &c);
    CHECK(mp_link_send(l, 0xFFFFFF, NULL, 0, 1, 1, 0) == MP_ERR_UNKNOWN_MSG);
    CHECK(mp_link_send(l, 0, too_long, sizeof(too_long), 1, 1, 0) == MP_ERR_PAYLOAD_SIZE);
    CHECK(mp_link_send(NULL, 0, heartbeat, sizeof(heartbeat), 1, 1, 0) == MP_ERR_ARGUMENT);
    CHECK(mp_link_feed(NULL, heartbeat, sizeof(heartbeat), 0) == 0);
    CHECK(mp_link_poll(NULL, 0) == 0);
    CHECK(mp_link_poll(l, 10000000) == 0); /* nothing pending */
    CHECK(mp_link_add_signing_key(l, NULL) == MP_ERR_ARGUMENT);
    CHECK(c.len == 0);
    mp_link_destroy(l);
    mp_link_destroy(NULL);

    mp_link_config_init(&cfg);
    cfg.struct_size = sizeof(cfg) + 4; /* from a newer, unknown header */
    CHECK(mp_link_create(&cfg, NULL) == NULL);
    CHECK((l = mp_link_create(NULL, NULL)) != NULL);
    mp_link_destroy(l);
}

static void test_frame_encode(void) {
    static const uint8_t heartbeat[9] = {0, 0, 0, 0, 2, 3, 81, 4, 3};
    static uint8_t key[32];
    uint8_t command[33];
    uint8_t frame[MPCORE_MAX_FRAME_LEN];
    mp_encode_args args;
    capture c;
    mp_link *gcs;
    int n;

    memset(&args, 0, sizeof(args));
    args.struct_size = sizeof(args);
    args.msgid = 0;
    args.src_sysid = 1;
    args.src_compid = 1;
    args.seq = 7;
    args.mavlink1 = 1;
    n = mp_frame_encode(&args, heartbeat, sizeof(heartbeat), frame, sizeof(frame));
    CHECK(n == 6 + 9 + 2 && frame[0] == 0xFE && frame[2] == 7 && frame[3] == 1);

    memset(&c, 0, sizeof(c));
    gcs = make_link(255, 190, &c);
    CHECK(mp_link_feed(gcs, frame, (size_t)n, 0) == 1 && c.last_sysid == 1);

    /* MAVLink 2, signed, to a 32-bit system: the receiving link verifies it with the same key */
    memset(key, 9, sizeof(key));
    memset(command, 0, sizeof(command));
    command[28] = 44; command[29] = 1; command[31] = 1; command[32] = 1;
    args.msgid = 76;
    args.mavlink1 = 0;
    args.target_sysid = 0x80000001u;
    args.sign = 1;
    args.key = key;
    args.link_id = 3;
    args.timestamp = 12345;
    n = mp_frame_encode(&args, command, sizeof(command), frame, sizeof(frame));
    CHECK(n == 14 + 33 + 2 + 13 && frame[0] == 0xFD && frame[2] == 0x05);
    CHECK(frame[14 + 30] == 255); /* payload target: the sentinel */
    mp_link_add_signing_key(gcs, key);
    c.frames = 0;
    CHECK(mp_link_feed(gcs, frame, (size_t)n, 0) == 1);
    CHECK(c.last_msgid == 76 && (c.last_flags & MP_FRAME_SIGNATURE_OK) && (c.last_flags & MP_FRAME_NOT_FOR_US));
    mp_link_destroy(gcs);

    /* errors */
    args.key = NULL;
    CHECK(mp_frame_encode(&args, command, sizeof(command), frame, sizeof(frame)) == MP_ERR_ARGUMENT);
    args.sign = 0;
    args.mavlink1 = 1;
    CHECK(mp_frame_encode(&args, command, sizeof(command), frame, sizeof(frame)) == MP_ERR_UNREPRESENTABLE);
    args.mavlink1 = 0;
    CHECK(mp_frame_encode(&args, command, sizeof(command), frame, 10) == MP_ERR_ARGUMENT);
    CHECK(mp_frame_encode(&args, command, sizeof(command), NULL, 0) == MP_ERR_ARGUMENT);
    args.struct_size = sizeof(args) + 8;
    CHECK(mp_frame_encode(&args, command, sizeof(command), frame, sizeof(frame)) == MP_ERR_ARGUMENT);
    CHECK(mp_frame_encode(NULL, command, sizeof(command), frame, sizeof(frame)) == MP_ERR_ARGUMENT);
}

typedef struct param_capture {
    uint32_t sent[8];
    int sent_count;
    int params;
    int duplicates;
    char last_name[17];
    float last_value;
} param_capture;

static void MPCORE_CALL param_send(void *user, uint32_t msgid, const uint8_t *payload, size_t len) {
    param_capture *c = (param_capture *)user;
    (void)payload;
    (void)len;
    if (c->sent_count < 8)
        c->sent[c->sent_count] = msgid;
    c->sent_count++;
}

static void MPCORE_CALL param_got(void *user, const mp_param *p) {
    param_capture *c = (param_capture *)user;
    if (p->duplicate) {
        c->duplicates++;
        return;
    }
    c->params++;
    strncpy(c->last_name, p->name, 16);
    c->last_name[16] = 0;
    c->last_value = p->value;
}

/* a PARAM_VALUE frame from sysid 1 compid 1 */
static int param_frame(uint8_t *frame, uint16_t index, uint16_t count, const char *name, float value) {
    uint8_t payload[25];
    mp_encode_args args;
    memset(payload, 0, sizeof(payload));
    memcpy(payload, &value, 4);
    memcpy(payload + 4, &count, 2);
    memcpy(payload + 6, &index, 2);
    memcpy(payload + 8, name, strlen(name));
    payload[24] = 9;
    memset(&args, 0, sizeof(args));
    args.struct_size = sizeof(args);
    args.msgid = 22;
    args.src_sysid = 1;
    args.src_compid = 1;
    return mp_frame_encode(&args, payload, sizeof(payload), frame, MPCORE_MAX_FRAME_LEN);
}

static void test_param_download(void) {
    param_capture c;
    mp_param_download_status st;
    uint8_t frame[MPCORE_MAX_FRAME_LEN];
    mp_param_download *d;
    int n;

    memset(&c, 0, sizeof(c));
    d = mp_param_download_create(1, 1, param_send, param_got, &c);
    CHECK(d != NULL);
    CHECK(mp_param_download_start(d, 0) == MP_OK);
    CHECK(c.sent_count == 1 && c.sent[0] == 21); /* PARAM_REQUEST_LIST */

    n = param_frame(frame, 0, 2, "FRAME_CLASS", 1.0f);
    CHECK(mp_param_download_frame(d, frame, (size_t)n, 1000) == MP_OK);
    CHECK(mp_param_download_frame(d, frame, (size_t)n, 2000) == MP_OK);
    CHECK(c.params == 1 && c.duplicates == 1 && strcmp(c.last_name, "FRAME_CLASS") == 0);
    CHECK(mp_param_download_tick(d, 3000) == 0);

    /* a full 16-character name has no NUL on the wire */
    n = param_frame(frame, 1, 2, "ABCDEFGHIJKLMNOP", 2.5f);
    mp_param_download_frame(d, frame, (size_t)n, 4000);
    CHECK(strcmp(c.last_name, "ABCDEFGHIJKLMNOP") == 0 && c.last_value == 2.5f);
    CHECK(mp_param_download_tick(d, 5000) == 1);

    memset(&st, 0, sizeof(st));
    st.struct_size = sizeof(st);
    CHECK(mp_param_download_status_get(d, &st) == MP_OK);
    CHECK(st.done == 1 && st.received == 2 && st.total == 2 && st.full_list_requests == 1 && st.single_requests == 0);
    mp_param_download_destroy(d);
    mp_param_download_destroy(NULL);
    CHECK(mp_param_download_start(NULL, 0) == MP_ERR_ARGUMENT);
    CHECK(mp_param_download_tick(NULL, 0) == MP_ERR_ARGUMENT);
}

int main(void) {
    test_version_and_messages();
    test_round_trip();
    test_errors();
    test_frame_encode();
    test_param_download();
    printf("%s (%d failed checks)\n", failures == 0 ? "ok" : "FAIL", failures);
    return failures == 0 ? 0 : 1;
}
