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

int main(void) {
    test_version_and_messages();
    test_round_trip();
    test_errors();
    printf("%s (%d failed checks)\n", failures == 0 ? "ok" : "FAIL", failures);
    return failures == 0 ? 0 : 1;
}
