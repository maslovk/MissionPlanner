#include <cmath>
#include <cstring>
#include <vector>

#include "mavlink_c.hpp"
#include "protocol.hpp"
#include "test.hpp"
#include "vehicle_state.hpp"

using namespace mpcore;

namespace {

constexpr int64_t kSec = 1'000'000;

template <typename T>
std::vector<uint8_t> frame(uint32_t msgid, const T &msg, std::size_t len, uint32_t sysid = 1, uint8_t compid = 1,
                           bool mavlink1 = false) {
    EncodeRequest req;
    req.msgid = msgid;
    req.payload = reinterpret_cast<const uint8_t *>(&msg);
    req.payload_len = len;
    req.src_sysid = sysid;
    req.src_compid = compid;
    req.mavlink1 = mavlink1;
    std::vector<uint8_t> out(kMaxFrameLen);
    const int n = encode(req, out.data(), out.size());
    CHECK(n > 0);
    out.resize(n > 0 ? static_cast<std::size_t>(n) : 0);
    return out;
}

#define FRAME(NAME, msg) frame(MAVLINK_MSG_ID_##NAME, msg, MAVLINK_MSG_ID_##NAME##_LEN)
#define FRAME_FROM(NAME, msg, sysid, compid) frame(MAVLINK_MSG_ID_##NAME, msg, MAVLINK_MSG_ID_##NAME##_LEN, sysid, compid)
#define FRAME_V1(NAME, msg) frame(MAVLINK_MSG_ID_##NAME, msg, MAVLINK_MSG_ID_##NAME##_LEN, 1, 1, true)

bool feed(VehicleState &v, const std::vector<uint8_t> &f, int64_t now = 0) {
    return v.on_frame(f.data(), f.size(), now);
}

}  // namespace

TEST(vehicle_heartbeat_and_others) {
    VehicleState v(1, 1);
    mavlink_heartbeat_t hb{};
    hb.type = 2;
    hb.autopilot = 3;
    hb.base_mode = 128 | 1;
    hb.custom_mode = 5;
    hb.system_status = 4;
    CHECK(feed(v, FRAME(HEARTBEAT, hb)));
    CHECK(v.state().armed == 1 && v.state().custom_mode == 5 && v.state().custom_mode_seen == 1);
    CHECK(v.state().landed == 0 && v.state().failsafe == 0);
    // a heartbeat without the custom mode flag keeps the mode
    hb.base_mode = 0;
    hb.custom_mode = 9;
    hb.system_status = 5;
    CHECK(feed(v, FRAME(HEARTBEAT, hb)));
    CHECK(v.state().armed == 0 && v.state().custom_mode == 5 && v.state().failsafe == 1);
    // a GCS, another vehicle, another component
    hb.type = 6;
    CHECK(!feed(v, FRAME(HEARTBEAT, hb)));
    hb.type = 2;
    CHECK(!feed(v, FRAME_FROM(HEARTBEAT, hb, 2, 1)));
    CHECK(!feed(v, FRAME_FROM(HEARTBEAT, hb, 1, 2)));
    CHECK_EQ(v.state().messages, 2u);
    // the radio's RADIO_STATUS is for everyone
    mavlink_radio_status_t radio{};
    radio.rssi = 200;
    radio.remrssi = 190;
    radio.rxerrors = 7;
    CHECK(feed(v, FRAME_FROM(RADIO_STATUS, radio, 51, 68)));
    CHECK(v.state().rssi == 200 && v.state().remrssi == 190 && v.state().rxerrors == 7);
}

TEST(vehicle_attitude_and_position) {
    VehicleState v(1, 1);
    mavlink_attitude_t att{};
    att.roll = 0.1f;
    att.pitch = -0.2f;
    att.yaw = -1.0f;
    CHECK(feed(v, FRAME(ATTITUDE, att)));
    CHECK_EQ(v.state().roll, static_cast<float>(0.1f * (180 / 3.14159265358979323846)));
    CHECK(std::fabs(v.state().yaw - (360 - 57.29578f)) < 1e-3f);

    // GPS_RAW_INT gives the position until GLOBAL_POSITION_INT has one
    mavlink_gps_raw_int_t gps{};
    gps.lat = -353632610;
    gps.lon = 1491652300;
    gps.alt = 584000;
    gps.eph = 121;
    gps.satellites_visible = 10;
    gps.fix_type = 3;
    gps.vel = 250;
    gps.cog = 9000;
    gps.h_acc = 1500;
    CHECK(feed(v, FRAME(GPS_RAW_INT, gps)));
    CHECK_EQ(v.state().lat, -353632610 * 1.0e-7);
    CHECK_EQ(v.state().altasl, 584.0f);
    CHECK_EQ(v.state().gps_hdop, 1.21f);
    CHECK_EQ(v.state().satcount, 10.0f);
    CHECK_EQ(v.state().groundspeed, 2.5f);
    CHECK_EQ(v.state().groundcourse, 90.0f);
    CHECK_EQ(v.state().gps_h_acc, 1.5f);
    // MAVLink 1 has no accuracies
    CHECK(feed(v, FRAME_V1(GPS_RAW_INT, gps)));
    CHECK_EQ(v.state().gps_h_acc, -1.0f);

    mavlink_global_position_int_t pos{};
    pos.lat = -353000000;
    pos.lon = 1490000000;
    pos.alt = 600500;
    pos.relative_alt = 12345;
    pos.vx = 150;
    CHECK(feed(v, FRAME(GLOBAL_POSITION_INT, pos)));
    CHECK(v.state().use_location == 1 && v.state().lat == -35.3 && v.state().alt == 12.345f);
    CHECK_EQ(v.state().vx, 1.5);
    CHECK(feed(v, FRAME(GPS_RAW_INT, gps)));
    CHECK_EQ(v.state().lat, -35.3);  // GLOBAL_POSITION_INT's, not the GPS's

    // dead reckoning without a position: the altitude only, and the GPS's position again
    pos.lat = 0;
    pos.relative_alt = 1000;
    CHECK(feed(v, FRAME(GLOBAL_POSITION_INT, pos)));
    CHECK(v.state().use_location == 0 && v.state().alt == 1.0f && v.state().lat == -35.3);
    CHECK(feed(v, FRAME(GPS_RAW_INT, gps)));
    CHECK_EQ(v.state().lat, -353632610 * 1.0e-7);

    // no fix: the speed and course stay
    gps.fix_type = 1;
    gps.vel = 0;
    gps.cog = 0;
    CHECK(feed(v, FRAME(GPS_RAW_INT, gps)));
    CHECK(v.state().groundspeed == 2.5f && v.state().groundcourse == 90.0f && v.state().gps_fix_type == 1);
}

TEST(vehicle_battery) {
    VehicleState v(1, 1);
    mavlink_battery_status_t b{};
    for (int c = 0; c < 10; c++)
        b.voltages[c] = UINT16_MAX;
    b.voltages[0] = 4000;
    b.voltages[1] = 4100;
    b.current_battery = 1234;
    b.current_consumed = 500;
    b.battery_remaining = 80;
    b.temperature = INT16_MAX;
    b.time_remaining = 600;
    CHECK(feed(v, FRAME(BATTERY_STATUS, b)));
    const auto &s = v.state();
    // the cells' sum, smoothed as CurrentState.battery_voltage (from the first reading)
    const double first = 0.0 + 4000 / 1000.0 + 4100 / 1000.0;
    const double smoothed = first * 0.4f + first * 0.6f;
    CHECK_EQ(s.battery_voltage[0], smoothed);
    CHECK(s.battery_cells[0] == 4.0 && s.battery_cells[1] == 4.1 && s.battery_cells[2] == 0.0);
    CHECK_EQ(s.battery_current[0], static_cast<double>(12.34f));
    CHECK(s.battery_used_mah[0] == 500 && s.battery_remaining[0] == 80 && s.battery_remain_min[0] == 10.0f);
    CHECK_EQ(s.battery_temp[0], 0.0);
    b.voltages[1] = 3900;
    b.temperature = 2550;
    b.battery_remaining = -1;
    CHECK(feed(v, FRAME(BATTERY_STATUS, b)));
    const double second = 0.0 + 4000 / 1000.0 + 3900 / 1000.0;
    CHECK_EQ(s.battery_voltage[0], second * 0.4f + smoothed * 0.6f);
    CHECK(s.battery_temp[0] == 25.5 && s.battery_remaining[0] == 0);
    // another battery: as reported
    b.id = 3;
    b.battery_remaining = -1;
    CHECK(feed(v, FRAME(BATTERY_STATUS, b)));
    CHECK(s.battery_voltage[3] == second && s.battery_remaining[3] == -1);
    b.id = 9;
    CHECK(!feed(v, FRAME(BATTERY_STATUS, b)));
}

TEST(vehicle_sys_status_current) {
    VehicleState v(1, 1);
    mavlink_sys_status_t ss{};
    ss.load = 455;
    ss.current_battery = 1000;  // 10 A
    ss.battery_remaining = 120;
    ss.onboard_control_sensors_present = 0x800000;
    ss.onboard_control_sensors_enabled = 0x800000;
    ss.onboard_control_sensors_health = 0x800000;
    CHECK(feed(v, FRAME(SYS_STATUS, ss), 0));
    CHECK(v.state().load == 45.5f && v.state().battery_remaining[0] == 0 && v.state().battery_current[0] == 10.0);
    CHECK(feed(v, FRAME(SYS_STATUS, ss), 3600 * kSec / 10));  // 10 A for 6 minutes
    CHECK(std::fabs(v.state().battery_used_mah[0] - 1000.0) < 1e-6);
    ss.current_battery = -1;  // no sensor
    CHECK(feed(v, FRAME(SYS_STATUS, ss), 3600 * kSec));
    CHECK(v.state().battery_current[0] == 0 && std::fabs(v.state().battery_used_mah[0] - 1000.0) < 1e-6);

    // the reversed-motor sensor makes the throttle negative
    mavlink_vfr_hud_t hud{};
    hud.throttle = 40;
    hud.climb = 1.5f;
    CHECK(feed(v, FRAME(VFR_HUD, hud)));
    CHECK(v.state().throttle == -40.0f && v.state().climbrate == 1.5f && v.state().got_vfr == 1);
}

TEST(vehicle_rc_servo_nav_ekf) {
    VehicleState v(1, 1);
    mavlink_rc_channels_t rc{};
    rc.chan1_raw = 1100;
    rc.chan16_raw = 1900;
    rc.rssi = 127;
    CHECK(feed(v, FRAME(RC_CHANNELS, rc)));
    CHECK(v.state().rc_in[0] == 1100 && v.state().rc_in[15] == 1900 && v.state().rxrssi == 50);
    rc.rssi = 255;
    CHECK(feed(v, FRAME(RC_CHANNELS, rc)));
    CHECK_EQ(v.state().rxrssi, 0);
    mavlink_rc_channels_raw_t raw{};
    raw.chan1_raw = 1500;
    raw.rssi = 255;
    CHECK(feed(v, FRAME(RC_CHANNELS_RAW, raw)));
    CHECK(v.state().rc_in[0] == 1500 && v.state().rc_in[15] == 1900 && v.state().rxrssi == 100);

    mavlink_servo_output_raw_t so{};
    so.servo1_raw = 1000;
    so.servo16_raw = 2000;
    so.port = 1;
    CHECK(feed(v, FRAME(SERVO_OUTPUT_RAW, so)));
    CHECK(v.state().servo_out[16] == 1000 && v.state().servo_out[31] == 2000 && v.state().servo_out[0] == 0);
    so.port = 2;
    CHECK(!feed(v, FRAME(SERVO_OUTPUT_RAW, so)));

    mavlink_nav_controller_output_t nav{};
    nav.nav_bearing = -45;
    nav.wp_dist = 300;
    nav.aspd_error = 250;
    CHECK(feed(v, FRAME(NAV_CONTROLLER_OUTPUT, nav)));
    CHECK(v.state().nav_bearing == -45.0f && v.state().wp_dist == 300.0f && v.state().aspd_error == 2.5f);

    mavlink_ekf_status_report_t ekf{};
    ekf.velocity_variance = 0.3f;
    ekf.compass_variance = 0.6f;
    ekf.flags = 1 | 2;
    CHECK(feed(v, FRAME(EKF_STATUS_REPORT, ekf)));
    CHECK_EQ(v.state().ekf_status, 0.6f);
    ekf.flags = 2;  // no attitude: red
    CHECK(feed(v, FRAME(EKF_STATUS_REPORT, ekf)));
    CHECK_EQ(v.state().ekf_status, 1.0f);
    ekf.flags = 1 | 1024;  // not initialised: red
    CHECK(feed(v, FRAME(EKF_STATUS_REPORT, ekf)));
    CHECK_EQ(v.state().ekf_status, 1.0f);
}
