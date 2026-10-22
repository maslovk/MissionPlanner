#include "vehicle_state.hpp"

#include <algorithm>
#include <climits>
#include <cmath>

#include "mavlink_c.hpp"
#include "protocol.hpp"

namespace mpcore {

namespace {

constexpr double kRad2Deg = 180 / 3.14159265358979323846;  // Mission Planner's MathHelper.rad2deg
constexpr uint8_t kTypeGcs = 6;                              // MAV_TYPE_GCS
constexpr uint8_t kStateStandby = 3, kStateCritical = 5;     // MAV_STATE
constexpr uint8_t kModeFlagCustom = 1, kModeFlagArmed = 128;
constexpr uint32_t kSensorReverseMotor = 0x800000;           // MAV_SYS_STATUS_REVERSE_MOTOR
constexpr int kEkfUninitialized = 1024;

template <typename T>
T read(const FrameView &f) {
    T v{};
    copy_payload(f, reinterpret_cast<uint8_t *>(&v), sizeof(v));
    return v;
}

// CurrentState's yaw and groundcourse setters
float wrap360(float v) {
    return v < 0 ? v + 360 : v;
}

// CurrentState.battery_remaining's setter
int32_t clamp_remaining(int32_t v) {
    return v < 0 || v > 100 ? 0 : v;
}

}  // namespace

VehicleState::VehicleState(uint32_t sysid, uint8_t compid) : sysid_(sysid), compid_(compid) {
    s_.struct_size = sizeof(s_);
}

// CurrentState.current and current2: "no sensor" (-0.01 A, or negative for the second) is not a
// reading; the rest is integrated into the used mAh against the clock, from the first reading on.
void VehicleState::set_current(int index, double value, int64_t now_us) {
    if (last_current_us_[index] < 0)
        last_current_us_[index] = now_us;
    if (index == 0 && value == static_cast<double>(-0.01f)) {
        s_.battery_current[0] = 0;
        return;
    }
    if (index == 1 && value < 0)
        return;
    const double hours = static_cast<double>(now_us - last_current_us_[index]) / 3.6e9;
    s_.battery_used_mah[index] += value * 1000.0 * hours;
    s_.battery_current[index] = value;
    last_current_us_[index] = now_us;
}

bool VehicleState::on_frame(const uint8_t *frame, std::size_t len, int64_t now_us) {
    FrameView f;
    if (!view_frame(frame, len, f))
        return false;
    const bool mine = f.sysid == sysid_ && f.compid == compid_;
    const bool radio = f.msgid == MAVLINK_MSG_ID_RADIO || f.msgid == MAVLINK_MSG_ID_RADIO_STATUS;
    if (!mine && !radio)
        return false;
    const bool mavlink2 = frame[0] == kStxV2;

    switch (f.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
        const auto hb = read<mavlink_heartbeat_t>(f);
        if (hb.type == kTypeGcs)
            return false;
        s_.heartbeat_seen = 1;
        s_.type = hb.type;
        s_.autopilot = hb.autopilot;
        s_.base_mode = hb.base_mode;
        s_.system_status = hb.system_status;
        s_.armed = (hb.base_mode & kModeFlagArmed) != 0;
        s_.landed = hb.system_status == kStateStandby;
        s_.failsafe = hb.system_status == kStateCritical;
        if ((hb.base_mode & kModeFlagCustom) != 0) {
            s_.custom_mode = hb.custom_mode;
            s_.custom_mode_seen = 1;
        }
        break;
    }
    case MAVLINK_MSG_ID_SYS_STATUS: {
        const auto ss = read<mavlink_sys_status_t>(f);
        s_.load = ss.load / 10.0f;
        s_.battery_remaining[0] = clamp_remaining(ss.battery_remaining);
        set_current(0, ss.current_battery / 100.0f, now_us);
        s_.drop_rate_comm = ss.drop_rate_comm;
        s_.errors_count[0] = ss.errors_count1;
        s_.errors_count[1] = ss.errors_count2;
        s_.errors_count[2] = ss.errors_count3;
        s_.errors_count[3] = ss.errors_count4;
        s_.sensors_enabled = ss.onboard_control_sensors_enabled;
        s_.sensors_health = ss.onboard_control_sensors_health;
        s_.sensors_present = ss.onboard_control_sensors_present;
        s_.sensors_seen = 1;
        break;
    }
    case MAVLINK_MSG_ID_EXTENDED_SYS_STATE: {
        const auto es = read<mavlink_extended_sys_state_t>(f);
        s_.vtol_state = es.vtol_state;
        s_.landed_state = es.landed_state;
        break;
    }
    case MAVLINK_MSG_ID_BATTERY2: {
        const auto b = read<mavlink_battery2_t>(f);
        set_current(1, b.current_battery / 100.0f, now_us);
        break;
    }
    case MAVLINK_MSG_ID_BATTERY_STATUS: {
        const auto b = read<mavlink_battery_status_t>(f);
        if (b.id >= MP_VEHICLE_BATTERIES)
            return false;
        // the cells add up to more than SYS_STATUS's 65.535 V
        double total = 0;
        for (uint16_t v : b.voltages)
            total += v != UINT16_MAX ? v / 1000.0 : 0;
        for (uint16_t v : b.voltages_ext)
            total += v != UINT16_MAX ? v / 1000.0 : 0;
        const int i = b.id;
        if (i == 0) {
            if (b.voltages[0] != UINT16_MAX) {
                for (int c = 0; c < 10; c++)
                    s_.battery_cells[c] = b.voltages[c] != UINT16_MAX ? b.voltages[c] / 1000.0 : 0.0;
                for (int c = 0; c < 4; c++)
                    s_.battery_cells[10 + c] = b.voltages_ext[c] != UINT16_MAX ? b.voltages_ext[c] / 1000.0 : 0.0;
            }
            s_.battery_remaining[0] = clamp_remaining(b.battery_remaining);
            // CurrentState.battery_voltage's smoothing
            if (s_.battery_voltage[0] == 0)
                s_.battery_voltage[0] = total;
            s_.battery_voltage[0] = total * 0.4f + s_.battery_voltage[0] * 0.6f;
        } else {
            s_.battery_remaining[i] = b.battery_remaining;
            s_.battery_voltage[i] = total;
        }
        s_.battery_used_mah[i] = b.current_consumed;
        s_.battery_current[i] = b.current_battery / 100.0f;
        if (b.temperature != INT16_MAX)
            s_.battery_temp[i] = b.temperature / 100.0;
        s_.battery_remain_min[i] = static_cast<float>(b.time_remaining) / 60.0f;
        break;
    }
    case MAVLINK_MSG_ID_SCALED_PRESSURE: {
        const auto p = read<mavlink_scaled_pressure_t>(f);
        s_.press_abs = p.press_abs;
        s_.press_temp = p.temperature;
        s_.airspeed1_temp = p.temperature_press_diff;
        break;
    }
    case MAVLINK_MSG_ID_TERRAIN_REPORT: {
        const auto t = read<mavlink_terrain_report_t>(f);
        s_.ter_curalt = t.current_height;
        s_.ter_alt = t.terrain_height;
        s_.ter_load = t.loaded;
        s_.ter_pend = t.pending;
        s_.ter_space = t.spacing;
        break;
    }
    case MAVLINK_MSG_ID_ATTITUDE: {
        const auto a = read<mavlink_attitude_t>(f);
        s_.roll = static_cast<float>(a.roll * kRad2Deg);
        s_.pitch = static_cast<float>(a.pitch * kRad2Deg);
        s_.yaw = wrap360(static_cast<float>(a.yaw * kRad2Deg));
        break;
    }
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        const auto p = read<mavlink_global_position_int_t>(f);
        s_.alt = static_cast<float>(p.relative_alt) / 1000.0f;
        // dead reckoning may send 0 or no position
        s_.use_location = !(p.lat == 0 || p.lon == 0 || p.lat == INT32_MAX || p.lon == INT32_MAX);
        if (s_.use_location) {
            s_.lat = p.lat / 10000000.0;
            s_.lng = p.lon / 10000000.0;
            s_.altasl = static_cast<float>(p.alt) / 1000.0f;
            s_.vx = p.vx * 0.01;
            s_.vy = p.vy * 0.01;
            s_.vz = p.vz * 0.01;
        }
        break;
    }
    case MAVLINK_MSG_ID_GPS_RAW_INT: {
        const auto g = read<mavlink_gps_raw_int_t>(f);
        if (!s_.use_location) {
            if (g.lat != INT32_MAX)
                s_.lat = g.lat * 1.0e-7;
            if (g.lon != INT32_MAX)
                s_.lng = g.lon * 1.0e-7;
            s_.altasl = static_cast<float>(g.alt) / 1000.0f;
        }
        s_.gps_fix_type = g.fix_type;
        if (g.eph != UINT16_MAX)
            s_.gps_hdop = static_cast<float>(std::nearbyint(g.eph / 100.0 * 100.0) / 100.0);
        if (g.satellites_visible != UINT8_MAX)
            s_.satcount = g.satellites_visible;
        // without a fix the velocity is 0 while VFR_HUD keeps the EKF's, so the speed would flicker
        const bool fix = g.fix_type >= 2;
        if (fix && g.vel != UINT16_MAX)
            s_.groundspeed = g.vel * 1.0e-2f;
        if (fix && s_.groundspeed > 0.5 && g.cog != UINT16_MAX)
            s_.groundcourse = wrap360(g.cog * 1.0e-2f);
        if (mavlink2) {
            s_.gps_h_acc = static_cast<float>(g.h_acc) / 1000.0f;
            s_.gps_v_acc = static_cast<float>(g.v_acc) / 1000.0f;
            s_.gps_vel_acc = static_cast<float>(g.vel_acc) / 1000.0f;
            s_.gps_hdg_acc = static_cast<float>(g.hdg_acc) / 1e5f;
            s_.gps_yaw = g.yaw / 100.0f;
        } else {
            s_.gps_h_acc = s_.gps_v_acc = s_.gps_vel_acc = s_.gps_hdg_acc = s_.gps_yaw = -1;
        }
        break;
    }
    case MAVLINK_MSG_ID_GPS_STATUS:
        s_.satcount = read<mavlink_gps_status_t>(f).satellites_visible;
        break;
    case MAVLINK_MSG_ID_RADIO:
    case MAVLINK_MSG_ID_RADIO_STATUS: {
        // the same fields; RADIO is ArduPilot's older copy of RADIO_STATUS
        const auto r = read<mavlink_radio_status_t>(f);
        s_.rssi = r.rssi;
        s_.remrssi = r.remrssi;
        s_.txbuf = r.txbuf;
        s_.rxerrors = r.rxerrors;
        s_.noise = r.noise;
        s_.remnoise = r.remnoise;
        s_.fixed = r.fixed;
        break;
    }
    case MAVLINK_MSG_ID_MISSION_CURRENT:
        s_.wpno = read<mavlink_mission_current_t>(f).seq;
        break;
    case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: {
        const auto n = read<mavlink_nav_controller_output_t>(f);
        s_.nav_roll = n.nav_roll;
        s_.nav_pitch = n.nav_pitch;
        s_.nav_bearing = n.nav_bearing;
        s_.target_bearing = n.target_bearing;
        s_.wp_dist = n.wp_dist;
        s_.alt_error = n.alt_error;
        s_.aspd_error = n.aspd_error / 100.0f;
        s_.xtrack_error = n.xtrack_error;
        break;
    }
    case MAVLINK_MSG_ID_RC_CHANNELS_RAW: {
        const auto rc = read<mavlink_rc_channels_raw_t>(f);
        const uint16_t in[8] = {rc.chan1_raw, rc.chan2_raw, rc.chan3_raw, rc.chan4_raw,
                                rc.chan5_raw, rc.chan6_raw, rc.chan7_raw, rc.chan8_raw};
        std::copy(std::begin(in), std::end(in), s_.rc_in);
        s_.rxrssi = static_cast<int32_t>(rc.rssi / 255.0 * 100.0);
        break;
    }
    case MAVLINK_MSG_ID_RC_CHANNELS: {
        const auto rc = read<mavlink_rc_channels_t>(f);
        const uint16_t in[16] = {rc.chan1_raw,  rc.chan2_raw,  rc.chan3_raw,  rc.chan4_raw,
                                 rc.chan5_raw,  rc.chan6_raw,  rc.chan7_raw,  rc.chan8_raw,
                                 rc.chan9_raw,  rc.chan10_raw, rc.chan11_raw, rc.chan12_raw,
                                 rc.chan13_raw, rc.chan14_raw, rc.chan15_raw, rc.chan16_raw};
        std::copy(std::begin(in), std::end(in), s_.rc_in);
        // 0..254, 255 unknown
        s_.rxrssi = rc.rssi == 255 ? 0 : static_cast<int32_t>(rc.rssi / 254.0 * 100.0);
        break;
    }
    case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW: {
        const auto so = read<mavlink_servo_output_raw_t>(f);
        if (so.port > 1)
            return false;
        const uint16_t out[16] = {so.servo1_raw,  so.servo2_raw,  so.servo3_raw,  so.servo4_raw,
                                  so.servo5_raw,  so.servo6_raw,  so.servo7_raw,  so.servo8_raw,
                                  so.servo9_raw,  so.servo10_raw, so.servo11_raw, so.servo12_raw,
                                  so.servo13_raw, so.servo14_raw, so.servo15_raw, so.servo16_raw};
        std::copy(std::begin(out), std::end(out), s_.servo_out + 16 * so.port);
        break;
    }
    case MAVLINK_MSG_ID_VFR_HUD: {
        const auto v = read<mavlink_vfr_hud_t>(f);
        s_.groundspeed = v.groundspeed;
        s_.airspeed = v.airspeed;
        s_.throttle = v.throttle;
        const uint32_t all = s_.sensors_present & s_.sensors_enabled & s_.sensors_health;
        if ((all & kSensorReverseMotor) != 0 && s_.throttle > 0)
            s_.throttle *= -1;
        s_.climbrate = v.climb;
        s_.got_vfr = 1;
        break;
    }
    case MAVLINK_MSG_ID_VIBRATION: {
        const auto v = read<mavlink_vibration_t>(f);
        s_.vibe_clip[0] = v.clipping_0;
        s_.vibe_clip[1] = v.clipping_1;
        s_.vibe_clip[2] = v.clipping_2;
        s_.vibe_x = v.vibration_x;
        s_.vibe_y = v.vibration_y;
        s_.vibe_z = v.vibration_z;
        break;
    }
    case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
        const auto e = read<mavlink_ekf_status_report_t>(f);
        s_.ekf_velv = e.velocity_variance;
        s_.ekf_compv = e.compass_variance;
        s_.ekf_poshor = e.pos_horiz_variance;
        s_.ekf_posvert = e.pos_vert_variance;
        s_.ekf_teralt = e.terrain_alt_variance;
        s_.ekf_flags = e.flags;
        s_.ekf_status = std::max(s_.ekf_velv,
                                 std::max(s_.ekf_compv, std::max(s_.ekf_poshor, std::max(s_.ekf_posvert, s_.ekf_teralt))));
        // red: no attitude, a GPS but no horizontal velocity, or not initialised
        for (int bit = 1; bit <= kEkfUninitialized; bit <<= 1) {
            const bool set = (e.flags & bit) != 0;
            if (!set && (bit == 1 || (bit == 2 && s_.gps_fix_type > 0)))
                s_.ekf_status = 1;
            if (set && bit == kEkfUninitialized)
                s_.ekf_status = 1;
        }
        break;
    }
    default:
        return false;
    }
    s_.messages++;
    return true;
}

}  // namespace mpcore
