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
constexpr uint8_t kRotationPitch270 = 25;                   // MAV_SENSOR_ROTATION_PITCH_270: facing down

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

// (float)Math.Round(eph / 100.0, 2)
float hdop(uint16_t eph) {
    return static_cast<float>(std::nearbyint(eph / 100.0 * 100.0) / 100.0);
}

void put_imu(mp_vehicle_state &s, int i, float ax, float ay, float az, float gx, float gy, float gz, float mx,
             float my, float mz) {
    s.imu_accel[3 * i] = ax;
    s.imu_accel[3 * i + 1] = ay;
    s.imu_accel[3 * i + 2] = az;
    s.imu_gyro[3 * i] = gx;
    s.imu_gyro[3 * i + 1] = gy;
    s.imu_gyro[3 * i + 2] = gz;
    s.imu_mag[3 * i] = mx;
    s.imu_mag[3 * i + 1] = my;
    s.imu_mag[3 * i + 2] = mz;
}

template <typename Esc>
void put_esc(mp_vehicle_state &s, int first, const Esc &e) {
    for (int i = 0; i < 4; i++) {
        s.esc_voltage[first + i] = e.voltage[i] / 100.0f;
        s.esc_current[first + i] = e.current[i] / 100.0f;
        s.esc_rpm[first + i] = e.rpm[i];
        s.esc_temp[first + i] = e.temperature[i];
    }
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
            s_.gps_hdop = hdop(g.eph);
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
    case MAVLINK_MSG_ID_GPS2_RAW: {
        const auto g = read<mavlink_gps2_raw_t>(f);
        s_.lat2 = g.lat * 1.0e-7;
        s_.lng2 = g.lon * 1.0e-7;
        s_.altasl2 = static_cast<float>(g.alt) / 1000.0f;
        s_.gps_fix_type2 = g.fix_type;
        s_.gps_hdop2 = hdop(g.eph);
        s_.satcount2 = g.satellites_visible;
        s_.groundspeed2 = g.vel * 1.0e-2f;
        s_.groundcourse2 = g.cog * 1.0e-2f;
        if (mavlink2) {
            s_.gps_h_acc2 = static_cast<float>(g.h_acc) / 1000.0f;
            s_.gps_v_acc2 = static_cast<float>(g.v_acc) / 1000.0f;
            s_.gps_vel_acc2 = static_cast<float>(g.vel_acc) / 1000.0f;
            s_.gps_hdg_acc2 = static_cast<float>(g.hdg_acc) / 1e5f;
            s_.gps_yaw2 = g.yaw / 100.0f;
        } else {
            s_.gps_h_acc2 = s_.gps_v_acc2 = s_.gps_vel_acc2 = s_.gps_hdg_acc2 = s_.gps_yaw2 = -1;
        }
        break;
    }
    case MAVLINK_MSG_ID_SCALED_PRESSURE2: {
        const auto p = read<mavlink_scaled_pressure2_t>(f);
        s_.press_abs2 = p.press_abs;
        s_.press_temp2 = p.temperature;
        s_.airspeed2_temp = p.temperature_press_diff;
        break;
    }
    case MAVLINK_MSG_ID_RPM: {
        const auto r = read<mavlink_rpm_t>(f);
        s_.rpm[0] = r.rpm1;
        s_.rpm[1] = r.rpm2;
        break;
    }
    case MAVLINK_MSG_ID_ESC_TELEMETRY_1_TO_4:
        put_esc(s_, 0, read<mavlink_esc_telemetry_1_to_4_t>(f));
        break;
    case MAVLINK_MSG_ID_ESC_TELEMETRY_5_TO_8:
        put_esc(s_, 4, read<mavlink_esc_telemetry_5_to_8_t>(f));
        break;
    case MAVLINK_MSG_ID_ESC_TELEMETRY_9_TO_12:
        put_esc(s_, 8, read<mavlink_esc_telemetry_9_to_12_t>(f));
        break;
    case MAVLINK_MSG_ID_ESC_TELEMETRY_13_TO_16:
        put_esc(s_, 12, read<mavlink_esc_telemetry_13_to_16_t>(f));
        break;
    case MAVLINK_MSG_ID_RAW_IMU: {
        // (CurrentState's log playback speed-up from its time_usec stays in C#)
        const auto m = read<mavlink_raw_imu_t>(f);
        put_imu(s_, 0, m.xacc, m.yacc, m.zacc, m.xgyro, m.ygyro, m.zgyro, m.xmag, m.ymag, m.zmag);
        s_.imu_temp[0] = m.temperature / 100.0f;
        break;
    }
    case MAVLINK_MSG_ID_SCALED_IMU: {
        const auto m = read<mavlink_scaled_imu_t>(f);
        put_imu(s_, 0, m.xacc, m.yacc, m.zacc, m.xgyro, m.ygyro, m.zgyro, m.xmag, m.ymag, m.zmag);
        s_.imu_temp[0] = m.temperature / 100.0f;
        break;
    }
    case MAVLINK_MSG_ID_SCALED_IMU2: {
        const auto m = read<mavlink_scaled_imu2_t>(f);
        put_imu(s_, 1, m.xacc, m.yacc, m.zacc, m.xgyro, m.ygyro, m.zgyro, m.xmag, m.ymag, m.zmag);
        s_.imu_temp[1] = m.temperature / 100.0f;
        break;
    }
    case MAVLINK_MSG_ID_SCALED_IMU3: {
        const auto m = read<mavlink_scaled_imu3_t>(f);
        put_imu(s_, 2, m.xacc, m.yacc, m.zacc, m.xgyro, m.ygyro, m.zgyro, m.xmag, m.ymag, m.zmag);
        s_.imu_temp[2] = m.temperature / 100.0f;
        break;
    }
    case MAVLINK_MSG_ID_HIGHRES_IMU: {
        const auto m = read<mavlink_highres_imu_t>(f);
        if (m.id > 2)
            return false;
        const int i = m.id;
        if ((m.fields_updated & 0x01) != 0) {  // XACC: the three
            s_.imu_accel[3 * i] = m.xacc;
            s_.imu_accel[3 * i + 1] = m.yacc;
            s_.imu_accel[3 * i + 2] = m.zacc;
        }
        if ((m.fields_updated & 0x08) != 0) {  // XGYRO
            s_.imu_gyro[3 * i] = m.xgyro;
            s_.imu_gyro[3 * i + 1] = m.ygyro;
            s_.imu_gyro[3 * i + 2] = m.zgyro;
        }
        if ((m.fields_updated & 0x40) != 0) {  // XMAG
            s_.imu_mag[3 * i] = m.xmag;
            s_.imu_mag[3 * i + 1] = m.ymag;
            s_.imu_mag[3 * i + 2] = m.zmag;
        }
        if ((m.fields_updated & 0x200) != 0)
            s_.press_abs = m.abs_pressure;
        if ((m.fields_updated & 0x1000) != 0)
            s_.press_temp = static_cast<float>(static_cast<int32_t>(m.temperature));
        if ((m.fields_updated & 0x800) != 0)
            s_.altasl = m.pressure_alt;
        break;
    }
    case MAVLINK_MSG_ID_PID_TUNING: {
        const auto p = read<mavlink_pid_tuning_t>(f);
        s_.pid_ff = p.FF;
        s_.pid_p = p.P;
        s_.pid_i = p.I;
        s_.pid_d = p.D;
        s_.pid_axis = p.axis;
        s_.pid_desired = p.desired;
        s_.pid_achieved = p.achieved;
        s_.pid_srate = p.SRate;
        s_.pid_pdmod = p.PDmod;
        if (p.axis >= 1 && p.axis <= 6)  // PID_TUNING_ROLL .. PID_TUNING_LANDING
            s_.pid_srate_axis[p.axis - 1] = p.SRate;
        break;
    }
    case MAVLINK_MSG_ID_HYGROMETER_SENSOR: {
        const auto h = read<mavlink_hygrometer_sensor_t>(f);
        if (h.id > 1)
            return false;
        s_.hygro_temp[h.id] = h.temperature;
        s_.hygro_humi[h.id] = h.humidity;
        break;
    }
    case MAVLINK_MSG_ID_AIRSPEED_AUTOCAL:
        s_.asratio = read<mavlink_airspeed_autocal_t>(f).ratio;
        break;
    case MAVLINK_MSG_ID_AIRSPEED: {
        const auto a = read<mavlink_airspeed_t>(f);
        if (a.id != 0)
            return false;
        s_.airspeed = a.airspeed;
        s_.airspeed1_temp = static_cast<float>(a.temperature / 100.0);
        break;
    }
    case MAVLINK_MSG_ID_SYSTEM_TIME:
        s_.time_unix_usec = read<mavlink_system_time_t>(f).time_unix_usec;
        break;
    case MAVLINK_MSG_ID_HWSTATUS: {
        const auto h = read<mavlink_hwstatus_t>(f);
        s_.hwvoltage = h.Vcc / 1000.0f;
        s_.i2cerrors = h.I2Cerr;
        break;
    }
    case MAVLINK_MSG_ID_RANGEFINDER: {
        const auto r = read<mavlink_rangefinder_t>(f);
        s_.sonarrange = r.distance;
        s_.sonarvoltage = r.voltage;
        rangefinder_index_ = -1;
        break;
    }
    case MAVLINK_MSG_ID_DISTANCE_SENSOR: {
        const auto d = read<mavlink_distance_sensor_t>(f);
        if (d.id < 10)
            s_.rangefinder[d.id] = d.current_distance;
        // the lowest downward-facing sensor gives the rangefinder altitude, in whole metres
        const bool down = d.orientation == kRotationPitch270;
        if (down && d.id < rangefinder_index_)
            rangefinder_index_ = d.id;
        if (d.id == rangefinder_index_) {
            if (!down)
                rangefinder_index_ = INT32_MAX;
            else
                s_.sonarrange = static_cast<float>(d.current_distance / 100);
        }
        break;
    }
    case MAVLINK_MSG_ID_EFI_STATUS: {
        const auto e = read<mavlink_efi_status_t>(f);
        if (e.ecu_index != 0)
            return false;
        s_.efi_baro = e.barometric_pressure;
        s_.efi_headtemp = e.cylinder_head_temperature;
        s_.efi_load = e.engine_load;
        s_.efi_health = e.health;
        s_.efi_exhausttemp = e.exhaust_gas_temperature;
        s_.efi_intaketemp = e.intake_manifold_temperature;
        s_.efi_rpm = e.rpm;
        s_.efi_fuelflow = e.fuel_flow;
        s_.efi_fuelconsumed = e.fuel_consumed;
        s_.efi_fuelpressure = e.fuel_pressure == 0 ? -1 : e.fuel_pressure;
        break;
    }
    case MAVLINK_MSG_ID_POWER_STATUS: {
        const auto p = read<mavlink_power_status_t>(f);
        s_.boardvoltage = p.Vcc;
        s_.servovoltage = p.Vservo;
        s_.voltageflag = p.flags;
        break;
    }
    case MAVLINK_MSG_ID_WIND: {
        const auto w = read<mavlink_wind_t>(f);
        s_.got_wind = 1;
        s_.wind_dir = std::fmod(w.direction + 360, 360.0f);
        s_.wind_speed = w.speed;
        break;
    }
    case MAVLINK_MSG_ID_MEMINFO: {
        const auto m = read<mavlink_meminfo_t>(f);
        s_.freemem = m.freemem32 > 0 ? static_cast<float>(m.freemem32) : m.freemem;
        s_.brklevel = m.brkval;
        break;
    }
    case MAVLINK_MSG_ID_AOA_SSA: {
        const auto a = read<mavlink_aoa_ssa_t>(f);
        s_.aoa = a.AOA;
        s_.ssa = a.SSA;
        break;
    }
    case MAVLINK_MSG_ID_MOUNT_STATUS: {
        const auto m = read<mavlink_mount_status_t>(f);
        s_.campoint[0] = static_cast<float>(m.pointing_a) / 100.0f;
        s_.campoint[1] = static_cast<float>(m.pointing_b) / 100.0f;
        s_.campoint[2] = static_cast<float>(m.pointing_c) / 100.0f;
        break;
    }
    case MAVLINK_MSG_ID_OPTICAL_FLOW: {
        const auto o = read<mavlink_optical_flow_t>(f);
        s_.opt_m_x = o.flow_comp_m_x;
        s_.opt_m_y = o.flow_comp_m_y;
        s_.opt_x = o.flow_x;
        s_.opt_y = o.flow_y;
        s_.opt_qua = o.quality;
        break;
    }
    case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
        const auto l = read<mavlink_local_position_ned_t>(f);
        s_.posn = l.x;
        s_.pose = l.y;
        s_.posd = l.z;
        break;
    }
    case MAVLINK_MSG_ID_FENCE_STATUS: {
        const auto fs = read<mavlink_fence_status_t>(f);
        s_.fenceb_type = fs.breach_type;
        s_.fenceb_status = fs.breach_status;
        s_.fenceb_count = fs.breach_count;
        break;
    }
    case MAVLINK_MSG_ID_GENERATOR_STATUS: {
        const auto g = read<mavlink_generator_status_t>(f);
        s_.gen_status = static_cast<float>(g.status);
        s_.gen_speed = g.generator_speed;
        s_.gen_current = g.load_current;
        s_.gen_voltage = g.bus_voltage;
        s_.gen_runtime = g.runtime;
        s_.gen_maint_time = g.time_until_maintenance;
        break;
    }
    case MAVLINK_MSG_ID_MCU_STATUS: {
        const auto m = read<mavlink_mcu_status_t>(f);
        if (m.id != 0)
            return false;
        s_.mcu_temp = m.MCU_temperature / 100.0f;
        s_.mcu_voltage = m.MCU_voltage / 1000.0f;
        s_.mcu_voltage_min = m.MCU_voltage_min / 1000.0f;
        s_.mcu_voltage_max = m.MCU_voltage_max / 1000.0f;
        break;
    }
    case MAVLINK_MSG_ID_AHRS2: {
        const auto a = read<mavlink_ahrs2_t>(f);
        // unlike ATTITUDE, in float
        const auto deg = static_cast<float>(kRad2Deg);
        s_.ahrs2_roll = a.roll * deg;
        s_.ahrs2_pitch = a.pitch * deg;
        s_.ahrs2_yaw = a.yaw * deg;
        s_.ahrs2_alt = a.altitude;
        s_.ahrs2_lat = a.lat / 1e7;
        s_.ahrs2_lng = a.lng / 1e7;
        break;
    }
    case MAVLINK_MSG_ID_AUTOPILOT_VERSION: {
        const auto v = read<mavlink_autopilot_version_t>(f);
        s_.flight_sw_version = v.flight_sw_version;
        s_.capabilities = static_cast<uint32_t>(v.capabilities);
        s_.uid = v.uid;
        break;
    }
    default:
        return false;
    }
    s_.messages++;
    return true;
}

}  // namespace mpcore
