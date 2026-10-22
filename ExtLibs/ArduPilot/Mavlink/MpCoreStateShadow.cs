using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using log4net;
using MissionPlanner.Mavlink.Native;
using MissionPlanner.Utilities;

namespace MissionPlanner.ArduPilot.Mavlink
{
    /// <summary>
    /// Decodes each message a CurrentState takes with the C++ core too (mp_vehicle) and, after
    /// every message the core decodes, compares the values that message sets with CurrentState's
    /// stored ones (before display units). It only logs. Setting "mpcore_state_shadow"; see
    /// ExtLibs/mpcore/DESIGN.md (phase 2, CurrentState).
    /// </summary>
    public sealed class MpCoreStateShadow : IDisposable
    {
        private static readonly ILog log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);

        const int DetailLogsMax = 50;
        const int SummaryEvery = 10000;

        sealed class Field
        {
            public string Name;
            public Func<CurrentState, double> CSharp;
            public Func<MpCoreNative.mp_vehicle_state, double> Core;
            public Func<MAVLink.MAVLinkMessage, bool> When;  // null: after every such message
        }

        public sealed class Report
        {
            public long Messages;
            public long Compared;
            public long Differences;
            public readonly Dictionary<string, long> ByField = new Dictionary<string, long>();

            public override string ToString()
            {
                return string.Format("messages {0}, values compared {1}, differences {2}{3}", Messages, Compared,
                    Differences,
                    ByField.Count == 0
                        ? ""
                        : " (" + string.Join(", ", ByField.OrderByDescending(a => a.Value).Take(10)
                            .Select(a => a.Key + " " + a.Value)) + ")");
            }
        }

        static readonly Dictionary<uint, Field[]> Fields = BuildFields();

        readonly object _sync = new object();
        readonly Action<string> _log;
        readonly Report _report = new Report();
        MpVehicle _vehicle;
        int _detailLogs;

        public MpCoreStateShadow(uint sysid, byte compid, Action<string> log = null)
        {
            _log = log ?? (s => MpCoreStateShadow.log.Info(s));
            _vehicle = new MpVehicle(sysid, compid);
            _log("mpcore state shadow started for " + sysid + "-" + compid);
        }

        public Report Totals
        {
            get
            {
                lock (_sync)
                    return _report;
            }
        }

        /// <summary>The shadow for a CurrentState, made when the setting is on and the library loads.</summary>
        public static MpCoreStateShadow TryCreate(uint sysid, byte compid)
        {
            if (!Settings.Instance.GetBoolean("mpcore_state_shadow", false) ||
                !MpCoreNativeLoader.Available("state shadow", log.Info))
                return null;
            try
            {
                return new MpCoreStateShadow(sysid, compid);
            }
            catch (Exception ex)
            {
                log.Error("mpcore state shadow", ex);
                return null;
            }
        }

        /// <summary>After CurrentState took the message: the core takes it too and the values are compared.</summary>
        public void Check(CurrentState cs, MAVLink.MAVLinkMessage message)
        {
            if (message?.buffer == null)
                return;
            lock (_sync)
            {
                if (_vehicle == null || !_vehicle.Frame(message.buffer))
                    return;
                _report.Messages++;
                if (!Fields.TryGetValue(message.msgid, out var fields))
                    return;
                var state = _vehicle.State();
                foreach (var field in fields)
                {
                    if (field.When != null && !field.When(message))
                        continue;
                    _report.Compared++;
                    double csharp = field.CSharp(cs), core = field.Core(state);
                    if (csharp.Equals(core))
                        continue;
                    _report.Differences++;
                    _report.ByField.TryGetValue(field.Name, out var n);
                    _report.ByField[field.Name] = n + 1;
                    if (_detailLogs++ < DetailLogsMax)
                        _log(string.Format("mpcore state shadow: {0} after {1}: C# {2:R}, core {3:R}", field.Name,
                            (MAVLink.MAVLINK_MSG_ID) message.msgid, csharp, core));
                }

                if (_report.Messages % SummaryEvery == 0)
                    _log("mpcore state shadow: " + _report);
            }
        }

        public void Dispose()
        {
            lock (_sync)
            {
                if (_vehicle == null)
                    return;
                _log("mpcore state shadow: " + _report);
                _vehicle.Dispose();
                _vehicle = null;
            }
        }

        // CurrentState's stored values: a property, or the field behind one that converts to display units
        static Func<CurrentState, double> Property(string name)
        {
            var p = typeof(CurrentState).GetProperty(name, BindingFlags.Public | BindingFlags.Instance);
            if (p == null)
                throw new MissingMemberException(nameof(CurrentState), name);
            return cs => Convert.ToDouble(p.GetValue(cs));
        }

        static Func<CurrentState, double> Stored(string name)
        {
            var f = typeof(CurrentState).GetField(name, BindingFlags.NonPublic | BindingFlags.Instance);
            if (f == null)
                throw new MissingMemberException(nameof(CurrentState), name);
            return cs => Convert.ToDouble(f.GetValue(cs));
        }

        static Func<CurrentState, double> Flag(string name)
        {
            var p = typeof(CurrentState).GetProperty(name, BindingFlags.Public | BindingFlags.Instance);
            if (p == null)
                throw new MissingMemberException(nameof(CurrentState), name);
            return cs => (bool) p.GetValue(cs) ? 1 : 0;
        }

        static Dictionary<uint, Field[]> BuildFields()
        {
            var all = new Dictionary<uint, List<Field>>();

            Action<MAVLink.MAVLINK_MSG_ID, string, Func<CurrentState, double>, Func<MpCoreNative.mp_vehicle_state, double>>
                Add = (msg, name, csharp, core) =>
                {
                    if (!all.TryGetValue((uint) msg, out var list))
                        all[(uint) msg] = list = new List<Field>();
                    list.Add(new Field { Name = name, CSharp = csharp, Core = core });
                };
            Action<MAVLink.MAVLINK_MSG_ID, string, Func<MpCoreNative.mp_vehicle_state, double>> Same =
                (msg, name, core) => Add(msg, name, Property(name), core);

            try
            {
                var hb = MAVLink.MAVLINK_MSG_ID.HEARTBEAT;
                Add(hb, "armed", Flag("armed"), s => s.armed);
                Add(hb, "landed", Flag("landed"), s => s.landed);
                Add(hb, "failsafe", Flag("failsafe"), s => s.failsafe);
                Add(hb, "_mode", Stored("_mode"), s => s.custom_mode_seen != 0 ? s.custom_mode : 99999);

                var ss = MAVLink.MAVLINK_MSG_ID.SYS_STATUS;
                Same(ss, "load", s => s.load);
                Same(ss, "battery_remaining", s => s.battery_remaining[0]);
                Add(ss, "_current", Stored("_current"), s => s.battery_current[0]);
                Same(ss, "packetdropremote", s => s.drop_rate_comm);
                Same(ss, "errors_count1", s => s.errors_count[0]);
                Same(ss, "errors_count2", s => s.errors_count[1]);
                Add(ss, "sensors_enabled", cs => cs.sensors_enabled.Value, s => s.sensors_enabled);
                Add(ss, "sensors_health", cs => cs.sensors_health.Value, s => s.sensors_health);
                Add(ss, "sensors_present", cs => cs.sensors_present.Value, s => s.sensors_present);

                Same(MAVLink.MAVLINK_MSG_ID.EXTENDED_SYS_STATE, "vtol_state", s => s.vtol_state);
                Same(MAVLink.MAVLINK_MSG_ID.EXTENDED_SYS_STATE, "landed_state", s => s.landed_state);
                Add(MAVLink.MAVLINK_MSG_ID.BATTERY2, "_current2", Stored("_current2"), s => s.battery_current[1]);

                var bat = MAVLink.MAVLINK_MSG_ID.BATTERY_STATUS;
                Add(bat, "_battery_voltage", Stored("_battery_voltage"), s => s.battery_voltage[0]);
                Same(bat, "battery_voltage2", s => s.battery_voltage[1]);
                Same(bat, "battery_cell1", s => s.battery_cells[0]);
                Same(bat, "battery_cell2", s => s.battery_cells[1]);
                // the used mAh only right after the battery's own report: in between both sides integrate
                // SYS_STATUS's and BATTERY2's current against their own clocks
                Func<int, Func<MAVLink.MAVLinkMessage, bool>> battery = id =>
                    m => m.ToStructure<MAVLink.mavlink_battery_status_t>().id == id;
                Same(bat, "battery_usedmah", s => s.battery_used_mah[0]);
                all[(uint) bat].Last().When = battery(0);
                Same(bat, "battery_usedmah2", s => s.battery_used_mah[1]);
                all[(uint) bat].Last().When = battery(1);
                Same(bat, "battery_remaining2", s => s.battery_remaining[1]);
                Same(bat, "battery_temp", s => s.battery_temp[0]);
                Same(bat, "battery_remainmin", s => s.battery_remain_min[0]);

                var att = MAVLink.MAVLINK_MSG_ID.ATTITUDE;
                Same(att, "roll", s => s.roll);
                Same(att, "pitch", s => s.pitch);
                Same(att, "yaw", s => s.yaw);

                var pos = MAVLink.MAVLINK_MSG_ID.GLOBAL_POSITION_INT;
                Add(pos, "_alt", Stored("_alt"), s => s.alt);
                Add(pos, "_altasl", Stored("_altasl"), s => s.altasl);
                Same(pos, "lat", s => s.lat);
                Same(pos, "lng", s => s.lng);
                Same(pos, "vx", s => s.vx);
                Same(pos, "vz", s => s.vz);

                var gps = MAVLink.MAVLINK_MSG_ID.GPS_RAW_INT;
                Same(gps, "lat", s => s.lat);
                Same(gps, "lng", s => s.lng);
                Add(gps, "_altasl", Stored("_altasl"), s => s.altasl);
                Same(gps, "gpsstatus", s => s.gps_fix_type);
                Same(gps, "gpshdop", s => s.gps_hdop);
                Same(gps, "satcount", s => s.satcount);
                Add(gps, "_groundspeed", Stored("_groundspeed"), s => s.groundspeed);
                Same(gps, "groundcourse", s => s.groundcourse);
                Same(gps, "gpsh_acc", s => s.gps_h_acc);
                Same(gps, "gpsyaw", s => s.gps_yaw);

                foreach (var radio in new[] { MAVLink.MAVLINK_MSG_ID.RADIO, MAVLink.MAVLINK_MSG_ID.RADIO_STATUS })
                {
                    Same(radio, "rssi", s => s.rssi);
                    Same(radio, "remrssi", s => s.remrssi);
                    Same(radio, "rxerrors", s => s.rxerrors);
                    Same(radio, "fixedp", s => s.@fixed);
                }

                Same(MAVLink.MAVLINK_MSG_ID.MISSION_CURRENT, "wpno", s => s.wpno);

                var nav = MAVLink.MAVLINK_MSG_ID.NAV_CONTROLLER_OUTPUT;
                Same(nav, "nav_roll", s => s.nav_roll);
                Same(nav, "nav_bearing", s => s.nav_bearing);
                Same(nav, "target_bearing", s => s.target_bearing);
                Add(nav, "_wpdist", Stored("_wpdist"), s => s.wp_dist);
                Add(nav, "_alt_error", Stored("_alt_error"), s => s.alt_error);
                Add(nav, "_aspd_error", Stored("_aspd_error"), s => s.aspd_error);
                Same(nav, "xtrack_error", s => s.xtrack_error);

                foreach (var rc in new[] { MAVLink.MAVLINK_MSG_ID.RC_CHANNELS, MAVLink.MAVLINK_MSG_ID.RC_CHANNELS_RAW })
                {
                    Same(rc, "ch1in", s => s.rc_in[0]);
                    Same(rc, "ch8in", s => s.rc_in[7]);
                    Same(rc, "rxrssi", s => s.rxrssi);
                }
                Same(MAVLink.MAVLINK_MSG_ID.RC_CHANNELS, "ch16in", s => s.rc_in[15]);

                var servo = MAVLink.MAVLINK_MSG_ID.SERVO_OUTPUT_RAW;
                Same(servo, "ch1out", s => s.servo_out[0]);
                Same(servo, "ch8out", s => s.servo_out[7]);
                Same(servo, "ch16out", s => s.servo_out[15]);
                Same(servo, "ch17out", s => s.servo_out[16]);

                var hud = MAVLink.MAVLINK_MSG_ID.VFR_HUD;
                Add(hud, "_groundspeed", Stored("_groundspeed"), s => s.groundspeed);
                Add(hud, "_airspeed", Stored("_airspeed"), s => s.airspeed);
                Add(hud, "_ch3percent", Stored("_ch3percent"), s => s.throttle);
                Add(hud, "_climbrate", Stored("_climbrate"), s => s.climbrate);

                var vibe = MAVLink.MAVLINK_MSG_ID.VIBRATION;
                Same(vibe, "vibex", s => s.vibe_x);
                Same(vibe, "vibez", s => s.vibe_z);
                Same(vibe, "vibeclip0", s => s.vibe_clip[0]);

                var ekf = MAVLink.MAVLINK_MSG_ID.EKF_STATUS_REPORT;
                Same(ekf, "ekfstatus", s => s.ekf_status);
                Same(ekf, "ekfflags", s => s.ekf_flags);
                Same(ekf, "ekfcompv", s => s.ekf_compv);

                var press = MAVLink.MAVLINK_MSG_ID.SCALED_PRESSURE;
                Same(press, "press_abs", s => s.press_abs);
                Same(press, "press_temp", s => s.press_temp);

                var ter = MAVLink.MAVLINK_MSG_ID.TERRAIN_REPORT;
                Add(ter, "_ter_curalt", Stored("_ter_curalt"), s => s.ter_curalt);
                Add(ter, "_ter_alt", Stored("_ter_alt"), s => s.ter_alt);
                Same(ter, "ter_load", s => s.ter_load);
                Same(ter, "ter_pend", s => s.ter_pend);

                var press2 = MAVLink.MAVLINK_MSG_ID.SCALED_PRESSURE2;
                Same(press2, "press_abs2", s => s.press_abs2);
                Same(press2, "press_temp2", s => s.press_temp2);
                Same(press2, "airspeed2_temp", s => s.airspeed2_temp);

                var gps2 = MAVLink.MAVLINK_MSG_ID.GPS2_RAW;
                Same(gps2, "lat2", s => s.lat2);
                Same(gps2, "lng2", s => s.lng2);
                Same(gps2, "altasl2", s => s.altasl2);
                Same(gps2, "gpsstatus2", s => s.gps_fix_type2);
                Same(gps2, "gpshdop2", s => s.gps_hdop2);
                Same(gps2, "satcount2", s => s.satcount2);
                Same(gps2, "groundspeed2", s => s.groundspeed2);
                Same(gps2, "groundcourse2", s => s.groundcourse2);
                Same(gps2, "gpsh_acc2", s => s.gps_h_acc2);
                Same(gps2, "gpsyaw2", s => s.gps_yaw2);

                Same(MAVLink.MAVLINK_MSG_ID.RPM, "rpm1", s => s.rpm[0]);
                Same(MAVLink.MAVLINK_MSG_ID.RPM, "rpm2", s => s.rpm[1]);

                var escs = new[]
                {
                    MAVLink.MAVLINK_MSG_ID.ESC_TELEMETRY_1_TO_4, MAVLink.MAVLINK_MSG_ID.ESC_TELEMETRY_5_TO_8,
                    MAVLink.MAVLINK_MSG_ID.ESC_TELEMETRY_9_TO_12, MAVLink.MAVLINK_MSG_ID.ESC_TELEMETRY_13_TO_16
                };
                for (int i = 0; i < 16; i++)
                {
                    int esc = i;
                    var msg = escs[i / 4];
                    Same(msg, "esc" + (i + 1) + "_volt", s => s.esc_voltage[esc]);
                    Same(msg, "esc" + (i + 1) + "_curr", s => s.esc_current[esc]);
                    Same(msg, "esc" + (i + 1) + "_rpm", s => s.esc_rpm[esc]);
                    Same(msg, "esc" + (i + 1) + "_temp", s => s.esc_temp[esc]);
                }

                // the IMUs: RAW_IMU and SCALED_IMU are the first, HIGHRES_IMU any of three
                var imus = new[]
                {
                    new[] { MAVLink.MAVLINK_MSG_ID.RAW_IMU, MAVLink.MAVLINK_MSG_ID.SCALED_IMU, MAVLink.MAVLINK_MSG_ID.HIGHRES_IMU },
                    new[] { MAVLink.MAVLINK_MSG_ID.SCALED_IMU2, MAVLink.MAVLINK_MSG_ID.HIGHRES_IMU },
                    new[] { MAVLink.MAVLINK_MSG_ID.SCALED_IMU3, MAVLink.MAVLINK_MSG_ID.HIGHRES_IMU },
                };
                for (int imu = 0; imu < 3; imu++)
                {
                    var suffix = imu == 0 ? "" : (imu + 1).ToString();
                    foreach (var msg in imus[imu])
                    {
                        for (int axis = 0; axis < 3; axis++)
                        {
                            int i = imu * 3 + axis;
                            var name = "xyz"[axis] + suffix;
                            Same(msg, "a" + name, s => s.imu_accel[i]);
                            Same(msg, "g" + name, s => s.imu_gyro[i]);
                            Same(msg, "m" + name, s => s.imu_mag[i]);
                        }

                        int t = imu;
                        if (msg != MAVLink.MAVLINK_MSG_ID.HIGHRES_IMU)
                            Same(msg, "imu" + (imu + 1) + "_temp", s => s.imu_temp[t]);
                    }
                }
                var highres = MAVLink.MAVLINK_MSG_ID.HIGHRES_IMU;
                Same(highres, "press_abs", s => s.press_abs);
                Same(highres, "press_temp", s => s.press_temp);
                Add(highres, "_altasl", Stored("_altasl"), s => s.altasl);

                var pid = MAVLink.MAVLINK_MSG_ID.PID_TUNING;
                Same(pid, "pidff", s => s.pid_ff);
                Same(pid, "pidP", s => s.pid_p);
                Same(pid, "pidI", s => s.pid_i);
                Same(pid, "pidD", s => s.pid_d);
                Same(pid, "pidaxis", s => s.pid_axis);
                Same(pid, "piddesired", s => s.pid_desired);
                Same(pid, "pidachieved", s => s.pid_achieved);
                Same(pid, "pidSRate", s => s.pid_srate);
                Same(pid, "pidPDmod", s => s.pid_pdmod);
                var axes = new[] { "Roll", "Pitch", "Yaw", "AccZ", "Steer", "Landing" };
                for (int i = 0; i < axes.Length; i++)
                {
                    int axis = i;
                    Same(pid, "pidSRate" + axes[i], s => s.pid_srate_axis[axis]);
                }

                var hygro = MAVLink.MAVLINK_MSG_ID.HYGROMETER_SENSOR;
                Same(hygro, "hygrotemp1", s => s.hygro_temp[0]);
                Same(hygro, "hygrohumi1", s => s.hygro_humi[0]);
                Same(hygro, "hygrotemp2", s => s.hygro_temp[1]);
                Same(hygro, "hygrohumi2", s => s.hygro_humi[1]);

                Same(MAVLink.MAVLINK_MSG_ID.AIRSPEED_AUTOCAL, "asratio", s => s.asratio);
                Add(MAVLink.MAVLINK_MSG_ID.AIRSPEED, "_airspeed", Stored("_airspeed"), s => s.airspeed);
                Same(MAVLink.MAVLINK_MSG_ID.AIRSPEED, "airspeed1_temp", s => s.airspeed1_temp);

                var epoch = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);
                Add(MAVLink.MAVLINK_MSG_ID.SYSTEM_TIME, "gpstime", cs => (cs.gpstime - epoch).Ticks / 10000,
                    s => s.time_unix_usec / 1000);

                Same(MAVLink.MAVLINK_MSG_ID.HWSTATUS, "hwvoltage", s => s.hwvoltage);
                Same(MAVLink.MAVLINK_MSG_ID.HWSTATUS, "i2cerrors", s => s.i2cerrors);

                Add(MAVLink.MAVLINK_MSG_ID.RANGEFINDER, "_sonarrange", Stored("_sonarrange"), s => s.sonarrange);
                Same(MAVLink.MAVLINK_MSG_ID.RANGEFINDER, "sonarvoltage", s => s.sonarvoltage);
                var distance = MAVLink.MAVLINK_MSG_ID.DISTANCE_SENSOR;
                Add(distance, "_sonarrange", Stored("_sonarrange"), s => s.sonarrange);
                for (int i = 0; i < 10; i++)
                {
                    int id = i;
                    Same(distance, "rangefinder" + (i + 1), s => s.rangefinder[id]);
                }

                var efi = MAVLink.MAVLINK_MSG_ID.EFI_STATUS;
                Same(efi, "efi_baro", s => s.efi_baro);
                Same(efi, "efi_headtemp", s => s.efi_headtemp);
                Same(efi, "efi_load", s => s.efi_load);
                Same(efi, "efi_health", s => s.efi_health);
                Same(efi, "efi_exhasttemp", s => s.efi_exhausttemp);
                Same(efi, "efi_intaketemp", s => s.efi_intaketemp);
                Same(efi, "efi_rpm", s => s.efi_rpm);
                Same(efi, "efi_fuelflow", s => s.efi_fuelflow);
                Same(efi, "efi_fuelconsumed", s => s.efi_fuelconsumed);
                Same(efi, "efi_fuelpressure", s => s.efi_fuelpressure);

                var power = MAVLink.MAVLINK_MSG_ID.POWER_STATUS;
                Same(power, "boardvoltage", s => s.boardvoltage);
                Same(power, "servovoltage", s => s.servovoltage);
                Same(power, "voltageflag", s => s.voltageflag);

                // CurrentState keeps the wind speed in display units
                Same(MAVLink.MAVLINK_MSG_ID.WIND, "wind_dir", s => s.wind_dir);
                Same(MAVLink.MAVLINK_MSG_ID.WIND, "wind_vel", s => s.wind_speed * CurrentState.multiplierspeed);

                Same(MAVLink.MAVLINK_MSG_ID.MEMINFO, "freemem", s => s.freemem);
                Same(MAVLink.MAVLINK_MSG_ID.MEMINFO, "brklevel", s => s.brklevel);
                Same(MAVLink.MAVLINK_MSG_ID.AOA_SSA, "AOA", s => s.aoa);
                Same(MAVLink.MAVLINK_MSG_ID.AOA_SSA, "SSA", s => s.ssa);

                var mount = MAVLink.MAVLINK_MSG_ID.MOUNT_STATUS;
                Same(mount, "campointa", s => s.campoint[0]);
                Same(mount, "campointb", s => s.campoint[1]);
                Same(mount, "campointc", s => s.campoint[2]);

                var flow = MAVLink.MAVLINK_MSG_ID.OPTICAL_FLOW;
                Same(flow, "opt_m_x", s => s.opt_m_x);
                Same(flow, "opt_m_y", s => s.opt_m_y);
                Same(flow, "opt_x", s => s.opt_x);
                Same(flow, "opt_y", s => s.opt_y);
                Same(flow, "opt_qua", s => s.opt_qua);

                var local = MAVLink.MAVLINK_MSG_ID.LOCAL_POSITION_NED;
                Same(local, "posn", s => s.posn);
                Same(local, "pose", s => s.pose);
                Same(local, "posd", s => s.posd);

                var fence = MAVLink.MAVLINK_MSG_ID.FENCE_STATUS;
                Same(fence, "fenceb_type", s => s.fenceb_type);
                Same(fence, "fenceb_status", s => s.fenceb_status);
                Same(fence, "fenceb_count", s => s.fenceb_count);

                var gen = MAVLink.MAVLINK_MSG_ID.GENERATOR_STATUS;
                Same(gen, "gen_status", s => s.gen_status);
                Same(gen, "gen_speed", s => s.gen_speed);
                Same(gen, "gen_current", s => s.gen_current);
                Same(gen, "gen_voltage", s => s.gen_voltage);
                Same(gen, "gen_runtime", s => s.gen_runtime);
                Same(gen, "gen_maint_time", s => s.gen_maint_time);

                var mcu = MAVLink.MAVLINK_MSG_ID.MCU_STATUS;
                Same(mcu, "mcutemp", s => s.mcu_temp);
                Same(mcu, "mcuvoltage", s => s.mcu_voltage);
                Same(mcu, "mcuminvolt", s => s.mcu_voltage_min);
                Same(mcu, "mcumaxvolt", s => s.mcu_voltage_max);

                var ahrs2 = MAVLink.MAVLINK_MSG_ID.AHRS2;
                Same(ahrs2, "ahrs2_roll", s => s.ahrs2_roll);
                Same(ahrs2, "ahrs2_pitch", s => s.ahrs2_pitch);
                Same(ahrs2, "ahrs2_yaw", s => s.ahrs2_yaw);
                Same(ahrs2, "ahrs2_alt", s => s.ahrs2_alt);
                Same(ahrs2, "ahrs2_lat", s => s.ahrs2_lat);
                Same(ahrs2, "ahrs2_lng", s => s.ahrs2_lng);

                Same(MAVLink.MAVLINK_MSG_ID.AUTOPILOT_VERSION, "capabilities", s => s.capabilities);
                Same(MAVLink.MAVLINK_MSG_ID.AUTOPILOT_VERSION, "uid", s => s.uid);
            }
            catch (MissingMemberException ex)
            {
                // CurrentState changed under the shadow: nothing is compared rather than half of it
                log.Error("mpcore state shadow: " + ex.Message);
                return new Dictionary<uint, Field[]>();
            }

            return all.ToDictionary(a => a.Key, a => a.Value.ToArray());
        }
    }
}
