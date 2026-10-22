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
