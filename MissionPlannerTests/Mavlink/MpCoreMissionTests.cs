using System;
using System.Collections.Generic;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using MissionPlanner;
using MissionPlanner.ArduPilot;
using MissionPlanner.ArduPilot.Mavlink;
using MissionPlanner.Utilities;

namespace MissionPlannerTests.Mavlink
{
    /// <summary>
    /// mav_mission.download with the mpcore_missions setting (the C++ core running the count, the
    /// items and the ACK) against the C# loops, on a simulated vehicle that loses the first count
    /// and answers one request with the wrong item. Needs the mpcore native library like
    /// MpCoreReceiveTests.
    /// </summary>
    [TestClass]
    public class MpCoreMissionTests
    {
        const int Count = 12;

        sealed class Vehicle
        {
            readonly MpCoreTestSerial _serial;
            readonly MAVLink.MavlinkParse _encoder = new MAVLink.MavlinkParse();
            int _counts;
            bool _wrongOnce = true;
            public int Lists, Requests, Acks;

            public Vehicle(MpCoreTestSerial serial)
            {
                _serial = serial;
                serial.OnWrite = Receive;
            }

            // waypoints, with a speed change (not a location: its x/y are plain numbers) every fourth
            static ushort Command(int seq) => seq % 4 == 3 ? (ushort)MAVLink.MAV_CMD.DO_CHANGE_SPEED : (ushort)MAVLink.MAV_CMD.WAYPOINT;

            void SendItem(int seq, bool asInt)
            {
                if (asInt)
                {
                    var item = new MAVLink.mavlink_mission_item_int_t
                    {
                        param1 = seq + 0.5f, param2 = 2, param3 = 3, param4 = 4,
                        x = -353632610 + seq * 1000, y = 1491652300 + seq * 2000, z = 50 + seq,
                        seq = (ushort)seq, command = Command(seq), target_system = 255, target_component = 190,
                        frame = (byte)MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT, autocontinue = 1,
                    };
                    _serial.Feed(_encoder.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.MISSION_ITEM_INT, item, false, 1, 1));
                }
                else
                {
                    var item = new MAVLink.mavlink_mission_item_t
                    {
                        param1 = seq + 0.5f, param2 = 2, param3 = 3, param4 = 4,
                        x = -35.363261f + seq * 0.0001f, y = 149.16523f + seq * 0.0002f, z = 50 + seq,
                        seq = (ushort)seq, command = Command(seq), target_system = 255, target_component = 190,
                        frame = (byte)MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT, autocontinue = 1,
                    };
                    _serial.Feed(_encoder.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.MISSION_ITEM, item, false, 1, 1));
                }
            }

            void Receive(byte[] frame)
            {
                var msg = new MAVLink.MAVLinkMessage(frame);
                if (msg.msgid == (uint)MAVLink.MAVLINK_MSG_ID.MISSION_REQUEST_LIST)
                {
                    Lists++;
                    if (_counts++ == 0)
                        return; // the first count is lost
                    var count = new MAVLink.mavlink_mission_count_t { count = Count, target_system = 255, target_component = 190 };
                    _serial.Feed(_encoder.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.MISSION_COUNT, count, false, 1, 1));
                }
                else if (msg.msgid == (uint)MAVLink.MAVLINK_MSG_ID.MISSION_REQUEST_INT ||
                         msg.msgid == (uint)MAVLink.MAVLINK_MSG_ID.MISSION_REQUEST)
                {
                    Requests++;
                    bool asInt = msg.msgid == (uint)MAVLink.MAVLINK_MSG_ID.MISSION_REQUEST_INT;
                    int seq = asInt
                        ? msg.ToStructure<MAVLink.mavlink_mission_request_int_t>().seq
                        : msg.ToStructure<MAVLink.mavlink_mission_request_t>().seq;
                    if (seq == 5 && _wrongOnce)
                    {
                        _wrongOnce = false;
                        seq = 4; // a stale answer
                    }
                    SendItem(seq, asInt);
                }
                else if (msg.msgid == (uint)MAVLink.MAVLINK_MSG_ID.MISSION_ACK)
                {
                    Acks++;
                }
            }
        }

        static (List<Locationwp> items, Vehicle vehicle) Download(bool native, bool missionInt)
        {
            Settings.Instance["mpcore_missions"] = native.ToString();
            using (var port = new MAVLinkInterface { printbps = false })
            {
                var serial = new MpCoreTestSerial();
                port.BaseStream = serial;
                var vehicle = new Vehicle(serial);
                if (missionInt)
                    port.MAVlist[1, 1].cs.capabilities |= (uint)MAVLink.MAV_PROTOCOL_CAPABILITY.MISSION_INT;
                var items = mav_mission.download(port, 1, 1, MAVLink.MAV_MISSION_TYPE.MISSION).GetAwaiter().GetResult();
                return (items, vehicle);
            }
        }

        [TestMethod]
        public void NativeMissionDownloadGetsTheSameItems()
        {
            if (!MpCoreNativeLoader.Available("test", null))
            {
                const string why = "the mpcore native library is not available";
                if (Environment.GetEnvironmentVariable("MPCORE_REQUIRED") == "1")
                    Assert.Fail(why);
                Assert.Inconclusive(why);
            }

            string oldSetting = Settings.Instance["mpcore_missions"];
            try
            {
                foreach (bool missionInt in new[] { true, false })
                {
                    var csharp = Download(false, missionInt);
                    var native = Download(true, missionInt);
                    string how = missionInt ? "MISSION_ITEM_INT" : "MISSION_ITEM";
                    Assert.AreEqual(Count, csharp.items.Count, how);
                    Assert.AreEqual(Count, native.items.Count, how);
                    for (int i = 0; i < Count; i++)
                    {
                        var c = csharp.items[i];
                        var n = native.items[i];
                        Assert.AreEqual(c.id, n.id, how + " item " + i);
                        Assert.AreEqual(c.frame, n.frame, how + " item " + i);
                        Assert.AreEqual(c.p1, n.p1, how + " item " + i);
                        Assert.AreEqual(c.p4, n.p4, how + " item " + i);
                        Assert.AreEqual(c.lat, n.lat, how + " item " + i);
                        Assert.AreEqual(c.lng, n.lng, how + " item " + i);
                        Assert.AreEqual(c.alt, n.alt, how + " item " + i);
                    }

                    foreach (var v in new[] { csharp.vehicle, native.vehicle })
                    {
                        Assert.AreEqual(2, v.Lists, how + ": the lost count asked again");
                        Assert.AreEqual(Count + 1, v.Requests, how + ": the stale item asked again");
                        Assert.AreEqual(1, v.Acks, how + ": one ACK");
                    }
                }
            }
            finally
            {
                Settings.Instance["mpcore_missions"] = oldSetting ?? "False";
            }
        }
    }
}
