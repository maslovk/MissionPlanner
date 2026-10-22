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

        // A vehicle taking an upload: it loses the first count and its request for item 5 once (so
        // item 4 is sent again), and acknowledges the last item.
        sealed class Receiver
        {
            readonly MpCoreTestSerial _serial;
            readonly MAVLink.MavlinkParse _encoder = new MAVLink.MavlinkParse();
            readonly bool _asInt;
            int _total = -1;
            bool _lostCount, _lostRequest;
            public readonly Dictionary<int, Locationwp> Items = new Dictionary<int, Locationwp>();
            public int Counts, ItemsReceived, Acks;

            public Receiver(MpCoreTestSerial serial, bool asInt)
            {
                _serial = serial;
                _asInt = asInt;
                serial.OnWrite = Receive;
            }

            void Request(int seq)
            {
                if (seq == 5 && !_lostRequest)
                {
                    _lostRequest = true;
                    return;
                }
                if (_asInt)
                {
                    var req = new MAVLink.mavlink_mission_request_int_t { seq = (ushort)seq, target_system = 255, target_component = 190 };
                    _serial.Feed(_encoder.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.MISSION_REQUEST_INT, req, false, 1, 1));
                }
                else
                {
                    var req = new MAVLink.mavlink_mission_request_t { seq = (ushort)seq, target_system = 255, target_component = 190 };
                    _serial.Feed(_encoder.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.MISSION_REQUEST, req, false, 1, 1));
                }
            }

            void Receive(byte[] frame)
            {
                var msg = new MAVLink.MAVLinkMessage(frame);
                if (msg.msgid == (uint)MAVLink.MAVLINK_MSG_ID.MISSION_COUNT)
                {
                    Counts++;
                    if (!_lostCount)
                    {
                        _lostCount = true;
                        return;
                    }
                    _total = msg.ToStructure<MAVLink.mavlink_mission_count_t>().count;
                    Items.Clear();
                    Request(0);
                    return;
                }

                int seq;
                Locationwp loc;
                if (msg.msgid == (uint)MAVLink.MAVLINK_MSG_ID.MISSION_ITEM_INT)
                {
                    var item = msg.ToStructure<MAVLink.mavlink_mission_item_int_t>();
                    seq = item.seq;
                    loc = item;
                }
                else if (msg.msgid == (uint)MAVLink.MAVLINK_MSG_ID.MISSION_ITEM)
                {
                    var item = msg.ToStructure<MAVLink.mavlink_mission_item_t>();
                    seq = item.seq;
                    loc = item;
                }
                else
                {
                    return;
                }

                ItemsReceived++;
                Items[seq] = loc;
                if (seq + 1 < _total)
                {
                    Request(seq + 1);
                }
                else
                {
                    Acks++;
                    var ack = new MAVLink.mavlink_mission_ack_t { type = (byte)MAVLink.MAV_MISSION_RESULT.MAV_MISSION_ACCEPTED, target_system = 255, target_component = 190 };
                    _serial.Feed(_encoder.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.MISSION_ACK, ack, false, 1, 1));
                }
            }
        }

        static List<Locationwp> Mission()
        {
            var list = new List<Locationwp>();
            for (int seq = 0; seq < Count; seq++)
            {
                list.Add(new Locationwp
                {
                    id = seq % 4 == 3 ? (ushort)MAVLink.MAV_CMD.DO_CHANGE_SPEED : (ushort)MAVLink.MAV_CMD.WAYPOINT,
                    frame = (byte)MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT,
                    p1 = seq + 0.5f, p2 = 2, p3 = 3, p4 = 4,
                    lat = -35.363261 + seq * 0.0001, lng = 149.16523 + seq * 0.0002, alt = 50 + seq,
                });
            }
            return list;
        }

        static (Receiver vehicle, Dictionary<int, Locationwp> kept) Upload(bool native, bool missionInt)
        {
            Settings.Instance["mpcore_missions"] = native.ToString();
            using (var port = new MAVLinkInterface { printbps = false })
            {
                var serial = new MpCoreTestSerial();
                port.BaseStream = serial;
                var vehicle = new Receiver(serial, missionInt);
                if (missionInt)
                    port.MAVlist[1, 1].cs.capabilities |= (uint)MAVLink.MAV_PROTOCOL_CAPABILITY.MISSION_INT;
                mav_mission.upload(port, 1, 1, MAVLink.MAV_MISSION_TYPE.MISSION, Mission()).GetAwaiter().GetResult();
                var kept = new Dictionary<int, Locationwp>();
                foreach (var wp in port.MAVlist[1, 1].wps)
                    kept[wp.Key] = wp.Value;
                return (vehicle, kept);
            }
        }

        static void AssertSame(Locationwp c, Locationwp n, string what)
        {
            Assert.AreEqual(c.id, n.id, what);
            Assert.AreEqual(c.frame, n.frame, what);
            Assert.AreEqual(c.p1, n.p1, what);
            Assert.AreEqual(c.p4, n.p4, what);
            Assert.AreEqual(c.lat, n.lat, what);
            Assert.AreEqual(c.lng, n.lng, what);
            Assert.AreEqual(c.alt, n.alt, what);
        }

        [TestMethod]
        public void NativeMissionUploadSendsTheSameItems()
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
                    var csharp = Upload(false, missionInt);
                    var native = Upload(true, missionInt);
                    string how = missionInt ? "MISSION_ITEM_INT" : "MISSION_ITEM";
                    foreach (var v in new[] { csharp.vehicle, native.vehicle })
                    {
                        Assert.AreEqual(Count, v.Items.Count, how);
                        Assert.AreEqual(2, v.Counts, how + ": the lost count sent again");
                        Assert.AreEqual(Count + 1, v.ItemsReceived, how + ": item 4 sent again");
                        Assert.AreEqual(1, v.Acks, how + ": one ACK");
                    }

                    Assert.AreEqual(Count, csharp.kept.Count, how);
                    Assert.AreEqual(Count, native.kept.Count, how);
                    for (int i = 0; i < Count; i++)
                    {
                        AssertSame(csharp.vehicle.Items[i], native.vehicle.Items[i], how + " item " + i);
                        AssertSame(csharp.kept[i], native.kept[i], how + " kept item " + i);
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
