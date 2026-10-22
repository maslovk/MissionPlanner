using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using MissionPlanner;
using MissionPlanner.ArduPilot.Mavlink;
using MissionPlanner.Utilities;

namespace MissionPlannerTests.Mavlink
{
    /// <summary>
    /// MAVLinkInterface.getParamListAsync with the mpcore_params setting (the C++ core keeping count
    /// and deciding the retries) against its C# loop, on a simulated vehicle that loses part of the
    /// first list. Needs the mpcore native library like MpCoreReceiveTests.
    /// </summary>
    [TestClass]
    public class MpCoreParamsTests
    {
        const int Count = 150;

        // Answers PARAM_REQUEST_LIST (dropping every fifth parameter the first time),
        // PARAM_REQUEST_READ and PARAM_SET with PARAM_VALUE frames.
        sealed class Vehicle
        {
            readonly MpCoreTestSerial _serial;
            readonly MAVLink.MavlinkParse _encoder = new MAVLink.MavlinkParse();
            readonly float[] _values = Enumerable.Range(0, Count).Select(i => i * 0.25f - 3).ToArray();
            int _lists;
            public int Lists, Singles, Sets;
            public int IgnoreSets;          // this many PARAM_SETs get no answer
            public bool Silent;             // nothing gets an answer

            public Vehicle(MpCoreTestSerial serial)
            {
                _serial = serial;
                serial.OnWrite = Receive;
            }

            static string Name(byte[] id)
            {
                var s = Encoding.ASCII.GetString(id);
                int nul = s.IndexOf('\0');
                return nul >= 0 ? s.Substring(0, nul) : s;
            }

            void Send(int index)
            {
                if (Silent)
                    return;
                var name = new byte[16];
                Encoding.ASCII.GetBytes("PARAM_" + index).CopyTo(name, 0);
                var value = new MAVLink.mavlink_param_value_t
                {
                    param_id = name,
                    param_value = _values[index],
                    param_type = (byte)MAVLink.MAV_PARAM_TYPE.REAL32,
                    param_count = Count,
                    param_index = (ushort)index,
                };
                _serial.Feed(_encoder.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.PARAM_VALUE, value, false, 1, 1));
            }

            void Receive(byte[] frame)
            {
                var msg = new MAVLink.MAVLinkMessage(frame);
                if (msg.msgid == (uint)MAVLink.MAVLINK_MSG_ID.PARAM_REQUEST_LIST)
                {
                    Lists++;
                    bool first = _lists++ == 0;
                    for (int i = 0; i < Count; i++)
                        if (!first || i % 5 != 1)
                            Send(i);
                }
                else if (msg.msgid == (uint)MAVLink.MAVLINK_MSG_ID.PARAM_REQUEST_READ)
                {
                    Singles++;
                    var req = msg.ToStructure<MAVLink.mavlink_param_request_read_t>();
                    int index = req.param_index >= 0 ? req.param_index : Array.IndexOf(Names, Name(req.param_id));
                    if (index >= 0 && index < Count)
                        Send(index);
                }
                else if (msg.msgid == (uint)MAVLink.MAVLINK_MSG_ID.PARAM_SET)
                {
                    Sets++;
                    var set = msg.ToStructure<MAVLink.mavlink_param_set_t>();
                    int index = Array.IndexOf(Names, Name(set.param_id));
                    if (index < 0 || Sets <= IgnoreSets)
                        return;
                    _values[index] = set.param_value;
                    Send(index);
                }
            }
        }

        static readonly string[] Names = Enumerable.Range(0, Count).Select(i => "PARAM_" + i).ToArray();

        static (List<MAVLink.MAVLinkParam> list, Vehicle vehicle) Download(bool native)
        {
            Settings.Instance["mpcore_params"] = native.ToString();
            using (var port = new MAVLinkInterface { printbps = false })
            {
                var serial = new MpCoreTestSerial();
                port.BaseStream = serial;
                var vehicle = new Vehicle(serial);
                port.MAVlist[1, 1].apname = MAVLink.MAV_AUTOPILOT.ARDUPILOTMEGA;
                var list = port.getParamListAsync(1, 1).GetAwaiter().GetResult();
                return (list.ToList(), vehicle);
            }
        }

        static void RequireNative()
        {
            if (!MpCoreNativeLoader.Available("test", null))
            {
                const string why = "the mpcore native library is not available";
                if (Environment.GetEnvironmentVariable("MPCORE_REQUIRED") == "1")
                    Assert.Fail(why);
                Assert.Inconclusive(why);
            }
        }

        sealed class SetAndRead
        {
            public float AfterSet, ByName, ByIndex;
            public int Sets, Reads;
            public bool SetTimedOut, ReadTimedOut;
        }

        // setParamAsync against a vehicle that ignores the first two PARAM_SETs, then reads by name
        // and by index, then a set and a read the vehicle never answers.
        static SetAndRead SetThenRead(bool native)
        {
            Settings.Instance["mpcore_params"] = native.ToString();
            var r = new SetAndRead();
            using (var port = new MAVLinkInterface { printbps = false })
            {
                var serial = new MpCoreTestSerial();
                port.BaseStream = serial;
                var vehicle = new Vehicle(serial) { IgnoreSets = 2 };
                var mav = port.MAVlist[1, 1];
                mav.apname = MAVLink.MAV_AUTOPILOT.ARDUPILOTMEGA;
                foreach (var name in new[] { "PARAM_7", "PARAM_9", "PARAM_12" })
                {
                    mav.param[name] = new MAVLink.MAVLinkParam(name, 0, MAVLink.MAV_PARAM_TYPE.REAL32);
                    mav.param_types[name] = MAVLink.MAV_PARAM_TYPE.REAL32;
                }

                Assert.IsTrue(port.setParamAsync(1, 1, "PARAM_7", 42.5).GetAwaiter().GetResult());
                r.AfterSet = (float)mav.param["PARAM_7"].Value;
                r.Sets = vehicle.Sets;
                r.ByName = port.GetParamAsync(1, 1, "PARAM_9").GetAwaiter().GetResult();
                r.ByIndex = port.GetParamAsync(1, 1, "", 12).GetAwaiter().GetResult();
                r.Reads = vehicle.Singles;

                vehicle.Silent = true;
                try { port.setParamAsync(1, 1, "PARAM_9", 1).GetAwaiter().GetResult(); }
                catch (TimeoutException) { r.SetTimedOut = true; }
                try { port.GetParamAsync(1, 1, "PARAM_12").GetAwaiter().GetResult(); }
                catch (TimeoutException) { r.ReadTimedOut = true; }
            }

            return r;
        }

        [TestMethod]
        public void NativeParamSetAndReadBehaveTheSame()
        {
            RequireNative();
            string oldSetting = Settings.Instance["mpcore_params"];
            try
            {
                var csharp = SetThenRead(false);
                var native = SetThenRead(true);
                foreach (var r in new[] { csharp, native })
                {
                    Assert.AreEqual(42.5f, r.AfterSet, "the set value came back");
                    Assert.AreEqual(3, r.Sets, "two PARAM_SETs ignored, the third answered");
                    Assert.AreEqual(9 * 0.25f - 3, r.ByName);
                    Assert.AreEqual(12 * 0.25f - 3, r.ByIndex);
                    Assert.AreEqual(2, r.Reads);
                    Assert.IsTrue(r.SetTimedOut && r.ReadTimedOut, "no answer: a TimeoutException");
                }
            }
            finally
            {
                Settings.Instance["mpcore_params"] = oldSetting ?? "False";
            }
        }

        [TestMethod]
        public void NativeParamDownloadGetsTheSameList()
        {
            RequireNative();

            string oldSetting = Settings.Instance["mpcore_params"];
            try
            {
                var csharp = Download(false);
                var native = Download(true);
                Assert.AreEqual(Count, csharp.list.Count, "the C# loop got every parameter");
                Assert.AreEqual(Count, native.list.Count, "the core got every parameter");
                foreach (var p in csharp.list)
                {
                    var n = native.list.SingleOrDefault(x => x.Name == p.Name);
                    Assert.IsNotNull(n, p.Name);
                    Assert.AreEqual(p.Value, n.Value, p.Name);
                    Assert.AreEqual(p.Type, n.Type, p.Name);
                    Assert.AreEqual(p.TypeAP, n.TypeAP, p.Name);
                }

                // 80 % with the first list and the last index among them: no second list, the
                // missing 30 one by one, by both
                Assert.AreEqual(1, csharp.vehicle.Lists);
                Assert.AreEqual(1, native.vehicle.Lists);
                Assert.AreEqual(Count / 5, native.vehicle.Singles);
                Assert.AreEqual(csharp.vehicle.Singles, native.vehicle.Singles);
            }
            finally
            {
                Settings.Instance["mpcore_params"] = oldSetting ?? "False";
            }
        }
    }
}
