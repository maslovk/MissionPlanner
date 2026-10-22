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

        // Answers PARAM_REQUEST_LIST (dropping every fifth parameter the first time) and
        // PARAM_REQUEST_READ with PARAM_VALUE frames.
        sealed class Vehicle
        {
            readonly MpCoreTestSerial _serial;
            readonly MAVLink.MavlinkParse _encoder = new MAVLink.MavlinkParse();
            int _lists;
            public int Lists, Singles;

            public Vehicle(MpCoreTestSerial serial)
            {
                _serial = serial;
                serial.OnWrite = Receive;
            }

            void Send(int index)
            {
                var name = new byte[16];
                Encoding.ASCII.GetBytes("PARAM_" + index).CopyTo(name, 0);
                var value = new MAVLink.mavlink_param_value_t
                {
                    param_id = name,
                    param_value = index * 0.25f - 3,
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
                    if (req.param_index >= 0 && req.param_index < Count)
                        Send(req.param_index);
                }
            }
        }

        static (List<MAVLinkParam> list, Vehicle vehicle) Download(bool native)
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

        [TestMethod]
        public void NativeParamDownloadGetsTheSameList()
        {
            if (!MpCoreNativeLoader.Available("test", null))
            {
                const string why = "the mpcore native library is not available";
                if (Environment.GetEnvironmentVariable("MPCORE_REQUIRED") == "1")
                    Assert.Fail(why);
                Assert.Inconclusive(why);
            }

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
