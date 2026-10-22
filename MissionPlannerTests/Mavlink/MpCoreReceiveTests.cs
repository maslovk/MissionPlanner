using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using MissionPlanner;
using MissionPlanner.Comms;
using MissionPlanner.Utilities;

namespace MissionPlannerTests.Mavlink
{
    /// <summary>
    /// MAVLinkInterface.readPacketAsync with the mpcore_receive setting (the C++ core framing the
    /// bytes) against the C# byte loop, on the same streams. Needs the mpcore native library next
    /// to MissionPlanner.ArduPilot.dll (x64/ or x86/ on Windows); without it the tests are
    /// inconclusive, unless MPCORE_REQUIRED=1 makes that a failure (CI).
    /// </summary>
    [TestClass]
    public class MpCoreReceiveTests
    {
        static readonly MAVLink.mavlink_heartbeat_t Heartbeat = new MAVLink.mavlink_heartbeat_t
            { type = 2, autopilot = 3, base_mode = 81, system_status = 4, mavlink_version = 3 };

        sealed class Result
        {
            public List<byte[]> Frames = new List<byte[]>();
            public MAVLinkInterface Port;
        }

        static Result Read(byte[] stream, bool native)
        {
            Settings.Instance["mpcore_receive"] = native.ToString();
            var result = new Result { Port = new MAVLinkInterface { printbps = false } };
            var serial = new MpCoreTestSerial();
            result.Port.BaseStream = serial;
            serial.Feed(stream);
            while (serial.BytesToRead > 0 || result.Port.HasBufferedFrames)
            {
                var msg = result.Port.readPacket();
                if (msg != null && msg.Length > 0)
                    result.Frames.Add(msg.buffer);
            }

            if (native)
            {
                var receiver = typeof(MAVLinkInterface).GetField("_receiver", BindingFlags.NonPublic | BindingFlags.Instance)
                    .GetValue(result.Port);
                if (receiver == null)
                {
                    const string why = "the mpcore native library is not available";
                    if (Environment.GetEnvironmentVariable("MPCORE_REQUIRED") == "1")
                        Assert.Fail(why);
                    Assert.Inconclusive(why);
                }
            }

            return result;
        }

        static string PlainTextLine(MAVLinkInterface port)
        {
            return (string)typeof(MAVLinkInterface).GetField("plaintxtline", BindingFlags.NonPublic | BindingFlags.Instance)
                .GetValue(port);
        }

        // Several vehicles, MAVLink 1 and 2, a 32-bit system ID, a command addressed to another
        // 32-bit system, a sequence gap and a line of text.
        static byte[] MixedStream()
        {
            var parse = new MAVLink.MavlinkParse();
            var s = new MemoryStream();
            Action<byte[]> Add = b => s.Write(b, 0, b.Length);
            var text = Encoding.ASCII.GetBytes("ArduCopter V4.7.0\r\n");
            Add(text);
            for (int seq = 0; seq < 20; seq++)
            {
                if (seq == 12)
                    continue; // lost
                Add(parse.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, Heartbeat, false, 1, 1, seq));
                Add(parse.GenerateMAVLinkPacket10(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, Heartbeat, 2, 1, seq));
                Add(parse.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, Heartbeat, false, 0xffffff01, 1, seq));
            }

            var command = new MAVLink.mavlink_command_long_t { target_system = 255, target_component = 1, command = 300 };
            Add(parse.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.COMMAND_LONG, command, false, 1, 1, 20, 0xfffffffe, 1));
            Add(parse.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, Heartbeat, false, 1, 1, 21));
            return s.ToArray();
        }

        [TestMethod]
        public void NativeReceiveReturnsWhatTheByteLoopReturns()
        {
            uint oldGcs = MAVLinkInterface.gcssysid;
            string oldSetting = Settings.Instance["mpcore_receive"];
            try
            {
                MAVLinkInterface.gcssysid = 255;
                var stream = MixedStream();
                var csharp = Read(stream, false);
                var native = Read(stream, true);
                using (csharp.Port)
                using (native.Port)
                {
                    Assert.AreEqual(19 * 3 + 1, csharp.Frames.Count, "the C# loop: all but the command to another system");
                    Assert.AreEqual(csharp.Frames.Count, native.Frames.Count);
                    for (int i = 0; i < csharp.Frames.Count; i++)
                        CollectionAssert.AreEqual(csharp.Frames[i], native.Frames[i], "frame " + i);

                    foreach (uint sysid in new[] { 1u, 2u, 0xffffff01u })
                    {
                        var c = csharp.Port.MAVlist[sysid, 1];
                        var n = native.Port.MAVlist[sysid, 1];
                        Assert.AreEqual(c.packetslost, n.packetslost, "packets lost " + sysid);
                        Assert.AreEqual(c.packetsnotlost, n.packetsnotlost, "packets received " + sysid);
                        Assert.AreEqual(c.synclost, n.synclost, "sync losses " + sysid);
                        Assert.AreEqual(c.mavlinkv2, n.mavlinkv2, "MAVLink 2 " + sysid);
                    }

                    // at least the lost seq 12; the command to another system is dropped before it is
                    // counted, so the heartbeat after it shows as a gap too
                    Assert.IsTrue(native.Port.MAVlist[1, 1].synclost >= 1, "the gap was counted");
                    Assert.AreEqual("ArduCopter V4.7.0", PlainTextLine(csharp.Port));
                    Assert.AreEqual(PlainTextLine(csharp.Port), PlainTextLine(native.Port));
                }
            }
            finally
            {
                MAVLinkInterface.gcssysid = oldGcs;
                Settings.Instance["mpcore_receive"] = oldSetting ?? "False";
            }
        }

        [TestMethod]
        public void NativeReceiveRecoversFramesAfterACorruptedLength()
        {
            string oldSetting = Settings.Instance["mpcore_receive"];
            try
            {
                var parse = new MAVLink.MavlinkParse();
                var s = new MemoryStream();
                var damaged = parse.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, Heartbeat, false, 1, 1, 0);
                damaged[1] = 200; // claims 200 payload bytes: the next frames would be read as its payload
                s.Write(damaged, 0, damaged.Length);
                for (int seq = 1; seq <= 10; seq++)
                {
                    var f = parse.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.HEARTBEAT, Heartbeat, false, 1, 1, seq);
                    s.Write(f, 0, f.Length);
                }

                var native = Read(s.ToArray(), true);
                using (native.Port)
                    Assert.AreEqual(10, native.Frames.Count, "every frame after the damaged one");
            }
            finally
            {
                Settings.Instance["mpcore_receive"] = oldSetting ?? "False";
            }
        }
    }
}
