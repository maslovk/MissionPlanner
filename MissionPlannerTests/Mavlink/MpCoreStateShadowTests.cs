using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using MissionPlanner;
using MissionPlanner.ArduPilot.Mavlink;
using MissionPlanner.Utilities;

namespace MissionPlannerTests.Mavlink
{
    /// <summary>
    /// The mpcore_state_shadow setting on a recorded flight (the pymavlink submodule's flight.tlog,
    /// three systems): every value the core decodes must equal CurrentState's. Needs the mpcore
    /// native library like MpCoreReceiveTests.
    /// </summary>
    [TestClass]
    public class MpCoreStateShadowTests
    {
        const string Tlog = "ExtLibs/mpcore/third_party/pymavlink/generator/swift/Tests/MAVLinkTests/Testdata/flight.tlog";

        static string FindUp(string relative)
        {
            for (var dir = new DirectoryInfo(AppDomain.CurrentDomain.BaseDirectory); dir != null; dir = dir.Parent)
            {
                var path = Path.Combine(dir.FullName, relative);
                if (File.Exists(path))
                    return path;
            }

            return null;
        }

        // the frames of a tlog (8-byte timestamp + frame records)
        static List<byte[]> ReadTlog(string path)
        {
            var data = File.ReadAllBytes(path);
            var frames = new List<byte[]>();
            int i = 0;
            while (i + 10 <= data.Length)
            {
                int stx = data[i + 8];
                int len;
                if (stx == 0xFE)
                    len = data[i + 9] + 8;
                else if (stx == 0xFD)
                    len = data[i + 9] + MAVLink.GetHeaderLength(data[i + 10]) + 2 + ((data[i + 10] & 1) != 0 ? 13 : 0);
                else
                    break;
                if (i + 8 + len > data.Length)
                    break;
                var f = new byte[len];
                Array.Copy(data, i + 8, f, 0, len);
                frames.Add(f);
                i += 8 + len;
            }

            return frames;
        }

        static void Skip(string why)
        {
            if (Environment.GetEnvironmentVariable("MPCORE_REQUIRED") == "1")
                Assert.Fail(why);
            Assert.Inconclusive(why);
        }

        [TestMethod]
        public void StateShadowAgreesOnARecordedFlight()
        {
            if (!MpCoreNativeLoader.Available("test", null))
                Skip("the mpcore native library is not available");
            var tlog = FindUp(Tlog);
            if (tlog == null)
                Skip("no " + Tlog + " (git submodule update --init ExtLibs/mpcore/third_party/pymavlink)");

            var frames = ReadTlog(tlog);
            Assert.IsTrue(frames.Count > 100000, "frames read: " + frames.Count);
            string oldSetting = Settings.Instance["mpcore_state_shadow"];
            Settings.Instance["mpcore_state_shadow"] = "True";
            try
            {
                using (var port = new MAVLinkInterface { printbps = false })
                {
                    var serial = new MpCoreTestSerial();
                    port.BaseStream = serial;
                    // the vehicles, so that their CurrentState takes their messages
                    var vehicles = new Dictionary<(uint, byte), MAVState>();
                    foreach (var f in frames)
                    {
                        var m = new MAVLink.MAVLinkMessage(f);
                        if (m.msgid == (uint)MAVLink.MAVLINK_MSG_ID.HEARTBEAT && !vehicles.ContainsKey((m.sysid, m.compid)))
                            vehicles[(m.sysid, m.compid)] = port.MAVlist[m.sysid, m.compid];
                    }

                    serial.Feed(frames.SelectMany(f => f).ToArray());
                    for (int idle = 0; idle < 5;)
                    {
                        var msg = port.readPacketAsync().GetAwaiter().GetResult();
                        idle = serial.BytesToRead == 0 && (msg == null || msg.Length == 0) ? idle + 1 : 0;
                    }

                    var shadowField = typeof(CurrentState).GetField("_stateShadow", BindingFlags.NonPublic | BindingFlags.Instance);
                    Assert.IsNotNull(shadowField, "CurrentState._stateShadow");
                    long compared = 0;
                    foreach (var vehicle in vehicles)
                    {
                        var shadow = (MpCoreStateShadow)shadowField.GetValue(vehicle.Value.cs);
                        Assert.IsNotNull(shadow, "a shadow for " + vehicle.Key);
                        var totals = shadow.Totals;
                        Console.WriteLine(vehicle.Key + ": " + totals);
                        Assert.AreEqual(0, totals.Differences,
                            vehicle.Key + ": " + totals + "\n" + string.Join("\n", totals.Details));
                        compared += totals.Compared;
                        shadow.Dispose();
                    }

                    Assert.IsTrue(compared > 100000, "values compared: " + compared);
                }
            }
            finally
            {
                Settings.Instance["mpcore_state_shadow"] = oldSetting ?? "False";
            }
        }
    }
}
