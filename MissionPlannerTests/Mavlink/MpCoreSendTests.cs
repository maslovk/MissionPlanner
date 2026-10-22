using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using MissionPlanner;
using MissionPlanner.Utilities;

namespace MissionPlannerTests.Mavlink
{
    /// <summary>
    /// MAVLinkInterface.generatePacket with the mpcore_send setting (the C++ core encoding) against
    /// its C# encoder: every message of the dialect, to vehicles on MAVLink 1, MAVLink 2, a 32-bit ID
    /// and signing. Needs the mpcore native library like MpCoreReceiveTests.
    /// </summary>
    [TestClass]
    public class MpCoreSendTests
    {
        static readonly byte[] Key = Enumerable.Range(0, 32).Select(i => (byte)(i * 7 + 1)).ToArray();

        // MAVLink 1 (has only sent MAVLink 1), MAVLink 2, a 32-bit system, signing
        static readonly uint[] Vehicles = { 1, 2, 0x80000003, 4 };

        static List<byte[]> SendAll(bool native)
        {
            Settings.Instance["mpcore_send"] = native.ToString();
            var sent = new List<byte[]>();
            using (var port = new MAVLinkInterface { printbps = false })
            {
                port.BaseStream = new MpCoreTestSerial();
                port.OnPacketSent += (sender, msg) => sent.Add(msg.buffer);
                port.MAVlist[2, 1].mavlinkv2 = true;
                var signer = port.MAVlist[4, 1];
                signer.mavlinkv2 = true;
                signer.signing = true;
                typeof(MAVState).GetField("signingKey", BindingFlags.NonPublic | BindingFlags.Instance).SetValue(signer, Key);
                typeof(MAVState).GetProperty("sendlinkid").SetValue(signer, (byte)3);

                var rnd = new Random(11);
                foreach (var info in MAVLink.MAVLINK_MESSAGE_INFOS)
                {
                    int size = Marshal.SizeOf(info.type);
                    var bytes = new byte[size];
                    rnd.NextBytes(bytes);
                    var obj = MavlinkUtil.ByteArrayToStructureGC(bytes, info.type, 0, (byte)size);
                    foreach (uint sysid in Vehicles)
                        port.generatePacket((int)info.msgid, obj, sysid, 1);
                }

                if (native)
                {
                    var on = (bool?)typeof(MAVLinkInterface).GetField("_nativeSend", BindingFlags.NonPublic | BindingFlags.Instance)
                        .GetValue(port);
                    if (on != true)
                    {
                        const string why = "the mpcore native library is not available";
                        if (Environment.GetEnvironmentVariable("MPCORE_REQUIRED") == "1")
                            Assert.Fail(why);
                        Assert.Inconclusive(why);
                    }
                }
            }

            return sent;
        }

        static bool Signed(byte[] f) => f[0] == MAVLink.MAVLINK_STX && (f[2] & MAVLink.MAVLINK_IFLAG_SIGNED) != 0;

        [TestMethod]
        public void NativeSendWritesWhatTheCSharpEncoderWrites()
        {
            string oldSetting = Settings.Instance["mpcore_send"];
            try
            {
                var csharp = SendAll(false);
                var native = SendAll(true);
                Assert.AreEqual(MAVLink.MAVLINK_MESSAGE_INFOS.Length * Vehicles.Length, csharp.Count, "a frame per message and vehicle");
                Assert.AreEqual(csharp.Count, native.Count);
                int signed = 0;
                for (int i = 0; i < csharp.Count; i++)
                {
                    var c = csharp[i];
                    var n = native[i];
                    Assert.AreEqual(c.Length, n.Length, "frame " + i + " length");
                    // the signature timestamp is the time of sending, and the signature covers it
                    int compared = Signed(c) ? c.Length - 12 : c.Length;
                    CollectionAssert.AreEqual(c.Take(compared).ToArray(), n.Take(compared).ToArray(), "frame " + i);
                    if (Signed(n))
                    {
                        signed++;
                        using (var sha = SHA256.Create())
                        {
                            var hash = sha.ComputeHash(Key.Concat(n.Take(n.Length - 6)).ToArray()).Take(6).ToArray();
                            CollectionAssert.AreEqual(hash, n.Skip(n.Length - 6).ToArray(), "signature of frame " + i);
                        }
                    }
                }

                Assert.AreEqual(MAVLink.MAVLINK_MESSAGE_INFOS.Length, signed, "a signed frame per message");
                Assert.IsTrue(native.Any(f => f[0] == MAVLink.MAVLINK_STX_MAVLINK1), "MAVLink 1 frames too");
                Assert.IsTrue(native.Any(f => f[0] == MAVLink.MAVLINK_STX && (f[2] & MAVLink.MAVLINK_IFLAG_TARGET32) != 0),
                    "32-bit destinations too");
            }
            finally
            {
                Settings.Instance["mpcore_send"] = oldSetting ?? "False";
            }
        }
    }
}
