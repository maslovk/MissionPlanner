using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using MissionPlanner;
using MissionPlanner.ArduPilot.Mavlink;
using MissionPlanner.Utilities;

namespace MissionPlannerTests.Mavlink
{
    /// <summary>
    /// MAVFtp with the mpcore_ftp setting (the C++ core running the operations) against the C#
    /// loops, on a simulated ArduPilot FTP server that loses a burst packet and a write ACK: the
    /// same file, upload, listing, CRC and errors. A reader thread delivers the replies, as
    /// MainV2.SerialReader does. Needs the mpcore native library like MpCoreReceiveTests.
    /// </summary>
    [TestClass]
    public class MpCoreFtpTests
    {
        sealed class FtpVehicle
        {
            readonly MpCoreTestSerial _serial;
            readonly MAVLink.MavlinkParse _encoder = new MAVLink.MavlinkParse();
            readonly object _sync = new object();
            public readonly Dictionary<string, byte[]> Files = new Dictionary<string, byte[]>();
            public readonly HashSet<string> Dirs = new HashSet<string> { "/" };
            bool _open, _writing;
            string _path;
            byte[] _last;
            ushort _lastSeq;
            int _replies;
            public int LoseBurstOffset = -1; // the first data packet at this offset is lost
            public int LoseWriteAck = -1;    // the first ACK of the write at this offset is lost

            public FtpVehicle(MpCoreTestSerial serial)
            {
                _serial = serial;
                serial.OnWrite = Receive;
            }

            void Send(byte[] ftp)
            {
                _last = ftp;
                _lastSeq = (ushort)(ftp[0] | (ftp[1] << 8));
                _replies++;
                uint offset = BitConverter.ToUInt32(ftp, 8);
                if (ftp[3] == 128 && ftp[5] == 15 && offset == LoseBurstOffset)
                {
                    LoseBurstOffset = -1;
                    return;
                }
                if (ftp[3] == 128 && ftp[5] == 7 && offset == LoseWriteAck)
                {
                    LoseWriteAck = -1;
                    return;
                }
                var msg = new MAVLink.mavlink_file_transfer_protocol_t
                {
                    target_network = 0, target_system = 255, target_component = 190, payload = ftp,
                };
                _serial.Feed(_encoder.GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.FILE_TRANSFER_PROTOCOL, msg, false, 1, 1));
            }

            static byte[] Reply(byte[] q, ushort seq, byte opcode, byte[] data = null, uint offset = 0, bool burstComplete = false)
            {
                var f = new byte[251];
                f[0] = (byte)seq;
                f[1] = (byte)(seq >> 8);
                f[2] = q[2];
                f[3] = opcode;
                f[4] = (byte)(data?.Length ?? 0);
                f[5] = q[3];
                f[6] = (byte)(burstComplete ? 1 : 0);
                BitConverter.GetBytes(offset).CopyTo(f, 8);
                data?.CopyTo(f, 12);
                return f;
            }

            void Ack(byte[] q, byte[] data = null, uint offset = 0) => Send(Reply(q, (ushort)(Seq(q) + 1), 128, data, offset));
            void Nak(byte[] q, params byte[] code) => Send(Reply(q, (ushort)(Seq(q) + 1), 129, code));
            static ushort Seq(byte[] q) => (ushort)(q[0] | (q[1] << 8));
            static string Text(byte[] q) => System.Text.Encoding.UTF8.GetString(q, 12, q[4]);
            static string Parent(string path) { int i = path.LastIndexOf('/'); return i <= 0 ? "/" : path.Substring(0, i); }
            static string Leaf(string path) => path.Substring(path.LastIndexOf('/') + 1);

            void Receive(byte[] frame)
            {
                var msg = new MAVLink.MAVLinkMessage(frame);
                if (msg.msgid != (uint)MAVLink.MAVLINK_MSG_ID.FILE_TRANSFER_PROTOCOL)
                    return;
                var q = msg.ToStructure<MAVLink.mavlink_file_transfer_protocol_t>().payload;
                lock (_sync)
                {
                    // a request sent again gets the last reply again, as ArduPilot's GCS_FTP does
                    if (_last != null && (ushort)(Seq(q) + 1) == _lastSeq)
                    {
                        Send(_last);
                        return;
                    }
                    Handle(q);
                }
            }

            void Handle(byte[] q)
            {
                uint offset = BitConverter.ToUInt32(q, 8);
                switch (q[3])
                {
                    case 1: // TerminateSession
                    case 2: // ResetSessions
                        _open = false;
                        Ack(q);
                        return;
                    case 4: // OpenFileRO
                        if (!Files.ContainsKey(Text(q)))
                        {
                            Nak(q, 2, 2); // ENOENT
                            return;
                        }
                        _open = true;
                        _writing = false;
                        _path = Text(q);
                        Ack(q, BitConverter.GetBytes(Files[_path].Length));
                        return;
                    case 5:  // ReadFile
                    case 15: // BurstReadFile
                    {
                        if (!_open || _writing)
                        {
                            Nak(q, 4);
                            return;
                        }
                        var file = Files[_path];
                        int packets = q[3] == 15 ? 10 : 1;
                        var seq = (ushort)(Seq(q) + 1);
                        for (int i = 0; i < packets; i++, seq++)
                        {
                            long off = offset + (long)i * q[4];
                            if (off >= file.Length)
                            {
                                Send(Reply(q, seq, 129, new byte[] { 6 }));
                                return;
                            }
                            var chunk = file.Skip((int)off).Take(q[4]).ToArray();
                            Send(Reply(q, seq, 128, chunk, (uint)off, i == packets - 1));
                        }
                        return;
                    }
                    case 6: // CreateFile
                        Files[Text(q)] = new byte[0];
                        _open = true;
                        _writing = true;
                        _path = Text(q);
                        Ack(q);
                        return;
                    case 7: // WriteFile
                    {
                        var file = Files[_path];
                        int end = (int)offset + q[4];
                        if (file.Length < end)
                            Array.Resize(ref file, end);
                        Array.Copy(q, 12, file, offset, q[4]);
                        Files[_path] = file;
                        Ack(q, null, offset);
                        return;
                    }
                    case 16: // ListDirectoryWithTime
                    {
                        var dir = Text(q);
                        var entries = Dirs.Where(d => d != "/" && Parent(d) == dir).Select(d => "D" + Leaf(d) + "\t0\t1700000000")
                            .Concat(Files.Where(f => Parent(f.Key) == dir).OrderBy(f => f.Key)
                                .Select(f => "F" + Leaf(f.Key) + "\t" + f.Value.Length + "\t1600000000"))
                            .ToList();
                        if (offset >= entries.Count)
                        {
                            Nak(q, 6);
                            return;
                        }
                        var data = new List<byte>();
                        for (int i = (int)offset; i < entries.Count; i++)
                        {
                            var bytes = System.Text.Encoding.UTF8.GetBytes(entries[i] + "\0");
                            if (data.Count + bytes.Length > 239)
                                break;
                            data.AddRange(bytes);
                        }
                        Ack(q, data.ToArray(), offset);
                        return;
                    }
                    case 8: // RemoveFile
                        if (Files.Remove(Text(q)))
                            Ack(q);
                        else
                            Nak(q, 2, 2);
                        return;
                    case 9: // CreateDirectory
                        if (!Dirs.Add(Text(q)))
                            Nak(q, 8);
                        else
                            Ack(q);
                        return;
                    case 14: // CalcFileCRC32
                        Ack(q, BitConverter.GetBytes(MAVFtp.crc_crc32(0, Files[Text(q)])));
                        return;
                    default:
                        Nak(q, 7);
                        return;
                }
            }
        }

        static byte[] Pattern(int n, int salt)
        {
            var b = new byte[n];
            for (int i = 0; i < n; i++)
                b[i] = (byte)((i * 31 + salt) ^ (i >> 8));
            return b;
        }

        sealed class Outcome
        {
            public byte[] File;
            public string MissingFile, MissingRemove;
            public byte[] Uploaded;
            public string[] Listing;
            public uint Crc;
            public bool MkdirTwice;
        }

        static string Error(Action run)
        {
            try
            {
                run();
                return null;
            }
            catch (Exception ex)
            {
                return ex.GetType().Name + ": " + ex.Message;
            }
        }

        static Outcome Run(bool native)
        {
            Settings.Instance["mpcore_ftp"] = native.ToString();
            using (var port = new MAVLinkInterface { printbps = false })
            {
                var serial = new MpCoreTestSerial();
                port.BaseStream = serial;
                var vehicle = new FtpVehicle(serial);
                vehicle.Dirs.Add("/APM");
                vehicle.Dirs.Add("/APM/LOGS");
                vehicle.Files["/@PARAM/param.pck"] = Pattern(2345, 1);
                vehicle.Files["/APM/a.txt"] = Pattern(10, 2);
                vehicle.Files["/APM/b.txt"] = Pattern(20, 3);
                vehicle.LoseBurstOffset = 3 * 110;
                vehicle.LoseWriteAck = 160;

                var stop = false;
                var reader = Task.Run(async () =>
                {
                    while (!Volatile.Read(ref stop))
                    {
                        try { await port.readPacketAsync().ConfigureAwait(false); }
                        catch { Thread.Sleep(1); }
                    }
                });
                try
                {
                    var ftp = new MAVFtp(port, 1, 1);
                    var outcome = new Outcome();
                    outcome.File = ftp.GetFile("/@PARAM/param.pck", null, true, 110)?.ToArray();
                    outcome.MissingFile = Error(() => ftp.GetFile("/nope", null));
                    ftp.UploadFile("/APM/scripts.lua", new MemoryStream(Pattern(700, 4)), null);
                    vehicle.Files.TryGetValue("/APM/scripts.lua", out outcome.Uploaded);
                    outcome.Listing = ftp.kCmdListDirectory("/APM/", null)
                        .Select(e => e.Name + " " + e.isDirectory + " " + e.Size + " " + e.ModifiedUtc?.ToString("o"))
                        .ToArray();
                    uint crc = 0;
                    Assert.IsTrue(ftp.kCmdCalcFileCRC32("/APM/b.txt", ref crc, null));
                    outcome.Crc = crc;
                    outcome.MissingRemove = Error(() => ftp.kCmdRemoveFile("/APM/nope", null));
                    outcome.MkdirTwice = ftp.kCmdCreateDirectory("/APM/new", null) && ftp.kCmdCreateDirectory("/APM/new", null);
                    return outcome;
                }
                finally
                {
                    Volatile.Write(ref stop, true);
                    reader.Wait(5000);
                }
            }
        }

        [TestMethod]
        public void NativeFtpBehavesTheSame()
        {
            if (!MpCoreNativeLoader.Available("test", null))
            {
                const string why = "the mpcore native library is not available";
                if (Environment.GetEnvironmentVariable("MPCORE_REQUIRED") == "1")
                    Assert.Fail(why);
                Assert.Inconclusive(why);
            }

            string oldSetting = Settings.Instance["mpcore_ftp"];
            try
            {
                var csharp = Run(false);
                var native = Run(true);
                CollectionAssert.AreEqual(Pattern(2345, 1), csharp.File, "C# download");
                CollectionAssert.AreEqual(Pattern(2345, 1), native.File, "native download");
                CollectionAssert.AreEqual(Pattern(700, 4), csharp.Uploaded, "C# upload");
                CollectionAssert.AreEqual(Pattern(700, 4), native.Uploaded, "native upload");
                Assert.AreEqual(4, native.Listing.Length, string.Join(", ", native.Listing));
                CollectionAssert.AreEqual(csharp.Listing, native.Listing, "listing");
                Assert.AreEqual(csharp.Crc, native.Crc, "CRC");
                Assert.AreEqual(MAVFtp.crc_crc32(0, Pattern(20, 3)), native.Crc, "CRC value");
                Assert.IsNotNull(native.MissingFile);
                Assert.AreEqual(csharp.MissingFile, native.MissingFile, "a missing file");
                Assert.IsNotNull(native.MissingRemove);
                Assert.AreEqual(csharp.MissingRemove, native.MissingRemove, "removing a missing file");
                Assert.IsTrue(csharp.MkdirTwice && native.MkdirTwice, "a directory already there");
            }
            finally
            {
                Settings.Instance["mpcore_ftp"] = oldSetting ?? "False";
            }
        }
    }
}
