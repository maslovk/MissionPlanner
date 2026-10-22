using System;
using System.Collections.Generic;
using System.IO;
using MissionPlanner.ArduPilot.Mavlink;

// Drives MpCoreShadow the way MAVLinkInterface does, with Mission Planner's C# parser
// (MavlinkParse) reading a stream: every byte the parser reads goes to the shadow, every frame it
// accepts is reported. Usage: MpCoreShadowParity.exe flight.tlog. Exit code 0 on success.
static class MpCoreShadowParity
{
    static int failures;

    static void Check(bool ok, string what)
    {
        if (!ok)
        {
            failures++;
            Console.WriteLine("    CHECK failed: " + what);
        }
    }

    // A stream that tells the shadow about every byte read from it, like the taps on BaseStream.Read.
    sealed class TapStream : Stream
    {
        readonly Stream _inner;
        readonly MpCoreShadow _shadow;
        readonly Random _chunks;

        public TapStream(Stream inner, MpCoreShadow shadow, int seed)
        {
            _inner = inner;
            _shadow = shadow;
            _chunks = new Random(seed);
        }

        public override int Read(byte[] buffer, int offset, int count)
        {
            // serial ports return less than asked; vary it
            if (count > 1)
                count = 1 + _chunks.Next(count);
            int n = _inner.Read(buffer, offset, count);
            _shadow.BytesRead(buffer, offset, n);
            return n;
        }

        public override bool CanRead => true;
        public override bool CanSeek => true;  // MavlinkParse throws EndOfStream on seekable streams
        public override bool CanWrite => false;
        public override long Length => _inner.Length;
        public override long Position { get => _inner.Position; set => throw new NotSupportedException(); }
        public override void Flush() { }
        public override long Seek(long offset, SeekOrigin origin) => throw new NotSupportedException();
        public override void SetLength(long value) => throw new NotSupportedException();
        public override void Write(byte[] buffer, int offset, int count) => throw new NotSupportedException();
    }

    // The frames of a tlog (8-byte timestamp + frame records), without the timestamps.
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
                len = data[i + 9] + MAVLink.GetHeaderLength(data[i + 10]) + 2 +
                      ((data[i + 10] & 1) != 0 ? 13 : 0);
            else
                break;
            if (i + 8 + len > data.Length)
                break;  // a truncated last record
            var f = new byte[len];
            Array.Copy(data, i + 8, f, 0, len);
            frames.Add(f);
            i += 8 + len;
        }

        return frames;
    }

    // Parses the stream with the C# parser while the shadow follows; returns the parser's count.
    static long Run(byte[] stream, MpCoreShadow shadow, int seed)
    {
        var parser = new MAVLink.MavlinkParse();
        var tap = new TapStream(new MemoryStream(stream), shadow, seed);
        long accepted = 0;
        while (true)
        {
            MAVLink.MAVLinkMessage msg;
            try
            {
                msg = parser.ReadPacket(tap);
            }
            catch (EndOfStreamException)
            {
                break;
            }
            catch (InvalidDataException)
            {
                continue;  // a frame MAVLinkMessage rejects as malformed
            }

            if (msg == null || msg.Length == 0)
                continue;
            shadow.FrameAccepted(msg.buffer);
            accepted++;
        }

        shadow.Finish();
        return accepted;
    }

    static void Clean(List<byte[]> frames)
    {
        var stream = new MemoryStream();
        foreach (var f in frames)
            stream.Write(f, 0, f.Length);
        var log = new List<string>();
        using (var shadow = MpCoreShadow.TryCreate(255, 190, null, log.Add))
        {
            Check(shadow != null, "shadow created");
            long accepted = Run(stream.ToArray(), shadow, 1);
            var r = shadow.Counters;
            Console.WriteLine("clean: parser " + accepted + ", " + r);
            Check(accepted == frames.Count, "parser read every frame");
            Check(r.Matched == accepted, "every frame matched");
            Check(r.MissedByCore == 0 && r.ExtraInCore == 0, "no differences");
        }
    }

    static void Noisy(List<byte[]> frames)
    {
        var rnd = new Random(7);
        var stream = new MemoryStream();
        int corrupted = 0;
        foreach (var original in frames)
        {
            var f = original;
            if (rnd.Next(200) == 0)
            {
                // line noise, with start bytes
                int n = rnd.Next(1, 30);
                for (int k = 0; k < n; k++)
                    stream.WriteByte(rnd.Next(4) == 0 ? (byte)0xFE : (byte)rnd.Next(256));
            }

            if (rnd.Next(500) == 0)
            {
                f = (byte[])original.Clone();
                f[rnd.Next(f.Length)] ^= (byte)(1 << rnd.Next(8));
                corrupted++;
            }

            stream.Write(f, 0, f.Length);
        }

        var log = new List<string>();
        using (var shadow = MpCoreShadow.TryCreate(255, 190, null, log.Add))
        {
            long accepted = Run(stream.ToArray(), shadow, 2);
            var r = shadow.Counters;
            Console.WriteLine("noisy (" + corrupted + " corrupted frames): parser " + accepted + ", " + r);
            // the core rescans after a failed frame, the C# parser drops it: the core may find more,
            // but must never lose a frame the parser accepted
            Check(r.MissedByCore == 0, "core missed nothing the parser accepted");
            Check(r.Matched == accepted, "every accepted frame matched");
            Check(r.Matched + r.ExtraInCore >= frames.Count - corrupted, "core recovered the undamaged frames");
            foreach (var line in log)
                if (line.Contains("missed by core"))
                    Console.WriteLine("  " + line);
        }
    }

    static void SyncLoss()
    {
        // heartbeats from sysid 1 with seq 0..9 and 15: one gap of 5 lost frames
        var log = new List<string>();
        using (var encoder = new MissionPlanner.Mavlink.Native.MpLink(1, 1))
        using (var shadow = MpCoreShadow.TryCreate(255, 190, null, log.Add))
        {
            var frames = new List<byte[]>();
            encoder.Write += b => frames.Add(b);
            var hb = new byte[] { 0, 0, 0, 0, 2, 3, 81, 4, 3 };
            for (int seq = 0; seq < 16; seq++)
                encoder.Send(0, hb, 255, 190);
            frames.RemoveRange(10, 5);

            float parserSyncLost = 0;
            foreach (var f in frames)
            {
                shadow.BytesRead(f, 0, f.Length);
                shadow.FrameAccepted(f);
                if (f[2] == 15)
                    parserSyncLost++;  // what MAVLinkInterface counts for the gap
                shadow.CheckSyncLoss(1, 1, parserSyncLost);
            }

            var r = shadow.Counters;
            Check(r.SyncLossChecks == frames.Count - 1 && r.SyncLossMismatches == 0, "sync loss agrees: " + r);

            // a parser that miscounts is reported
            shadow.CheckSyncLoss(1, 1, parserSyncLost + 1);
            Check(shadow.Counters.SyncLossMismatches == 1, "a sync loss difference is reported");
            Check(log.Exists(l => l.Contains("sync loss differs for 1-1")), "and logged");
        }
    }

    static int Main(string[] args)
    {
        if (args.Length != 1)
        {
            Console.WriteLine("usage: MpCoreShadowParity flight.tlog");
            return 2;
        }

        var frames = ReadTlog(args[0]);
        Console.WriteLine(frames.Count + " frames in " + Path.GetFileName(args[0]));
        Check(frames.Count > 1000, "the log has frames");
        Clean(frames);
        Noisy(frames);
        SyncLoss();
        Console.WriteLine(failures == 0 ? "ok" : "FAIL (" + failures + " failed checks)");
        return failures == 0 ? 0 : 1;
    }
}
