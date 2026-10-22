using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using MissionPlanner.ArduPilot.Mavlink;
using MissionPlanner.Comms;

// Tests MpCoreReceiver, the core reading a port for MAVLinkInterface.readPacketAsync, against a
// fake port. Usage: MpCoreReceiverTest.exe flight.tlog. Exit code 0 on success.
static class MpCoreReceiverTest
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

    // A port that makes its bytes available a few at a time, as a serial port or socket does.
    sealed class FakePort : ICommsSerial
    {
        readonly byte[] _data;
        readonly Random _arrivals;
        int _position;
        int _arrived;

        public FakePort(byte[] data, int seed = 1)
        {
            _data = data;
            _arrivals = new Random(seed);
        }

        public int Position => _position;
        public int Reads;

        public int BytesToRead
        {
            get
            {
                if (_arrived == _position && _position < _data.Length)
                    _arrived = Math.Min(_data.Length, _position + 1 + _arrivals.Next(300));
                return _arrived - _position;
            }
        }

        public int Read(byte[] buffer, int offset, int count)
        {
            Reads++;
            int n = Math.Min(count, _arrived - _position);
            Array.Copy(_data, _position, buffer, offset, n);
            _position += n;
            return n;
        }

        public bool IsOpen { get; set; } = true;
        public Stream BaseStream => null;
        public int BaudRate { get; set; }
        public int BytesToWrite => 0;
        public int DataBits { get; set; }
        public bool DtrEnable { get; set; }
        public string PortName { get; set; } = "fake";
        public int ReadBufferSize { get; set; }
        public int ReadTimeout { get; set; } = 1200;
        public bool RtsEnable { get; set; }
        public int WriteBufferSize { get; set; }
        public int WriteTimeout { get; set; }
        public void Close() => IsOpen = false;
        public void DiscardInBuffer() { }
        public void Open() => IsOpen = true;
        public int ReadByte() => throw new NotSupportedException();
        public int ReadChar() => throw new NotSupportedException();
        public string ReadExisting() => throw new NotSupportedException();
        public string ReadLine() => throw new NotSupportedException();
        public void Write(string text) { }
        public void Write(byte[] buffer, int offset, int count) { }
        public void WriteLine(string text) { }
        public void toggleDTR() { }
        public void Dispose() { }
    }

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
                len = data[i + 9] + 12 + ((data[i + 10] & 2) != 0 ? 3 : 0) + ((data[i + 10] & 4) != 0 ? 4 : 0) +
                      ((data[i + 10] & 1) != 0 ? 13 : 0);
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

    static bool Same(byte[] a, byte[] b)
    {
        if (a.Length != b.Length)
            return false;
        for (int i = 0; i < a.Length; i++)
            if (a[i] != b[i])
                return false;
        return true;
    }

    static byte[] Concat(IEnumerable<byte[]> parts)
    {
        var s = new MemoryStream();
        foreach (var p in parts)
            s.Write(p, 0, p.Length);
        return s.ToArray();
    }

    static MpCoreReceiver Create()
    {
        var r = MpCoreReceiver.TryCreate(255, 190, Console.WriteLine);
        if (r == null)
            throw new InvalidOperationException("native library not available");
        return r;
    }

    // Every frame of a real flight comes out unchanged and in order, one per call, however the
    // bytes arrive; every byte read is reported for the raw log.
    static void FlightLog(List<byte[]> frames)
    {
        var data = Concat(frames);
        var port = new FakePort(data);
        long reported = 0;
        using (var r = Create())
        {
            r.BytesRead += (b, o, n) => reported += n;
            int i = 0;
            bool inOrder = true;
            byte[] f;
            while ((f = r.NextFrameAsync(port, 100).GetAwaiter().GetResult()) != null)
            {
                if (i >= frames.Count || !Same(f, frames[i]))
                    inOrder = false;
                i++;
            }

            Console.WriteLine("flight log: " + i + " of " + frames.Count + " frames, " + port.Reads + " reads");
            Check(i == frames.Count && inOrder, "every frame, unchanged and in order");
            Check(reported == data.Length && port.Position == data.Length, "every byte reported once");
        }
    }

    // Reads stop shortly after a frame: what follows stays in the port, and frames already parsed
    // are announced so the reader loop does not wait for more bytes.
    static void ReadsLittleAndAnnouncesParsedFrames(List<byte[]> frames)
    {
        var data = Concat(frames.GetRange(0, 50));
        var port = new FakePort(data);
        using (var r = Create())
        {
            var first = r.NextFrameAsync(port, 100).GetAwaiter().GetResult();
            Check(first != null && Same(first, frames[0]), "first frame");
            Check(port.Position <= frames[0].Length + 64, "at most one chunk past the frame was read: " + port.Position);

            // small frames: a chunk holds several, the rest wait in the receiver
            int got = 1;
            bool announced = true;
            while (r.HasFrames || port.Position < data.Length)
            {
                if (port.Position == data.Length && !r.HasFrames)
                    announced = false;
                if (r.NextFrameAsync(port, 100).GetAwaiter().GetResult() != null)
                    got++;
            }

            Check(got == 50 && announced, "all 50 frames, the buffered ones announced: " + got);
        }
    }

    static void TextLinesAndNoise(List<byte[]> frames)
    {
        var text = Encoding.ASCII.GetBytes("PX4 bootloader v5\r\nab\n");
        var noise = new byte[] { 0xFE, 0xFE, 0x12, 0xFD, 0x99 };
        var damaged = (byte[])frames[1].Clone();
        damaged[damaged.Length - 1] ^= 0xFF;
        var data = Concat(new[] { text, frames[0], noise, damaged, frames[2], frames[3] });
        var lines = new List<string>();
        using (var r = Create())
        {
            r.TextLine += lines.Add;
            var port = new FakePort(data);
            var got = new List<byte[]>();
            byte[] f;
            // 2 s: longer than the core's 1.2 s stall timeout, so a false start in the noise is released
            while ((f = r.NextFrameAsync(port, 2000).GetAwaiter().GetResult()) != null)
                got.Add(f);
            Check(lines.Count == 1 && lines[0] == "PX4 bootloader v5", "one text line (short ones are dropped)");
            Check(got.Count == 3 && Same(got[0], frames[0]) && Same(got[1], frames[2]) && Same(got[2], frames[3]),
                "the frames around the noise and the damaged frame: " + got.Count);
        }
    }

    static void TimeoutsAndClosing(List<byte[]> frames)
    {
        using (var r = Create())
        {
            var quiet = new FakePort(new byte[0]);
            var start = DateTime.UtcNow;
            Check(r.NextFrameAsync(quiet, 50).GetAwaiter().GetResult() == null, "nothing on a quiet port");
            var waited = (DateTime.UtcNow - start).TotalMilliseconds;
            Check(waited >= 45 && waited < 1000, "after the timeout: " + waited + " ms");

            var closed = new FakePort(frames[0]) { IsOpen = false };
            Check(r.NextFrameAsync(closed, 1000).GetAwaiter().GetResult() == null, "nothing from a closed port");
        }

        var disposed = Create();
        disposed.Dispose();
        Check(disposed.NextFrameAsync(new FakePort(frames[0]), 100).GetAwaiter().GetResult() == null, "nothing once disposed");
    }

    static int Main(string[] args)
    {
        if (args.Length != 1)
        {
            Console.WriteLine("usage: MpCoreReceiverTest flight.tlog");
            return 2;
        }

        var frames = ReadTlog(args[0]);
        FlightLog(frames);
        ReadsLittleAndAnnouncesParsedFrames(frames);
        TextLinesAndNoise(frames);
        TimeoutsAndClosing(frames);
        Console.WriteLine(failures == 0 ? "ok" : "FAIL (" + failures + " failed checks)");
        return failures == 0 ? 0 : 1;
    }
}
