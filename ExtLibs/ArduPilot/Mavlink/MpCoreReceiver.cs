using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Threading.Tasks;
using MissionPlanner.Comms;
using MissionPlanner.Mavlink.Native;
using MissionPlanner.Utilities;

namespace MissionPlanner.ArduPilot.Mavlink
{
    /// <summary>
    /// Reads a link with the C++ core (ExtLibs/mpcore) instead of MAVLinkInterface's own byte loop:
    /// framing, resync after damaged frames and text lines come from the core, one frame per call.
    /// Everything after framing (CRC and length checks, signatures, targeting, statistics) stays in
    /// MAVLinkInterface. See ExtLibs/mpcore/DESIGN.md (phase 1, step 3).
    /// </summary>
    public sealed class MpCoreReceiver : IDisposable
    {
        // Read little at a time: bytes past the current frame stay in the port for code that reads
        // it directly after taking it over (terminal, firmware upload).
        const int ChunkSize = 64;

        readonly object _sync = new object();
        readonly Queue<byte[]> _frames = new Queue<byte[]>();
        readonly byte[] _chunk = new byte[ChunkSize];
        readonly Stopwatch _clock = Stopwatch.StartNew();
        MpLink _link;

        /// <summary>Bytes as read from the port, before parsing (for the raw log).</summary>
        public event Action<byte[], int, int> BytesRead;

        /// <summary>A line of text outside MAVLink frames (bootloader output, NMEA).</summary>
        public event Action<string> TextLine;

        MpCoreReceiver(uint gcsSysId, byte gcsCompId)
        {
            // MAVLinkInterface checks signatures itself, with its per-vehicle keys and settings
            _link = new MpLink(gcsSysId, gcsCompId, ignoreSignatures: true);
            _link.FrameReceived += f => _frames.Enqueue(f.Buffer);
            _link.TextLine += line => TextLine?.Invoke(line);
        }

        /// <summary>Returns null (and logs why) if the native library is missing or incompatible.</summary>
        public static MpCoreReceiver TryCreate(uint gcsSysId, byte gcsCompId, Action<string> log)
        {
            try
            {
                MpCoreNativeLoader.Load();
                var receiver = new MpCoreReceiver(gcsSysId, gcsCompId);
                log?.Invoke("mpcore receive: started, " + Marshal.PtrToStringAnsi(MpCoreNative.mp_version_string()));
                return receiver;
            }
            catch (Exception ex)
            {
                log?.Invoke("mpcore receive: not available, using the C# parser: " + ex.Message);
                return null;
            }
        }

        long NowUs => _clock.Elapsed.Ticks / (TimeSpan.TicksPerMillisecond / 1000);

        /// <summary>Frames already parsed and waiting, though the port may have nothing to read.</summary>
        public bool HasFrames
        {
            get
            {
                lock (_sync)
                    return _frames.Count > 0;
            }
        }

        /// <summary>
        /// The next complete frame (header to signature), or null if the port is closed or no frame
        /// arrived within timeoutMs of the last byte read.
        /// </summary>
        public async Task<byte[]> NextFrameAsync(ICommsSerial port, int timeoutMs)
        {
            var deadline = DateTime.UtcNow.AddMilliseconds(timeoutMs);
            while (true)
            {
                lock (_sync)
                {
                    if (_link == null)
                        return null;
                    if (_frames.Count > 0)
                        return _frames.Dequeue();
                }

                if (port == null || !port.IsOpen)
                    return null;

                int available = port.BytesToRead;
                if (available > 0)
                {
                    int n = port.Read(_chunk, 0, Math.Min(available, _chunk.Length));
                    if (n > 0)
                    {
                        BytesRead?.Invoke(_chunk, 0, n);
                        lock (_sync)
                        {
                            if (_link == null)
                                return null;
                            _link.Feed(_chunk, 0, n, NowUs);
                        }

                        deadline = DateTime.UtcNow.AddMilliseconds(timeoutMs);
                    }

                    continue;
                }

                // a frame start that stalled may be hiding the frames after it
                lock (_sync)
                {
                    if (_link == null)
                        return null;
                    _link.Poll(NowUs);
                    if (_frames.Count > 0)
                        continue;
                }

                if (DateTime.UtcNow > deadline)
                    return null;
                await PollDelay.Wait().ConfigureAwait(false);
            }
        }

        public void Dispose()
        {
            lock (_sync)
            {
                _link?.Dispose();
                _link = null;
                _frames.Clear();
            }
        }
    }
}
