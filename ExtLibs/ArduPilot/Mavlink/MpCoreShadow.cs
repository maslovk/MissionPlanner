using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Text;
using MissionPlanner.Mavlink.Native;

namespace MissionPlanner.ArduPilot.Mavlink
{
    /// <summary>
    /// Runs the C++ core (ExtLibs/mpcore) beside MAVLinkInterface's own parser on exactly the bytes
    /// that parser reads, and logs where the two disagree: frames one of them did not produce and
    /// sync losses counted differently. It never changes what MAVLinkInterface does. See
    /// ExtLibs/mpcore/DESIGN.md (phase 1, shadow mode).
    /// </summary>
    public sealed class MpCoreShadow : IDisposable
    {
        public sealed class Report
        {
            public long Matched;
            /// <summary>Frames MAVLinkInterface accepted that the core did not produce.</summary>
            public long MissedByCore;
            /// <summary>Frames the core produced that MAVLinkInterface did not accept.</summary>
            public long ExtraInCore;
            public long SyncLossChecks;
            public long SyncLossMismatches;

            public override string ToString()
            {
                return string.Format("matched {0}, missed by core {1}, extra in core {2}, sync loss mismatches {3}/{4}",
                    Matched, MissedByCore, ExtraInCore, SyncLossMismatches, SyncLossChecks);
            }
        }

        // longer than the core's 1.2 s stall timeout, so a frame it holds back is not reported early
        const long PendingLimitUs = 2500000;
        const int PendingMax = 1000;
        const long SummaryIntervalUs = 60000000;
        const int DetailLogsMax = 50;

        struct Pending
        {
            public byte[] Frame;
            public long Us;
        }

        struct SyncBaseline
        {
            public float Parser;
            public float Core;
        }

        readonly object _sync = new object();
        readonly Action<string> _log;
        readonly Stopwatch _clock = Stopwatch.StartNew();
        readonly List<Pending> _fromCore = new List<Pending>();
        readonly List<Pending> _fromParser = new List<Pending>();
        readonly Dictionary<ulong, SyncBaseline> _syncBaselines = new Dictionary<ulong, SyncBaseline>();
        readonly Report _report = new Report();
        MpLink _link;
        int _detailLogs;
        long _lastSummaryUs;
        long _lastSummaryEvents;

        MpCoreShadow(uint gcsSysId, byte gcsCompId, IEnumerable<byte[]> signingKeys, Action<string> log)
        {
            _log = log ?? (s => { });
            _link = new MpLink(gcsSysId, gcsCompId);
            if (signingKeys != null)
            {
                foreach (var key in signingKeys)
                {
                    if (key != null && key.Length == 32)
                        _link.AddSigningKey(key);
                }
            }

            _link.FrameReceived += OnCoreFrame;
        }

        /// <summary>
        /// Starts a shadow core, or returns null (and logs why) if the native library is missing or
        /// incompatible.
        /// </summary>
        public static MpCoreShadow TryCreate(uint gcsSysId, byte gcsCompId, IEnumerable<byte[]> signingKeys,
            Action<string> log)
        {
            try
            {
                MpCoreNativeLoader.Load();
                var shadow = new MpCoreShadow(gcsSysId, gcsCompId, signingKeys, log);
                shadow._log("mpcore shadow: started, " + Marshal.PtrToStringAnsi(MpCoreNative.mp_version_string()));
                return shadow;
            }
            catch (Exception ex)
            {
                log?.Invoke("mpcore shadow: not available: " + ex.Message);
                return null;
            }
        }

        public long NowUs => _clock.Elapsed.Ticks / (TimeSpan.TicksPerMillisecond / 1000);

        /// <summary>A copy of the counters so far.</summary>
        public Report Counters
        {
            get
            {
                lock (_sync)
                {
                    return new Report
                    {
                        Matched = _report.Matched,
                        MissedByCore = _report.MissedByCore,
                        ExtraInCore = _report.ExtraInCore,
                        SyncLossChecks = _report.SyncLossChecks,
                        SyncLossMismatches = _report.SyncLossMismatches,
                    };
                }
            }
        }

        /// <summary>Bytes the parser just read from the link, in order.</summary>
        public void BytesRead(byte[] buffer, int offset, int count)
        {
            if (count <= 0)
                return;
            lock (_sync)
            {
                if (_link == null)
                    return;
                Guard(() => _link.Feed(buffer, offset, count, NowUs));
            }
        }

        /// <summary>A frame the parser accepted (CRC and signature checked), as its exact bytes.</summary>
        public void FrameAccepted(byte[] frame)
        {
            if (frame == null || frame.Length == 0)
                return;
            lock (_sync)
            {
                if (_link == null)
                    return;
                Guard(() =>
                {
                    long now = NowUs;
                    _link.Poll(now);
                    Expire(now);
                    if (TakeMatch(_fromCore, frame, "extra in core", ref _report.ExtraInCore))
                        _report.Matched++;
                    else
                        Add(_fromParser, frame, now, "missed by core", ref _report.MissedByCore);
                    Summary(now);
                });
            }
        }

        /// <summary>
        /// Compares the parser's sync loss count for a peer with the core's. Only the growth since
        /// the first check is compared, since the parser's count may predate this shadow. Called
        /// after the parser has counted a frame.
        /// </summary>
        public void CheckSyncLoss(uint sysid, byte compid, float parserSyncLost)
        {
            lock (_sync)
            {
                // only while both have seen the same frames
                if (_link == null || _fromCore.Count != 0 || _fromParser.Count != 0)
                    return;
                Guard(() =>
                {
                    MpCoreNative.mp_peer_stats peer;
                    if (!_link.TryGetPeer(sysid, compid, out peer))
                        return;
                    ulong key = ((ulong)sysid << 8) | compid;
                    SyncBaseline baseline;
                    if (!_syncBaselines.TryGetValue(key, out baseline))
                    {
                        _syncBaselines[key] = new SyncBaseline { Parser = parserSyncLost, Core = peer.sync_lost };
                        return;
                    }

                    _report.SyncLossChecks++;
                    float parserGrowth = parserSyncLost - baseline.Parser;
                    float coreGrowth = peer.sync_lost - baseline.Core;
                    if (parserGrowth != coreGrowth)
                    {
                        _report.SyncLossMismatches++;
                        Detail(string.Format("sync loss differs for {0}-{1}: parser +{2}, core +{3}", sysid, compid,
                            parserGrowth, coreGrowth));
                        _syncBaselines[key] = new SyncBaseline { Parser = parserSyncLost, Core = peer.sync_lost };
                    }
                });
            }
        }

        /// <summary>Settles everything still pending (for example at the end of a log).</summary>
        public void Finish()
        {
            lock (_sync)
            {
                if (_link == null)
                    return;
                Guard(() =>
                {
                    long later = NowUs + PendingLimitUs * 2;
                    _link.Poll(later);
                    Expire(later);
                });
            }
        }

        public void Dispose()
        {
            lock (_sync)
            {
                if (_link == null)
                    return;
                Finish();
                _log("mpcore shadow: stopped, " + _report);
                try
                {
                    _link?.Dispose();  // Finish may have disabled it already
                }
                catch (Exception ex)
                {
                    _log("mpcore shadow: " + ex);
                }

                _link = null;
            }
        }

        void OnCoreFrame(MpFrame frame)
        {
            // the parser cannot check these either, so it never accepts them
            if ((frame.Flags & MpCoreNative.FrameFlags.UnknownMsgid) != 0)
                return;
            long now = NowUs;
            if (TakeMatch(_fromParser, frame.Buffer, "missed by core", ref _report.MissedByCore))
                _report.Matched++;
            else
                Add(_fromCore, frame.Buffer, now, "extra in core", ref _report.ExtraInCore);
        }

        // Both sides see the same bytes in the same order, so frames match in order: whatever is
        // queued before the match was only produced by that side.
        bool TakeMatch(List<Pending> queue, byte[] frame, string unmatched, ref long unmatchedCount)
        {
            for (int i = 0; i < queue.Count; i++)
            {
                if (!SameBytes(queue[i].Frame, frame))
                    continue;
                for (int j = 0; j < i; j++)
                {
                    unmatchedCount++;
                    Detail(unmatched + ": " + Describe(queue[j].Frame));
                }

                queue.RemoveRange(0, i + 1);
                return true;
            }

            return false;
        }

        void Add(List<Pending> queue, byte[] frame, long now, string unmatched, ref long unmatchedCount)
        {
            if (queue.Count >= PendingMax)
            {
                unmatchedCount++;
                Detail(unmatched + ": " + Describe(queue[0].Frame));
                queue.RemoveAt(0);
            }

            queue.Add(new Pending { Frame = frame, Us = now });
        }

        void Expire(long now)
        {
            while (_fromParser.Count > 0 && now - _fromParser[0].Us > PendingLimitUs)
            {
                _report.MissedByCore++;
                Detail("missed by core: " + Describe(_fromParser[0].Frame));
                _fromParser.RemoveAt(0);
            }

            while (_fromCore.Count > 0 && now - _fromCore[0].Us > PendingLimitUs)
            {
                _report.ExtraInCore++;
                Detail("extra in core: " + Describe(_fromCore[0].Frame));
                _fromCore.RemoveAt(0);
            }
        }

        void Summary(long now)
        {
            if (now - _lastSummaryUs < SummaryIntervalUs)
                return;
            long events = _report.MissedByCore + _report.ExtraInCore + _report.SyncLossMismatches;
            if (_lastSummaryUs == 0 || events != _lastSummaryEvents)
                _log("mpcore shadow: " + _report);
            _lastSummaryUs = now;
            _lastSummaryEvents = events;
        }

        void Detail(string message)
        {
            if (_detailLogs >= DetailLogsMax)
                return;
            _detailLogs++;
            _log("mpcore shadow: " + message + (_detailLogs == DetailLogsMax ? " (no more details)" : ""));
        }

        // A failure in the shadow must never reach the link: log it and stop shadowing.
        void Guard(Action action)
        {
            try
            {
                action();
            }
            catch (Exception ex)
            {
                _log("mpcore shadow: disabled after an error: " + ex);
                try
                {
                    _link.Dispose();
                }
                catch
                {
                }

                _link = null;
            }
        }

        static bool SameBytes(byte[] a, byte[] b)
        {
            if (a.Length != b.Length)
                return false;
            for (int i = 0; i < a.Length; i++)
            {
                if (a[i] != b[i])
                    return false;
            }

            return true;
        }

        static string Describe(byte[] f)
        {
            var sb = new StringBuilder();
            if (f.Length >= 10 && f[0] == 0xFD)
            {
                bool wide = (f[2] & 0x02) != 0;
                uint sysid = wide && f.Length >= 13 ? BitConverter.ToUInt32(f, 5) : f[5];
                int c = wide ? 9 : 6;
                if (f.Length > c + 3)
                    sb.AppendFormat("v2 sys {0} comp {1} msg {2}", sysid, f[c], f[c + 1] | f[c + 2] << 8 | f[c + 3] << 16);
            }
            else if (f.Length >= 6 && f[0] == 0xFE)
            {
                sb.AppendFormat("v1 sys {0} comp {1} msg {2}", f[3], f[4], f[5]);
            }

            sb.AppendFormat(" len {0}:", f.Length);
            for (int i = 0; i < f.Length && i < 24; i++)
                sb.AppendFormat(" {0:x2}", f[i]);
            return sb.ToString();
        }
    }
}
