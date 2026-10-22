using System;
using System.Runtime.InteropServices;

namespace MissionPlanner.Mavlink.Native
{
    /// <summary>
    /// P/Invoke declarations for mpcore.h. Layouts mirror the C structs field for field; keep
    /// both in step (MpCoreSmoke checks every size against the native library).
    /// </summary>
    public static class MpCoreNative
    {
        public const string Library = "mpcore";
        public const uint AbiVersion = 1;

        public const int MP_OK = 0;
        public const int MP_ERR_ARGUMENT = -1;
        public const int MP_ERR_UNKNOWN_MSG = -2;
        public const int MP_ERR_PAYLOAD_SIZE = -3;
        public const int MP_ERR_UNREPRESENTABLE = -4;
        public const int MP_ERR_NOT_FOUND = -5;

        [Flags]
        public enum FrameFlags : uint
        {
            None = 0,
            Mavlink2 = 0x0001,
            Signed = 0x0002,
            SignatureOk = 0x0004,
            UnknownMsgid = 0x0008,
            NotForUs = 0x0010,
            HasTarget = 0x0020,
        }

        public const uint MP_SEND_FORCE_MAVLINK2 = 0x0001;
        public const int MPCORE_MAX_FRAME_LEN = 287;

        [StructLayout(LayoutKind.Sequential)]
        public struct mp_frame
        {
            public uint struct_size;
            public FrameFlags flags;
            public uint msgid;
            public uint sysid;
            public uint target_sysid;
            public byte compid;
            public byte target_compid;
            public byte seq;
            public byte incompat_flags;
            public byte compat_flags;
            public byte payload_len;
            public ushort frame_len;
            public IntPtr frame;
            public IntPtr payload;
            public long rx_time_us;
        }

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate void mp_frame_fn(IntPtr user, ref mp_frame frame);

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate void mp_write_fn(IntPtr user, IntPtr data, UIntPtr len);

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate void mp_text_fn(IntPtr user, IntPtr line, UIntPtr len);

        [StructLayout(LayoutKind.Sequential)]
        public struct mp_link_callbacks
        {
            public uint struct_size;
            public IntPtr user;
            public IntPtr on_frame;
            public IntPtr on_write;
            public IntPtr on_text;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct mp_link_config
        {
            public uint struct_size;
            public uint gcs_sysid;
            public byte gcs_compid;
            public byte ignore_signatures;
            public byte reserved0;
            public byte reserved1;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct mp_peer_stats
        {
            public uint struct_size;
            public uint sysid;
            public byte compid;
            public byte mavlink2;
            public byte signing;
            public byte last_seq;
            public float packets_lost;
            public float packets_not_lost;
            public float sync_lost;
            public float link_quality;
            public ulong frames;
            public long last_rx_us;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct mp_link_stats
        {
            public uint struct_size;
            public uint reserved;
            public ulong bytes_in;
            public ulong bytes_out;
            public ulong frames_in;
            public ulong frames_out;
            public ulong mavlink1_in;
            public ulong mavlink2_in;
            public ulong signed_in;
            public ulong crc_errors;
            public ulong unknown_msgid;
            public ulong parse_errors;
            public ulong signature_failures;
            public ulong text_lines;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct mp_msg_info
        {
            public uint struct_size;
            public uint msgid;
            public IntPtr name;
            public byte crc_extra;
            public byte min_len;
            public byte max_len;
            public byte target_system_ofs;
            public byte target_component_ofs;
            public byte reserved0;
            public byte reserved1;
            public byte reserved2;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct mp_encode_args
        {
            public uint struct_size;
            public uint msgid;
            public uint src_sysid;
            public uint target_sysid;
            public byte src_compid;
            public byte seq;
            public byte mavlink1;
            public byte sign;
            public byte link_id;
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 11)]
            public byte[] reserved;
            public ulong timestamp;
            public IntPtr key;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct mp_param
        {
            public uint struct_size;
            public IntPtr name;
            public float value;
            public byte type;
            public byte duplicate;
            public ushort index;
            public ushort count;
            public ushort reserved;
        }

        [StructLayout(LayoutKind.Sequential)]
        public struct mp_param_download_status
        {
            public uint struct_size;
            public uint received;
            public uint total;
            public uint full_list_requests;
            public uint single_requests;
            public byte done;
            public byte one_by_one;
            public byte reserved0;
            public byte reserved1;
        }

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate void mp_message_fn(IntPtr user, uint msgid, IntPtr payload, UIntPtr len);

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate void mp_param_fn(IntPtr user, ref mp_param param);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr mp_param_download_create(uint sysid, byte compid, mp_message_fn send,
            mp_param_fn on_param, IntPtr user);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mp_param_download_destroy(IntPtr download);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_param_download_start(IntPtr download, long now_us);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_param_download_frame(IntPtr download, byte[] frame, UIntPtr len, long now_us);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_param_download_tick(IntPtr download, long now_us);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_param_download_status_get(IntPtr download, ref mp_param_download_status status);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint mp_abi_version();

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr mp_version_string();

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_msg_info_get(uint msgid, ref mp_msg_info info);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_frame_encode(ref mp_encode_args args, byte[] payload, UIntPtr payload_len,
            byte[] output, UIntPtr output_capacity);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mp_link_config_init(ref mp_link_config config);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr mp_link_create(ref mp_link_config config, ref mp_link_callbacks callbacks);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mp_link_destroy(IntPtr link);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern UIntPtr mp_link_feed(IntPtr link, byte[] data, UIntPtr len, long now_us);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern UIntPtr mp_link_poll(IntPtr link, long now_us);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_link_send(IntPtr link, uint msgid, byte[] payload, UIntPtr payload_len,
            uint target_sysid, byte target_compid, uint send_flags);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_link_add_signing_key(IntPtr link, byte[] key);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mp_link_clear_signing_keys(IntPtr link);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_link_set_peer_signing(IntPtr link, uint sysid, byte compid, byte[] key, byte link_id);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_link_peer_stats(IntPtr link, uint sysid, byte compid, ref mp_peer_stats stats);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern UIntPtr mp_link_peer_count(IntPtr link);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_link_peer_at(IntPtr link, UIntPtr index, ref mp_peer_stats stats);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_link_stats_get(IntPtr link, ref mp_link_stats stats);
    }

    /// <summary>Encodes single frames with no link state: the caller makes every decision.</summary>
    public static class MpFrameEncoder
    {
        /// <summary>
        /// Returns the frame, or null with the negative MpCoreNative.MP_ERR_* in error. key (32 bytes)
        /// signs a MAVLink 2 frame when not null.
        /// </summary>
        public static byte[] Encode(uint msgId, byte[] payload, uint srcSysId, byte srcCompId, uint targetSysId,
            byte seq, bool mavlink1, byte[] key, byte linkId, ulong timestamp, out int error)
        {
            if (key != null && key.Length != 32)
                throw new ArgumentException("signing keys are 32 bytes", nameof(key));
            var args = new MpCoreNative.mp_encode_args
            {
                struct_size = (uint)Marshal.SizeOf(typeof(MpCoreNative.mp_encode_args)),
                msgid = msgId,
                src_sysid = srcSysId,
                src_compid = srcCompId,
                target_sysid = targetSysId,
                seq = seq,
                mavlink1 = (byte)(mavlink1 ? 1 : 0),
                sign = (byte)(key != null ? 1 : 0),
                link_id = linkId,
                reserved = new byte[11],
                timestamp = timestamp,
            };
            var output = new byte[MpCoreNative.MPCORE_MAX_FRAME_LEN];
            int length = payload == null ? 0 : payload.Length;
            var pinned = key != null ? GCHandle.Alloc(key, GCHandleType.Pinned) : default(GCHandle);
            int n;
            try
            {
                if (key != null)
                    args.key = pinned.AddrOfPinnedObject();
                n = MpCoreNative.mp_frame_encode(ref args, length == 0 ? null : payload, (UIntPtr)length, output,
                    (UIntPtr)output.Length);
            }
            finally
            {
                if (key != null)
                    pinned.Free();
            }

            if (n < 0)
            {
                error = n;
                return null;
            }

            error = 0;
            Array.Resize(ref output, n);
            return output;
        }
    }

    /// <summary>A PARAM_VALUE as the parameter download saw it.</summary>
    public sealed class MpParam
    {
        public string Name;
        public float Value;
        public byte Type;
        public bool Duplicate;
        public ushort Index;
        public ushort Count;
    }

    /// <summary>
    /// A vehicle's parameter list download (see mp_param_download in mpcore.h). Thread-safe: frames
    /// may come from the reader thread while Tick runs elsewhere. Events are raised inside the calls,
    /// under the download's lock.
    /// </summary>
    public sealed class MpParamDownload : IDisposable
    {
        readonly object _sync = new object();
        readonly System.Diagnostics.Stopwatch _clock = System.Diagnostics.Stopwatch.StartNew();
        readonly MpCoreNative.mp_message_fn _onSend;
        readonly MpCoreNative.mp_param_fn _onParam;
        IntPtr _download;

        /// <summary>A message to send to the vehicle: msgid and payload in wire order.</summary>
        public event Action<uint, byte[]> Send;

        /// <summary>Every PARAM_VALUE from the vehicle, new or a duplicate.</summary>
        public event Action<MpParam> Param;

        public MpParamDownload(uint sysId, byte compId)
        {
            _onSend = OnSend;
            _onParam = OnParam;
            _download = MpCoreNative.mp_param_download_create(sysId, compId, _onSend, _onParam, IntPtr.Zero);
            if (_download == IntPtr.Zero)
                throw new InvalidOperationException("mp_param_download_create failed");
        }

        long NowUs => _clock.Elapsed.Ticks / (TimeSpan.TicksPerMillisecond / 1000);

        public void Start()
        {
            lock (_sync)
                if (_download != IntPtr.Zero)
                    MpCoreNative.mp_param_download_start(_download, NowUs);
        }

        /// <summary>A received frame (header to signature).</summary>
        public void Frame(byte[] frame)
        {
            if (frame == null || frame.Length == 0)
                return;
            lock (_sync)
                if (_download != IntPtr.Zero)
                    MpCoreNative.mp_param_download_frame(_download, frame, (UIntPtr)frame.Length, NowUs);
        }

        /// <summary>Sends retries when due; true once every parameter has arrived.</summary>
        public bool Tick()
        {
            lock (_sync)
                return _download != IntPtr.Zero && MpCoreNative.mp_param_download_tick(_download, NowUs) == 1;
        }

        public MpCoreNative.mp_param_download_status Status
        {
            get
            {
                var status = new MpCoreNative.mp_param_download_status
                    { struct_size = (uint)Marshal.SizeOf(typeof(MpCoreNative.mp_param_download_status)) };
                lock (_sync)
                    if (_download != IntPtr.Zero)
                        MpCoreNative.mp_param_download_status_get(_download, ref status);
                return status;
            }
        }

        public bool Done => Status.done != 0;

        void OnSend(IntPtr user, uint msgid, IntPtr payload, UIntPtr len)
        {
            var bytes = new byte[(int)len];
            Marshal.Copy(payload, bytes, 0, bytes.Length);
            // an exception must not unwind through native frames
            try { Send?.Invoke(msgid, bytes); } catch (Exception ex) { Console.Error.WriteLine("mpcore params Send: " + ex); }
        }

        void OnParam(IntPtr user, ref MpCoreNative.mp_param p)
        {
            var param = new MpParam
            {
                Name = Marshal.PtrToStringAnsi(p.name),
                Value = p.value,
                Type = p.type,
                Duplicate = p.duplicate != 0,
                Index = p.index,
                Count = p.count,
            };
            try { Param?.Invoke(param); } catch (Exception ex) { Console.Error.WriteLine("mpcore params Param: " + ex); }
        }

        public void Dispose()
        {
            lock (_sync)
            {
                if (_download != IntPtr.Zero)
                    MpCoreNative.mp_param_download_destroy(_download);
                _download = IntPtr.Zero;
            }

            GC.KeepAlive(_onSend);
            GC.KeepAlive(_onParam);
        }
    }

    /// <summary>A received frame, copied out of native memory.</summary>
    public sealed class MpFrame
    {
        public MpCoreNative.FrameFlags Flags;
        public uint MsgId;
        public uint SysId;
        public byte CompId;
        public byte Seq;
        public uint TargetSysId;
        public byte TargetCompId;
        public long RxTimeUs;
        /// <summary>The whole frame as received, header to signature.</summary>
        public byte[] Buffer;
        public int PayloadOffset;
        public int PayloadLength;

        public bool IsMavlink2 => (Flags & MpCoreNative.FrameFlags.Mavlink2) != 0;
    }

    /// <summary>
    /// One MAVLink byte stream. Not thread-safe: use it from one thread at a time, like the
    /// native link. Events are raised synchronously inside Feed/Poll/Send.
    /// </summary>
    public sealed class MpLink : IDisposable
    {
        IntPtr _link;

        // the native side holds pointers to these: they must live as long as the link
        readonly MpCoreNative.mp_frame_fn _onFrame;
        readonly MpCoreNative.mp_write_fn _onWrite;
        readonly MpCoreNative.mp_text_fn _onText;

        public event Action<MpFrame> FrameReceived;
        public event Action<byte[]> Write;
        public event Action<string> TextLine;

        public MpLink(uint gcsSysId = 255, byte gcsCompId = 190, bool ignoreSignatures = false)
        {
            if (MpCoreNative.mp_abi_version() != MpCoreNative.AbiVersion)
                throw new InvalidOperationException("mpcore ABI version mismatch");

            _onFrame = OnFrame;
            _onWrite = OnWrite;
            _onText = OnText;

            var config = new MpCoreNative.mp_link_config();
            config.struct_size = (uint)Marshal.SizeOf(typeof(MpCoreNative.mp_link_config));
            MpCoreNative.mp_link_config_init(ref config);
            config.gcs_sysid = gcsSysId;
            config.gcs_compid = gcsCompId;
            config.ignore_signatures = (byte)(ignoreSignatures ? 1 : 0);

            var callbacks = new MpCoreNative.mp_link_callbacks
            {
                struct_size = (uint)Marshal.SizeOf(typeof(MpCoreNative.mp_link_callbacks)),
                on_frame = Marshal.GetFunctionPointerForDelegate(_onFrame),
                on_write = Marshal.GetFunctionPointerForDelegate(_onWrite),
                on_text = Marshal.GetFunctionPointerForDelegate(_onText),
            };
            _link = MpCoreNative.mp_link_create(ref config, ref callbacks);
            if (_link == IntPtr.Zero)
                throw new InvalidOperationException("mp_link_create failed");
        }

        public IntPtr Handle => _link;

        // arrays are pinned by the marshaller; a slice is copied here first
        byte[] _scratch = new byte[300];

        public int Feed(byte[] buffer, int offset, int count, long nowUs)
        {
            if (buffer == null || offset < 0 || count < 0 || offset + count > buffer.Length)
                throw new ArgumentOutOfRangeException(nameof(count));
            if (count == 0)
                return (int)MpCoreNative.mp_link_poll(_link, nowUs);
            if (offset == 0)
                return (int)MpCoreNative.mp_link_feed(_link, buffer, (UIntPtr)count, nowUs);
            if (_scratch.Length < count)
                _scratch = new byte[count];
            Buffer.BlockCopy(buffer, offset, _scratch, 0, count);
            return (int)MpCoreNative.mp_link_feed(_link, _scratch, (UIntPtr)count, nowUs);
        }

        public int Poll(long nowUs)
        {
            return (int)MpCoreNative.mp_link_poll(_link, nowUs);
        }

        /// <summary>Returns the bytes written, or a negative MpCoreNative.MP_ERR_*.</summary>
        public int Send(uint msgId, byte[] payload, uint targetSysId, byte targetCompId, bool forceMavlink2 = false)
        {
            uint flags = forceMavlink2 ? MpCoreNative.MP_SEND_FORCE_MAVLINK2 : 0;
            int length = payload == null ? 0 : payload.Length;
            return MpCoreNative.mp_link_send(_link, msgId, length == 0 ? null : payload, (UIntPtr)length, targetSysId,
                targetCompId, flags);
        }

        public void AddSigningKey(byte[] key)
        {
            if (key == null || key.Length != 32)
                throw new ArgumentException("signing keys are 32 bytes", nameof(key));
            MpCoreNative.mp_link_add_signing_key(_link, key);
        }

        public void SetPeerSigning(uint sysId, byte compId, byte[] key, byte linkId)
        {
            if (key != null && key.Length != 32)
                throw new ArgumentException("signing keys are 32 bytes", nameof(key));
            MpCoreNative.mp_link_set_peer_signing(_link, sysId, compId, key, linkId);
        }

        public bool TryGetPeer(uint sysId, byte compId, out MpCoreNative.mp_peer_stats stats)
        {
            stats = new MpCoreNative.mp_peer_stats { struct_size = (uint)Marshal.SizeOf(typeof(MpCoreNative.mp_peer_stats)) };
            return MpCoreNative.mp_link_peer_stats(_link, sysId, compId, ref stats) == MpCoreNative.MP_OK;
        }

        public MpCoreNative.mp_link_stats Stats
        {
            get
            {
                var stats = new MpCoreNative.mp_link_stats { struct_size = (uint)Marshal.SizeOf(typeof(MpCoreNative.mp_link_stats)) };
                MpCoreNative.mp_link_stats_get(_link, ref stats);
                return stats;
            }
        }

        void OnFrame(IntPtr user, ref MpCoreNative.mp_frame f)
        {
            var handler = FrameReceived;
            if (handler == null)
                return;
            var frame = new MpFrame
            {
                Flags = f.flags,
                MsgId = f.msgid,
                SysId = f.sysid,
                CompId = f.compid,
                Seq = f.seq,
                TargetSysId = f.target_sysid,
                TargetCompId = f.target_compid,
                RxTimeUs = f.rx_time_us,
                Buffer = new byte[f.frame_len],
                PayloadOffset = (int)(f.payload.ToInt64() - f.frame.ToInt64()),
                PayloadLength = f.payload_len,
            };
            Marshal.Copy(f.frame, frame.Buffer, 0, f.frame_len);
            // an exception must not unwind through native frames
            try { handler(frame); } catch (Exception ex) { Console.Error.WriteLine("mpcore FrameReceived: " + ex); }
        }

        void OnWrite(IntPtr user, IntPtr data, UIntPtr len)
        {
            var handler = Write;
            if (handler == null)
                return;
            var bytes = new byte[(int)len];
            Marshal.Copy(data, bytes, 0, bytes.Length);
            try { handler(bytes); } catch (Exception ex) { Console.Error.WriteLine("mpcore Write: " + ex); }
        }

        void OnText(IntPtr user, IntPtr line, UIntPtr len)
        {
            var handler = TextLine;
            if (handler == null)
                return;
            var bytes = new byte[(int)len];
            Marshal.Copy(line, bytes, 0, bytes.Length);
            try { handler(System.Text.Encoding.ASCII.GetString(bytes)); } catch (Exception ex) { Console.Error.WriteLine("mpcore TextLine: " + ex); }
        }

        public void Dispose()
        {
            if (_link != IntPtr.Zero)
            {
                MpCoreNative.mp_link_destroy(_link);
                _link = IntPtr.Zero;
            }
            GC.KeepAlive(_onFrame);
            GC.KeepAlive(_onWrite);
            GC.KeepAlive(_onText);
        }
    }
}
