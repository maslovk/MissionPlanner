using System;
using System.Runtime.InteropServices;
using System.Text;

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

        public const int MP_EXCHANGE_WAITING = 0;
        public const int MP_EXCHANGE_DONE = 1;
        public const int MP_EXCHANGE_TIMED_OUT = 2;

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr mp_param_set_create(uint sysid, byte compid, string name, float value, byte type,
            mp_message_fn send, IntPtr user);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl, CharSet = CharSet.Ansi)]
        public static extern IntPtr mp_param_read_create(uint sysid, byte compid, string name, short index,
            mp_message_fn send, IntPtr user);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mp_exchange_destroy(IntPtr exchange);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_exchange_start(IntPtr exchange, long now_us);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_exchange_frame(IntPtr exchange, byte[] frame, UIntPtr len, long now_us);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_exchange_tick(IntPtr exchange, long now_us);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_exchange_reply(IntPtr exchange, byte[] output, UIntPtr capacity);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint mp_exchange_sends(IntPtr exchange);

        public const int MP_MISSION_RUNNING = 0;
        public const int MP_MISSION_DONE = 1;
        public const int MP_MISSION_FAILED = 2;

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr mp_mission_download_create(uint sysid, byte compid, uint gcs_sysid, byte gcs_compid,
            byte mission_type, byte use_int, mp_message_fn send, IntPtr user);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mp_mission_download_destroy(IntPtr download);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_mission_download_start(IntPtr download, long now_us);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_mission_download_frame(IntPtr download, byte[] frame, UIntPtr len, long now_us);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_mission_download_tick(IntPtr download, long now_us);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_mission_download_count(IntPtr download);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern UIntPtr mp_mission_download_received(IntPtr download);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_mission_download_item(IntPtr download, UIntPtr index, byte[] output, UIntPtr capacity);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr mp_mission_upload_create(uint sysid, byte compid, uint gcs_sysid, byte gcs_compid,
            byte mission_type, mp_message_fn send, IntPtr user);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mp_mission_upload_destroy(IntPtr upload);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_mission_upload_add(IntPtr upload, uint msgid, byte[] payload, UIntPtr len);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_mission_upload_start(IntPtr upload, long now_us);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_mission_upload_frame(IntPtr upload, byte[] frame, UIntPtr len, long now_us);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_mission_upload_tick(IntPtr upload, long now_us);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_mission_upload_result(IntPtr upload);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_mission_upload_count_accepted(IntPtr upload);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_mission_upload_highest_requested(IntPtr upload);

        public const int MP_FTP_RUNNING = 0;
        public const int MP_FTP_DONE = 1;
        public const int MP_FTP_FAILED = 2;
        public const int MP_FTP_MAX_DATA = 239;

        public const byte MP_FTP_TERMINATE_SESSION = 1;
        public const byte MP_FTP_RESET_SESSIONS = 2;
        public const byte MP_FTP_REMOVE_FILE = 8;
        public const byte MP_FTP_CREATE_DIRECTORY = 9;
        public const byte MP_FTP_REMOVE_DIRECTORY = 10;
        public const byte MP_FTP_TRUNCATE_FILE = 12;
        public const byte MP_FTP_RENAME = 13;
        public const byte MP_FTP_CALC_FILE_CRC32 = 14;

        [StructLayout(LayoutKind.Sequential)]
        public struct mp_ftp_entry
        {
            public uint struct_size;
            public byte kind;
            public byte reserved0, reserved1, reserved2;
            public uint mtime;
            public uint reserved3;
            public ulong size;
        }

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr mp_ftp_create(uint sysid, byte compid, uint gcs_sysid, byte gcs_compid,
            mp_message_fn send, IntPtr user);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mp_ftp_destroy(IntPtr ftp);

        // paths: NUL-terminated UTF-8
        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_ftp_get_file(IntPtr ftp, byte[] path, byte read_size, int burst, long now_us);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_ftp_put_file(IntPtr ftp, byte[] path, byte[] data, UIntPtr len, long now_us);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_ftp_list(IntPtr ftp, byte[] dir, long now_us);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_ftp_command(IntPtr ftp, byte opcode, byte[] path, byte[] path2, long now_us);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_ftp_cancel(IntPtr ftp);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_ftp_frame(IntPtr ftp, byte[] frame, UIntPtr len, long now_us);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_ftp_tick(IntPtr ftp, long now_us);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_ftp_nak(IntPtr ftp);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_ftp_nak_errno(IntPtr ftp);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_ftp_nak_opcode(IntPtr ftp);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_ftp_opcode(IntPtr ftp);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint mp_ftp_value(IntPtr ftp);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern long mp_ftp_file_size(IntPtr ftp);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern ulong mp_ftp_progress(IntPtr ftp);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern UIntPtr mp_ftp_data(IntPtr ftp, byte[] output, UIntPtr capacity);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern UIntPtr mp_ftp_entry_count(IntPtr ftp);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_ftp_entry_get(IntPtr ftp, UIntPtr index, ref mp_ftp_entry entry);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_ftp_entry_name(IntPtr ftp, UIntPtr index, byte[] output, UIntPtr capacity);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern ushort mp_ftp_next_seq(IntPtr ftp);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mp_ftp_set_next_seq(IntPtr ftp, ushort seq);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint mp_ftp_requests_sent(IntPtr ftp);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern int mp_ftp_plain_listing(IntPtr ftp);

        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)]
        public static extern void mp_ftp_set_plain_listing(IntPtr ftp, int plain);

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

    /// <summary>
    /// One request and its reply, resent when the reply is late (see mp_exchange in mpcore.h).
    /// Thread-safe like MpParamDownload.
    /// </summary>
    public sealed class MpExchange : IDisposable
    {
        public enum Result
        {
            Waiting = MpCoreNative.MP_EXCHANGE_WAITING,
            Done = MpCoreNative.MP_EXCHANGE_DONE,
            TimedOut = MpCoreNative.MP_EXCHANGE_TIMED_OUT,
        }

        readonly object _sync = new object();
        readonly System.Diagnostics.Stopwatch _clock = System.Diagnostics.Stopwatch.StartNew();
        readonly MpCoreNative.mp_message_fn _onSend;
        readonly Action<uint, byte[]> _send;
        IntPtr _exchange;

        MpExchange(Action<uint, byte[]> send, Func<MpCoreNative.mp_message_fn, IntPtr> create)
        {
            _send = send;
            _onSend = OnSend;
            _exchange = create(_onSend);
            if (_exchange == IntPtr.Zero)
                throw new ArgumentException("the request could not be created");
        }

        /// <summary>PARAM_SET; done when the PARAM_VALUE with the same name comes back.</summary>
        public static MpExchange ParamSet(uint sysId, byte compId, string name, float value, byte type,
            Action<uint, byte[]> send)
        {
            return new MpExchange(send, fn => MpCoreNative.mp_param_set_create(sysId, compId, name, value, type, fn, IntPtr.Zero));
        }

        /// <summary>PARAM_REQUEST_READ by name (index -1) or by index.</summary>
        public static MpExchange ParamRead(uint sysId, byte compId, string name, short index, Action<uint, byte[]> send)
        {
            return new MpExchange(send, fn => MpCoreNative.mp_param_read_create(sysId, compId, name, index, fn, IntPtr.Zero));
        }

        long NowUs => _clock.Elapsed.Ticks / (TimeSpan.TicksPerMillisecond / 1000);

        public void Start()
        {
            lock (_sync)
                if (_exchange != IntPtr.Zero)
                    MpCoreNative.mp_exchange_start(_exchange, NowUs);
        }

        public void Frame(byte[] frame)
        {
            if (frame == null || frame.Length == 0)
                return;
            lock (_sync)
                if (_exchange != IntPtr.Zero)
                    MpCoreNative.mp_exchange_frame(_exchange, frame, (UIntPtr)frame.Length, NowUs);
        }

        public Result Tick()
        {
            lock (_sync)
                return _exchange == IntPtr.Zero ? Result.TimedOut : (Result)MpCoreNative.mp_exchange_tick(_exchange, NowUs);
        }

        /// <summary>The reply frame, or null before it came.</summary>
        public byte[] Reply
        {
            get
            {
                var buffer = new byte[MpCoreNative.MPCORE_MAX_FRAME_LEN];
                int n;
                lock (_sync)
                    n = _exchange == IntPtr.Zero ? 0 : MpCoreNative.mp_exchange_reply(_exchange, buffer, (UIntPtr)buffer.Length);
                if (n <= 0)
                    return null;
                Array.Resize(ref buffer, n);
                return buffer;
            }
        }

        public uint Sends
        {
            get
            {
                lock (_sync)
                    return _exchange == IntPtr.Zero ? 0 : MpCoreNative.mp_exchange_sends(_exchange);
            }
        }

        void OnSend(IntPtr user, uint msgid, IntPtr payload, UIntPtr len)
        {
            var bytes = new byte[(int)len];
            Marshal.Copy(payload, bytes, 0, bytes.Length);
            try { _send?.Invoke(msgid, bytes); } catch (Exception ex) { Console.Error.WriteLine("mpcore exchange Send: " + ex); }
        }

        public void Dispose()
        {
            lock (_sync)
            {
                if (_exchange != IntPtr.Zero)
                    MpCoreNative.mp_exchange_destroy(_exchange);
                _exchange = IntPtr.Zero;
            }

            GC.KeepAlive(_onSend);
        }
    }

    /// <summary>
    /// A mission, fence or rally list download (see mp_mission_download in mpcore.h). Thread-safe
    /// like MpParamDownload.
    /// </summary>
    public sealed class MpMissionDownload : IDisposable
    {
        public enum Result
        {
            Running = MpCoreNative.MP_MISSION_RUNNING,
            Done = MpCoreNative.MP_MISSION_DONE,
            Failed = MpCoreNative.MP_MISSION_FAILED,
        }

        readonly object _sync = new object();
        readonly System.Diagnostics.Stopwatch _clock = System.Diagnostics.Stopwatch.StartNew();
        readonly MpCoreNative.mp_message_fn _onSend;
        readonly Action<uint, byte[]> _send;
        IntPtr _download;

        public MpMissionDownload(uint sysId, byte compId, uint gcsSysId, byte gcsCompId, byte missionType, bool useInt,
            Action<uint, byte[]> send)
        {
            _send = send;
            _onSend = OnSend;
            _download = MpCoreNative.mp_mission_download_create(sysId, compId, gcsSysId, gcsCompId, missionType,
                (byte)(useInt ? 1 : 0), _onSend, IntPtr.Zero);
            if (_download == IntPtr.Zero)
                throw new InvalidOperationException("mp_mission_download_create failed");
        }

        long NowUs => _clock.Elapsed.Ticks / (TimeSpan.TicksPerMillisecond / 1000);

        public void Start()
        {
            lock (_sync)
                if (_download != IntPtr.Zero)
                    MpCoreNative.mp_mission_download_start(_download, NowUs);
        }

        public void Frame(byte[] frame)
        {
            if (frame == null || frame.Length == 0)
                return;
            lock (_sync)
                if (_download != IntPtr.Zero)
                    MpCoreNative.mp_mission_download_frame(_download, frame, (UIntPtr)frame.Length, NowUs);
        }

        public Result Tick()
        {
            lock (_sync)
                return _download == IntPtr.Zero ? Result.Failed : (Result)MpCoreNative.mp_mission_download_tick(_download, NowUs);
        }

        /// <summary>The vehicle's count, or -1 before it answered.</summary>
        public int Count
        {
            get
            {
                lock (_sync)
                    return _download == IntPtr.Zero ? -1 : MpCoreNative.mp_mission_download_count(_download);
            }
        }

        public int Received
        {
            get
            {
                lock (_sync)
                    return _download == IntPtr.Zero ? 0 : (int)MpCoreNative.mp_mission_download_received(_download);
            }
        }

        /// <summary>The item frames received, in seq order.</summary>
        public System.Collections.Generic.List<byte[]> Items
        {
            get
            {
                var items = new System.Collections.Generic.List<byte[]>();
                lock (_sync)
                {
                    int n = _download == IntPtr.Zero ? 0 : (int)MpCoreNative.mp_mission_download_received(_download);
                    for (int i = 0; i < n; i++)
                    {
                        var buffer = new byte[MpCoreNative.MPCORE_MAX_FRAME_LEN];
                        int len = MpCoreNative.mp_mission_download_item(_download, (UIntPtr)i, buffer, (UIntPtr)buffer.Length);
                        if (len <= 0)
                            break;
                        Array.Resize(ref buffer, len);
                        items.Add(buffer);
                    }
                }

                return items;
            }
        }

        void OnSend(IntPtr user, uint msgid, IntPtr payload, UIntPtr len)
        {
            var bytes = new byte[(int)len];
            Marshal.Copy(payload, bytes, 0, bytes.Length);
            try { _send?.Invoke(msgid, bytes); } catch (Exception ex) { Console.Error.WriteLine("mpcore mission Send: " + ex); }
        }

        public void Dispose()
        {
            lock (_sync)
            {
                if (_download != IntPtr.Zero)
                    MpCoreNative.mp_mission_download_destroy(_download);
                _download = IntPtr.Zero;
            }

            GC.KeepAlive(_onSend);
        }
    }

    /// <summary>
    /// A mission, fence or rally list upload (see mp_mission_upload in mpcore.h). Thread-safe like
    /// MpParamDownload. Add the items, then Start.
    /// </summary>
    public sealed class MpMissionUpload : IDisposable
    {
        readonly object _sync = new object();
        readonly System.Diagnostics.Stopwatch _clock = System.Diagnostics.Stopwatch.StartNew();
        readonly MpCoreNative.mp_message_fn _onSend;
        readonly Action<uint, byte[]> _send;
        IntPtr _upload;

        public MpMissionUpload(uint sysId, byte compId, uint gcsSysId, byte gcsCompId, byte missionType,
            Action<uint, byte[]> send)
        {
            _send = send;
            _onSend = OnSend;
            _upload = MpCoreNative.mp_mission_upload_create(sysId, compId, gcsSysId, gcsCompId, missionType, _onSend,
                IntPtr.Zero);
            if (_upload == IntPtr.Zero)
                throw new InvalidOperationException("mp_mission_upload_create failed");
        }

        long NowUs => _clock.Elapsed.Ticks / (TimeSpan.TicksPerMillisecond / 1000);

        /// <summary>An item: MISSION_ITEM_INT or MISSION_ITEM payload with its seq, target and type set.</summary>
        public void Add(uint msgId, byte[] payload)
        {
            lock (_sync)
                if (_upload == IntPtr.Zero ||
                    MpCoreNative.mp_mission_upload_add(_upload, msgId, payload, (UIntPtr)payload.Length) != MpCoreNative.MP_OK)
                    throw new InvalidOperationException("items are added before Start");
        }

        public void Start()
        {
            lock (_sync)
                if (_upload != IntPtr.Zero)
                    MpCoreNative.mp_mission_upload_start(_upload, NowUs);
        }

        public void Frame(byte[] frame)
        {
            if (frame == null || frame.Length == 0)
                return;
            lock (_sync)
                if (_upload != IntPtr.Zero)
                    MpCoreNative.mp_mission_upload_frame(_upload, frame, (UIntPtr)frame.Length, NowUs);
        }

        public MpMissionDownload.Result Tick()
        {
            lock (_sync)
                return _upload == IntPtr.Zero
                    ? MpMissionDownload.Result.Failed
                    : (MpMissionDownload.Result)MpCoreNative.mp_mission_upload_tick(_upload, NowUs);
        }

        /// <summary>The vehicle's MISSION_ACK result (MAV_MISSION_RESULT), or -1 without one.</summary>
        public int Result
        {
            get
            {
                lock (_sync)
                    return _upload == IntPtr.Zero ? -1 : MpCoreNative.mp_mission_upload_result(_upload);
            }
        }

        public bool CountAccepted
        {
            get
            {
                lock (_sync)
                    return _upload != IntPtr.Zero && MpCoreNative.mp_mission_upload_count_accepted(_upload) == 1;
            }
        }

        public int HighestRequested
        {
            get
            {
                lock (_sync)
                    return _upload == IntPtr.Zero ? -1 : MpCoreNative.mp_mission_upload_highest_requested(_upload);
            }
        }

        void OnSend(IntPtr user, uint msgid, IntPtr payload, UIntPtr len)
        {
            var bytes = new byte[(int)len];
            Marshal.Copy(payload, bytes, 0, bytes.Length);
            try { _send?.Invoke(msgid, bytes); } catch (Exception ex) { Console.Error.WriteLine("mpcore mission Send: " + ex); }
        }

        public void Dispose()
        {
            lock (_sync)
            {
                if (_upload != IntPtr.Zero)
                    MpCoreNative.mp_mission_upload_destroy(_upload);
                _upload = IntPtr.Zero;
            }

            GC.KeepAlive(_onSend);
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

    /// <summary>
    /// MAVLink FTP, one operation at a time (see mp_ftp in mpcore.h). Thread-safe like
    /// MpParamDownload: frames may come from the reader thread while another thread ticks.
    /// </summary>
    public sealed class MpFtp : IDisposable
    {
        public enum Result
        {
            Running = MpCoreNative.MP_FTP_RUNNING,
            Done = MpCoreNative.MP_FTP_DONE,
            Failed = MpCoreNative.MP_FTP_FAILED,
        }

        public struct Entry
        {
            /// <summary>'F' file, 'D' directory, 'S' an entry the vehicle skipped</summary>
            public char Kind;
            public string Name;
            public ulong Size;
            /// <summary>seconds since 1970 (UTC), 0 if not known</summary>
            public uint MTime;
        }

        readonly object _sync = new object();
        readonly System.Diagnostics.Stopwatch _clock = System.Diagnostics.Stopwatch.StartNew();
        readonly MpCoreNative.mp_message_fn _onSend;
        readonly Action<uint, byte[]> _send;
        IntPtr _ftp;

        public MpFtp(uint sysId, byte compId, uint gcsSysId, byte gcsCompId, Action<uint, byte[]> send)
        {
            _send = send;
            _onSend = OnSend;
            _ftp = MpCoreNative.mp_ftp_create(sysId, compId, gcsSysId, gcsCompId, _onSend, IntPtr.Zero);
            if (_ftp == IntPtr.Zero)
                throw new InvalidOperationException("mp_ftp_create failed");
        }

        long NowUs => _clock.Elapsed.Ticks / (TimeSpan.TicksPerMillisecond / 1000);

        static byte[] Path(string path) => path == null ? null : Encoding.UTF8.GetBytes(path + "\0");

        bool Ok(Func<IntPtr, int> start)
        {
            lock (_sync)
                return _ftp != IntPtr.Zero && start(_ftp) == MpCoreNative.MP_OK;
        }

        /// <summary>Starts a download; false while an operation runs or for a path too long.</summary>
        public bool GetFile(string path, byte readSize, bool burst) =>
            Ok(f => MpCoreNative.mp_ftp_get_file(f, Path(path), readSize, burst ? 1 : 0, NowUs));

        public bool PutFile(string path, byte[] data) =>
            Ok(f => MpCoreNative.mp_ftp_put_file(f, Path(path), data, (UIntPtr)data.Length, NowUs));

        public bool List(string dir) => Ok(f => MpCoreNative.mp_ftp_list(f, Path(dir), NowUs));

        /// <summary>One of the MpCoreNative.MP_FTP_* commands.</summary>
        public bool Command(byte opcode, string path, string path2 = null) =>
            Ok(f => MpCoreNative.mp_ftp_command(f, opcode, Path(path ?? ""), Path(path2), NowUs));

        public void Cancel()
        {
            lock (_sync)
                if (_ftp != IntPtr.Zero)
                    MpCoreNative.mp_ftp_cancel(_ftp);
        }

        public void Frame(byte[] frame)
        {
            if (frame == null || frame.Length == 0)
                return;
            lock (_sync)
                if (_ftp != IntPtr.Zero)
                    MpCoreNative.mp_ftp_frame(_ftp, frame, (UIntPtr)frame.Length, NowUs);
        }

        public Result Tick()
        {
            lock (_sync)
                return _ftp == IntPtr.Zero ? Result.Failed : (Result)MpCoreNative.mp_ftp_tick(_ftp, NowUs);
        }

        int Get(Func<IntPtr, int> get)
        {
            lock (_sync)
                return _ftp == IntPtr.Zero ? 0 : get(_ftp);
        }

        /// <summary>Once failed: the NAK code (0: a timeout or a cancel), its errno and the opcode it answered.</summary>
        public int Nak => Get(MpCoreNative.mp_ftp_nak);
        public int NakErrno => Get(MpCoreNative.mp_ftp_nak_errno);
        public int NakOpcode => Get(MpCoreNative.mp_ftp_nak_opcode);
        /// <summary>The request being sent, the last one once finished.</summary>
        public int Opcode => Get(MpCoreNative.mp_ftp_opcode);

        public uint Value
        {
            get
            {
                lock (_sync)
                    return _ftp == IntPtr.Zero ? 0 : MpCoreNative.mp_ftp_value(_ftp);
            }
        }

        /// <summary>A download: the size opened (-1 before), cut to where the file ended.</summary>
        public long FileSize
        {
            get
            {
                lock (_sync)
                    return _ftp == IntPtr.Zero ? -1 : MpCoreNative.mp_ftp_file_size(_ftp);
            }
        }

        /// <summary>A download: bytes received; an upload: bytes acknowledged.</summary>
        public ulong Progress
        {
            get
            {
                lock (_sync)
                    return _ftp == IntPtr.Zero ? 0 : MpCoreNative.mp_ftp_progress(_ftp);
            }
        }

        public uint RequestsSent
        {
            get
            {
                lock (_sync)
                    return _ftp == IntPtr.Zero ? 0 : MpCoreNative.mp_ftp_requests_sent(_ftp);
            }
        }

        /// <summary>The seq of the next request, shared with code that sends FTP requests itself.</summary>
        public ushort NextSeq
        {
            get
            {
                lock (_sync)
                    return _ftp == IntPtr.Zero ? (ushort)0 : MpCoreNative.mp_ftp_next_seq(_ftp);
            }
            set
            {
                lock (_sync)
                    if (_ftp != IntPtr.Zero)
                        MpCoreNative.mp_ftp_set_next_seq(_ftp, value);
            }
        }

        /// <summary>Listings are plain: the vehicle NAKed a timed one. Kept by the caller across objects.</summary>
        public bool PlainListing
        {
            get
            {
                lock (_sync)
                    return _ftp != IntPtr.Zero && MpCoreNative.mp_ftp_plain_listing(_ftp) == 1;
            }
            set
            {
                lock (_sync)
                    if (_ftp != IntPtr.Zero)
                        MpCoreNative.mp_ftp_set_plain_listing(_ftp, value ? 1 : 0);
            }
        }

        /// <summary>A finished download.</summary>
        public byte[] Data()
        {
            lock (_sync)
            {
                if (_ftp == IntPtr.Zero)
                    return new byte[0];
                var data = new byte[(int)MpCoreNative.mp_ftp_data(_ftp, null, UIntPtr.Zero)];
                MpCoreNative.mp_ftp_data(_ftp, data, (UIntPtr)data.Length);
                return data;
            }
        }

        /// <summary>A listing's entries.</summary>
        public Entry[] Entries()
        {
            lock (_sync)
            {
                if (_ftp == IntPtr.Zero)
                    return new Entry[0];
                var count = (int)MpCoreNative.mp_ftp_entry_count(_ftp);
                var entries = new Entry[count];
                for (int i = 0; i < count; i++)
                {
                    var e = new MpCoreNative.mp_ftp_entry { struct_size = (uint)Marshal.SizeOf(typeof(MpCoreNative.mp_ftp_entry)) };
                    MpCoreNative.mp_ftp_entry_get(_ftp, (UIntPtr)i, ref e);
                    var name = new byte[Math.Max(0, MpCoreNative.mp_ftp_entry_name(_ftp, (UIntPtr)i, null, UIntPtr.Zero))];
                    MpCoreNative.mp_ftp_entry_name(_ftp, (UIntPtr)i, name, (UIntPtr)name.Length);
                    entries[i] = new Entry { Kind = (char)e.kind, Name = Encoding.UTF8.GetString(name), Size = e.size, MTime = e.mtime };
                }

                return entries;
            }
        }

        void OnSend(IntPtr user, uint msgid, IntPtr payload, UIntPtr len)
        {
            var bytes = new byte[(int)len];
            Marshal.Copy(payload, bytes, 0, bytes.Length);
            try { _send?.Invoke(msgid, bytes); } catch (Exception ex) { Console.Error.WriteLine("mpcore ftp Send: " + ex); }
        }

        public void Dispose()
        {
            lock (_sync)
            {
                if (_ftp != IntPtr.Zero)
                    MpCoreNative.mp_ftp_destroy(_ftp);
                _ftp = IntPtr.Zero;
            }

            GC.KeepAlive(_onSend);
        }
    }
}
