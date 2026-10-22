using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using MissionPlanner.Mavlink.Native;

// Runs the C# binding against the native library: struct layouts, callbacks, a round trip with
// 32-bit IDs, and signing. Exit code 0 on success.
static class MpCoreSmoke
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

    static readonly byte[] Heartbeat = { 0, 0, 0, 0, 2, 3, 81, 4, 3 };

    static byte[] CommandLong(uint target, byte component)
    {
        var p = new byte[33];
        p[28] = 44; p[29] = 1; // command 300
        p[30] = target > 255 ? (byte)255 : (byte)target;
        p[31] = component;
        p[32] = 1;
        return p;
    }

    static void Layouts()
    {
        Check(MpCoreNative.mp_abi_version() == MpCoreNative.AbiVersion, "ABI version");
        Check(Marshal.PtrToStringAnsi(MpCoreNative.mp_version_string()).StartsWith("mpcore"), "version string");

        // the native init writes its own sizeof
        var config = new MpCoreNative.mp_link_config();
        MpCoreNative.mp_link_config_init(ref config);
        Check(config.struct_size == Marshal.SizeOf(typeof(MpCoreNative.mp_link_config)), "mp_link_config size " + config.struct_size);
        Check(config.gcs_sysid == 255 && config.gcs_compid == 190, "mp_link_config defaults");

        var info = new MpCoreNative.mp_msg_info { struct_size = (uint)Marshal.SizeOf(typeof(MpCoreNative.mp_msg_info)) };
        Check(MpCoreNative.mp_msg_info_get(76, ref info) == MpCoreNative.MP_OK, "mp_msg_info_get");
        Check(Marshal.PtrToStringAnsi(info.name) == "COMMAND_LONG", "message name");
        Check(info.crc_extra == 152 && info.min_len == 33 && info.target_system_ofs == 30 && info.target_component_ofs == 31,
            "mp_msg_info fields");

        // a frame through the raw API: the native side stamps sizeof(mp_frame)
        uint frameSize = 0;
        MpCoreNative.mp_frame_fn onFrame = (IntPtr user, ref MpCoreNative.mp_frame f) => frameSize = f.struct_size;
        var callbacks = new MpCoreNative.mp_link_callbacks
        {
            struct_size = (uint)Marshal.SizeOf(typeof(MpCoreNative.mp_link_callbacks)),
            on_frame = Marshal.GetFunctionPointerForDelegate(onFrame),
        };
        IntPtr link = MpCoreNative.mp_link_create(ref config, ref callbacks);
        Check(link != IntPtr.Zero, "mp_link_create");
        // a heartbeat from an encoder link, so this needs no CRC code here
        byte[] hb = null;
        using (var encoder = new MpLink(1, 1))
        {
            encoder.Write += b => hb = b;
            encoder.Send(0, Heartbeat, 255, 190, true);
        }
        MpCoreNative.mp_link_feed(link, hb, (UIntPtr)hb.Length, 0);
        Check(frameSize == Marshal.SizeOf(typeof(MpCoreNative.mp_frame)), "mp_frame size " + frameSize + " vs " + Marshal.SizeOf(typeof(MpCoreNative.mp_frame)));
        MpCoreNative.mp_link_destroy(link);
        GC.KeepAlive(onFrame);
    }

    static void RoundTrip()
    {
        const uint wide = 0x80000001;
        using (var gcs = new MpLink())
        using (var vehicle = new MpLink(wide, 1))
        {
            var toGcs = new List<byte>();
            var toVehicle = new List<byte>();
            var atGcs = new List<MpFrame>();
            var atVehicle = new List<MpFrame>();
            var lines = new List<string>();
            vehicle.Write += b => toGcs.AddRange(b);
            gcs.Write += b => toVehicle.AddRange(b);
            gcs.FrameReceived += atGcs.Add;
            vehicle.FrameReceived += atVehicle.Add;
            gcs.TextLine += lines.Add;

            for (int i = 0; i < 3; i++)
                Check(vehicle.Send(0, Heartbeat, 255, 190) > 0, "vehicle Send");
            var text = System.Text.Encoding.ASCII.GetBytes("Bootloader v2\r\n");
            gcs.Feed(text, 0, text.Length, 1000);
            Check(gcs.Feed(toGcs.ToArray(), 0, toGcs.Count, 123456) == 3, "Feed returns frames");
            Check(atGcs.Count == 3, "frames at GCS");
            Check(lines.Count == 1 && lines[0] == "Bootloader v2", "text line");
            var f = atGcs[2];
            Check(f.SysId == wide && f.CompId == 1 && f.MsgId == 0 && f.Seq == 2, "frame header fields");
            Check(f.IsMavlink2 && f.RxTimeUs == 123456, "frame flags/time");
            Check(f.PayloadOffset == 13 && f.PayloadLength == 9, "payload position " + f.PayloadOffset + "/" + f.PayloadLength);
            Check(f.Buffer.Length == 13 + 9 + 2 && f.Buffer[0] == 0xFD, "frame bytes");

            MpCoreNative.mp_peer_stats peer;
            Check(gcs.TryGetPeer(wide, 1, out peer), "TryGetPeer");
            Check(peer.sysid == wide && peer.compid == 1 && peer.mavlink2 == 1 && peer.last_seq == 2, "peer identity");
            Check(peer.frames == 3 && peer.last_rx_us == 123456 && peer.link_quality == 100f && peer.packets_lost == 0f,
                "peer stats (last fields prove the layout)");
            Check(!gcs.TryGetPeer(wide & 0xff, 1, out peer), "no 8-bit alias");

            var stats = gcs.Stats;
            Check(stats.frames_in == 3 && stats.mavlink2_in == 3 && stats.text_lines == 1, "link stats");
            Check(stats.bytes_in == (ulong)(toGcs.Count + text.Length), "link stats bytes");

            // a command to the wide vehicle carries TARGET32
            Check(gcs.Send(76, CommandLong(wide, 1), wide, 1) > 0, "GCS Send COMMAND_LONG");
            vehicle.Feed(toVehicle.ToArray(), 0, toVehicle.Count, 200000);
            Check(atVehicle.Count == 1, "frames at vehicle");
            if (atVehicle.Count == 1)
            {
                var c = atVehicle[0];
                Check(c.MsgId == 76 && c.TargetSysId == wide && c.TargetCompId == 1, "wide target");
                Check((c.Flags & MpCoreNative.FrameFlags.HasTarget) != 0 && (c.Flags & MpCoreNative.FrameFlags.NotForUs) == 0, "target flags");
                Check(c.Buffer[c.PayloadOffset + 30] == 255, "payload target sentinel");
            }

            Check(gcs.Send(0xFFFFFF, null, 1, 1) == MpCoreNative.MP_ERR_UNKNOWN_MSG, "unknown msgid error");
        }
    }

    static void Signing()
    {
        var key = new byte[32];
        for (int i = 0; i < key.Length; i++)
            key[i] = (byte)(i * 7);
        using (var gcs = new MpLink())
        using (var vehicle = new MpLink(1, 1))
        {
            var wire = new List<byte>();
            var received = new List<MpFrame>();
            gcs.Write += b => wire.AddRange(b);
            vehicle.FrameReceived += received.Add;
            vehicle.AddSigningKey(key);
            gcs.SetPeerSigning(1, 1, key, 3);
            gcs.Send(0, Heartbeat, 1, 1);
            vehicle.Feed(wire.ToArray(), 0, wire.Count, 0);
            Check(received.Count == 1 && (received[0].Flags & MpCoreNative.FrameFlags.SignatureOk) != 0, "signed frame verified");
            MpCoreNative.mp_peer_stats peer;
            Check(vehicle.TryGetPeer(255, 190, out peer) && peer.signing == 1, "vehicle signs back");
        }
    }

    static int Main()
    {
        Layouts();
        RoundTrip();
        Signing();
        Console.WriteLine((failures == 0 ? "ok" : "FAIL") + " (" + failures + " failed checks, " + IntPtr.Size * 8 + "-bit)");
        return failures == 0 ? 0 : 1;
    }
}
