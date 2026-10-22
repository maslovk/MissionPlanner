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

    // mp_encode_args crosses P/Invoke intact: the key pointer and the 64-bit timestamp after the
    // padding arrive, so a receiving link verifies the signature and reads the timestamp back.
    static void Encoder()
    {
        var key = new byte[32];
        for (int i = 0; i < key.Length; i++)
            key[i] = (byte)(255 - i);
        const ulong timestamp = 0x0000123456789AUL;
        int error;
        var frame = MpFrameEncoder.Encode(0, Heartbeat, 0x80000005, 1, 255, 9, false, key, 7, timestamp, out error);
        Check(frame != null && error == 0, "MpFrameEncoder.Encode: " + error);
        if (frame == null)
            return;
        Check(frame[0] == 0xFD && frame[2] == 0x03 && frame[4] == 9, "signed, 32-bit source, seq");
        ulong ts = 0;
        for (int i = 5; i >= 0; i--)
            ts = (ts << 8) | frame[frame.Length - 12 + i];
        Check(ts == timestamp && frame[frame.Length - 13] == 7, "timestamp and link id");
        using (var gcs = new MpLink())
        {
            MpFrame got = null;
            gcs.FrameReceived += f => got = f;
            gcs.AddSigningKey(key);
            gcs.Feed(frame, 0, frame.Length, 0);
            Check(got != null && (got.Flags & MpCoreNative.FrameFlags.SignatureOk) != 0 && got.SysId == 0x80000005,
                "the receiving link verifies it");
        }

        Check(MpFrameEncoder.Encode(0xFFFFFF, null, 1, 1, 0, 0, false, null, 0, 0, out error) == null &&
              error == MpCoreNative.MP_ERR_UNKNOWN_MSG, "unknown message");
        Check(MpFrameEncoder.Encode(0, Heartbeat, 0x80000005, 1, 0, 0, true, null, 0, 0, out error) == null &&
              error == MpCoreNative.MP_ERR_UNREPRESENTABLE, "no 32-bit source in MAVLink 1");
    }

    // the parameter download through the binding: mp_param's fields arrive where they should
    static byte[] ParamValue(ushort index, ushort count, string name, float value)
    {
        var payload = new byte[25];
        BitConverter.GetBytes(value).CopyTo(payload, 0);
        BitConverter.GetBytes(count).CopyTo(payload, 4);
        BitConverter.GetBytes(index).CopyTo(payload, 6);
        System.Text.Encoding.ASCII.GetBytes(name).CopyTo(payload, 8);
        payload[24] = 9;
        int error;
        return MpFrameEncoder.Encode(22, payload, 1, 1, 0, (byte)index, false, null, 0, 0, out error);
    }

    static void Params()
    {
        using (var download = new MpParamDownload(1, 1))
        {
            var sent = new List<uint>();
            var got = new List<MpParam>();
            download.Send += (msgid, payload) => sent.Add(msgid);
            download.Param += got.Add;
            download.Start();
            Check(sent.Count == 1 && sent[0] == 21, "PARAM_REQUEST_LIST sent");
            download.Frame(ParamValue(0, 3, "ARMING_CHECK", 1));
            download.Frame(ParamValue(2, 3, "ABCDEFGHIJKLMNOP", -2.5f));
            download.Frame(ParamValue(2, 3, "ABCDEFGHIJKLMNOP", -2.5f));
            // the last index came with 1 missing: it is asked for at once
            Check(!download.Tick() && sent.Count == 2 && sent[1] == 20, "PARAM_REQUEST_READ for the gap");
            download.Frame(ParamValue(1, 3, "SERIAL0_BAUD", 115));
            Check(download.Tick() && download.Done, "done");
            Check(got.Count == 4 && got[2].Duplicate && !got[1].Duplicate, "duplicates flagged");
            var p = got[1];
            Check(p.Name == "ABCDEFGHIJKLMNOP" && p.Value == -2.5f && p.Type == 9 && p.Index == 2 && p.Count == 3,
                "mp_param fields: " + p.Name + " " + p.Value + " " + p.Type + " " + p.Index + " " + p.Count);
            var status = download.Status;
            Check(status.received == 3 && status.total == 3 && status.full_list_requests == 1 && status.single_requests == 1 &&
                  status.done == 1 && status.one_by_one == 1, "status");
        }
    }

    static void Exchange()
    {
        var sent = new List<uint>();
        using (var set = MpExchange.ParamSet(1, 1, "ARMING_CHECK", 0, 9, (msgid, payload) => sent.Add(msgid)))
        {
            set.Start();
            Check(sent.Count == 1 && sent[0] == 23, "PARAM_SET sent");
            set.Frame(ParamValue(5, 300, "ARMING_RUDDER", 2));
            Check(set.Tick() == MpExchange.Result.Waiting && set.Reply == null, "another parameter is not the answer");
            var answer = ParamValue(4, 300, "ARMING_CHECK", 0);
            set.Frame(answer);
            Check(set.Tick() == MpExchange.Result.Done && set.Sends == 1, "done");
            var reply = set.Reply;
            Check(reply != null && reply.Length == answer.Length && reply[reply.Length - 1] == answer[answer.Length - 1], "reply frame");
        }

        using (var read = MpExchange.ParamRead(1, 1, "RTL_ALT", -1, (msgid, payload) => sent.Add(msgid)))
        {
            read.Start();
            Check(sent[1] == 20, "PARAM_REQUEST_READ sent");
            read.Frame(ParamValue(9, 300, "RTL_ALT", 1500));
            Check(read.Tick() == MpExchange.Result.Done, "read by name");
        }
    }

    static byte[] Encode(uint msgid, byte[] payload, byte seq = 0)
    {
        int error;
        return MpFrameEncoder.Encode(msgid, payload, 1, 1, 0, seq, false, null, 0, 0, out error);
    }

    // the mission download through the binding: count, items, ACK
    static void Missions()
    {
        var sent = new List<uint>();
        using (var download = new MpMissionDownload(1, 1, 255, 190, 0, true, (msgid, payload) => sent.Add(msgid)))
        {
            download.Start();
            Check(sent.Count == 1 && sent[0] == 43 && download.Count == -1, "MISSION_REQUEST_LIST");
            download.Frame(Encode(44, new byte[] { 2, 0, 255, 190, 0 }));
            Check(download.Count == 2 && sent[1] == 51, "count, then MISSION_REQUEST_INT 0");
            for (byte seq = 0; seq < 2; seq++)
            {
                var item = new byte[38];
                item[28] = seq;
                item[30] = 16; // NAV_WAYPOINT
                item[32] = 255;
                item[33] = 190;
                download.Frame(Encode(73, item, seq));
            }

            Check(download.Tick() == MpMissionDownload.Result.Done && download.Received == 2, "done");
            Check(sent[sent.Count - 1] == 47, "MISSION_ACK last");
            var items = download.Items;
            Check(items.Count == 2 && items[1][0] == 0xFD && items[1][7] == 73, "the item frames");
        }
    }

    // the mission upload through the binding: count, the item asked for, the ACK's result
    static void Upload()
    {
        var sent = new List<uint>();
        var payloads = new List<byte[]>();
        using (var upload = new MpMissionUpload(1, 1, 255, 190, 0, (msgid, payload) => { sent.Add(msgid); payloads.Add(payload); }))
        {
            for (byte seq = 0; seq < 2; seq++)
            {
                var item = new byte[38];
                item[28] = seq;
                upload.Add(73, item);
            }

            upload.Start();
            Check(sent.Count == 1 && sent[0] == 44 && payloads[0][0] == 2, "MISSION_COUNT 2");
            upload.Frame(Encode(51, new byte[] { 1, 0, 255, 190, 0 }));
            Check(sent.Count == 2 && sent[1] == 73 && payloads[1][28] == 1, "item 1 when asked for");
            Check(upload.CountAccepted && upload.HighestRequested == 1, "count taken");
            upload.Frame(Encode(47, new byte[] { 255, 190, 4, 0 }));
            Check(upload.Tick() == MpMissionDownload.Result.Done && upload.Result == 4, "the ACK's result (NO_SPACE)");
        }
    }

    // an FTP reply to the last request: opcode 128 ACK / 129 NAK
    static byte[] FtpReply(byte[] request, byte opcode, int seqAdd, uint offset, byte[] data, bool burstComplete = false)
    {
        var p = new byte[254];
        p[1] = 255;
        p[2] = 190;
        var seq = (ushort)(request[3] | (request[4] << 8));
        seq = (ushort)(seq + seqAdd);
        p[3] = (byte)seq;
        p[4] = (byte)(seq >> 8);
        p[6] = opcode;
        p[7] = (byte)data.Length;
        p[8] = request[6];
        p[9] = (byte)(burstComplete ? 1 : 0);
        BitConverter.GetBytes(offset).CopyTo(p, 11);
        data.CopyTo(p, 15);
        return Encode(110, p);
    }

    // MAVLink FTP through the binding: a download in two packets and a listing
    static void Ftp()
    {
        byte[] last = null;
        using (var ftp = new MpFtp(1, 1, 255, 190, (msgid, payload) => { if (msgid == 110) last = payload; }))
        {
            var file = new byte[150];
            for (int i = 0; i < file.Length; i++)
                file[i] = (byte)(i * 3);
            Check(ftp.GetFile("/f", 100, true) && last[6] == MpCoreNative.MP_FTP_RESET_SESSIONS, "reset first");
            ftp.Frame(FtpReply(last, 128, 1, 0, new byte[0]));
            Check(last[6] == 4 && System.Text.Encoding.ASCII.GetString(last, 15, 2) == "/f", "open");
            ftp.Frame(FtpReply(last, 128, 1, 0, BitConverter.GetBytes(150)));
            Check(ftp.FileSize == 150 && last[6] == 15, "burst");
            var first = new byte[100];
            Array.Copy(file, first, 100);
            var second = new byte[50];
            Array.Copy(file, 100, second, 0, 50);
            ftp.Frame(FtpReply(last, 128, 1, 0, first));
            ftp.Frame(FtpReply(last, 128, 2, 100, second, true));
            Check(ftp.Tick() == MpFtp.Result.Done && ftp.Progress == 150, "downloaded");
            var data = ftp.Data();
            bool same = data.Length == file.Length;
            for (int i = 0; same && i < data.Length; i++)
                same = data[i] == file[i];
            Check(same, "the file");

            Check(ftp.List("/"), "list");
            var listing = System.Text.Encoding.UTF8.GetBytes("Fкарта.txt\t12\t1600000000\0Dsub\0");
            ftp.Frame(FtpReply(last, 128, 1, 0, listing));
            ftp.Frame(FtpReply(last, 129, 1, 0, new byte[] { 6 }));
            var entries = ftp.Entries();
            Check(ftp.Tick() == MpFtp.Result.Done && entries.Length == 2, "two entries");
            Check(entries[0].Kind == 'F' && entries[0].Name == "карта.txt" && entries[0].Size == 12 &&
                  entries[0].MTime == 1600000000u, "a file, UTF-8 name");
            Check(entries[1].Kind == 'D' && entries[1].Name == "sub", "a directory");

            Check(ftp.Command(MpCoreNative.MP_FTP_REMOVE_FILE, "/x"), "remove");
            ftp.Frame(FtpReply(last, 129, 1, 0, new byte[] { 2, 2 }));
            Check(ftp.Tick() == MpFtp.Result.Failed && ftp.Nak == 2 && ftp.NakErrno == 2 &&
                  ftp.NakOpcode == MpCoreNative.MP_FTP_REMOVE_FILE, "errno NAK");
            ftp.NextSeq = 1000;
            Check(ftp.NextSeq == 1000, "seq shared");
        }
    }

    static int Main()
    {
        Layouts();
        Encoder();
        Params();
        Exchange();
        Missions();
        Upload();
        Ftp();
        RoundTrip();
        Signing();
        Console.WriteLine((failures == 0 ? "ok" : "FAIL") + " (" + failures + " failed checks, " + IntPtr.Size * 8 + "-bit)");
        return failures == 0 ? 0 : 1;
    }
}
