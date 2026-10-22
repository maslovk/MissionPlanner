using System;
using System.Collections.Generic;
using MissionPlanner.Mavlink.Native;

// Every message of Mission Planner's C# dialect, encoded by the C# encoder (MavlinkParse) and by
// the core (mp_frame_encode), must give the same bytes: MAVLink 1 and 2, signed, 8 and 32-bit
// source and destination. Exit code 0 on success.
static class MpCoreEncoderParity
{
    static int failures;
    static int details;

    static void Fail(string what)
    {
        failures++;
        if (details++ < 30)
            Console.WriteLine("    " + what);
    }

    static bool Same(byte[] a, byte[] b)
    {
        if (a == null || b == null || a.Length != b.Length)
            return false;
        for (int i = 0; i < a.Length; i++)
            if (a[i] != b[i])
                return false;
        return true;
    }

    static string Hex(byte[] b)
    {
        return b == null ? "null" : BitConverter.ToString(b, 0, Math.Min(b.Length, 24));
    }

    // MavlinkParse.GenerateMAVLinkPacket10 also puts MAVLink 2 extension fields into MAVLink 1 frames;
    // MAVLinkInterface.generatePacket, which is what sends, cuts the payload to the base length.
    // Build that frame from the C# one: its header, the cut payload, Mission Planner's CRC.
    static byte[] V1AsSent(byte[] frame, MAVLink.message_info info)
    {
        int length = (int)info.minlength;
        var cut = new byte[6 + length + 2];
        Array.Copy(frame, cut, 6);
        Array.Copy(frame, 6, cut, 6, Math.Min(length, frame[1]));
        cut[1] = (byte)length;
        ushort crc = MAVLink.MavlinkCRC.crc_calculate(cut, length + 6);
        crc = MAVLink.MavlinkCRC.crc_accumulate(info.crc, crc);
        cut[6 + length] = (byte)crc;
        cut[7 + length] = (byte)(crc >> 8);
        return cut;
    }

    static int Main()
    {
        var rnd = new Random(5);
        var key = new byte[32];
        rnd.NextBytes(key);
        var skipped = new List<string>();
        int messages = 0, frames = 0;

        foreach (var info in MAVLink.MAVLINK_MESSAGE_INFOS)
        {
            var native = new MpCoreNative.mp_msg_info { struct_size = (uint)System.Runtime.InteropServices.Marshal.SizeOf(typeof(MpCoreNative.mp_msg_info)) };
            if (MpCoreNative.mp_msg_info_get(info.msgid, ref native) != MpCoreNative.MP_OK)
            {
                skipped.Add(info.name + " (" + info.msgid + ")");
                continue;
            }

            messages++;
            if (native.crc_extra != info.crc || native.min_len != info.minlength || native.max_len != info.length)
                Fail(string.Format("{0}: C# crc {1} len {2}/{3}, core crc {4} len {5}/{6}", info.name, info.crc,
                    info.minlength, info.length, native.crc_extra, native.min_len, native.max_len));

            int size = System.Runtime.InteropServices.Marshal.SizeOf(info.type);
            bool hasTarget = MAVLink.GetTargetSystemField(info.type) != null;
            var targets = hasTarget ? new uint?[] { null, 0x90000001 } : new uint?[] { null };
            for (int round = 0; round < 3; round++)
            {
                var bytes = new byte[size];
                rnd.NextBytes(bytes);
                if (round == 2)
                    Array.Clear(bytes, size / 2, size - size / 2); // a zero tail, trimmed in MAVLink 2
                var obj = MavlinkUtil.ByteArrayToStructureGC(bytes, info.type, 0, (byte)size);
                byte seq = (byte)rnd.Next(256);

                if (info.msgid < 256)
                {
                    var parser = new MAVLink.MavlinkParse();
                    var cs = V1AsSent(parser.GenerateMAVLinkPacket10((MAVLink.MAVLINK_MSG_ID)info.msgid, obj, 1, 1, seq), info);
                    var core = MpFrameEncoder.Encode(info.msgid, MavlinkUtil.StructureToByteArray(obj), 1, 1, 0, seq, true,
                        null, 0, 0, out int error);
                    frames++;
                    if (!Same(cs, core))
                        Fail(info.name + " v1: C# " + Hex(cs) + " core " + Hex(core) + " " + error);
                }

                foreach (uint source in new uint[] { 1, 0x80000002 })
                foreach (uint? target in targets)
                foreach (bool sign in new[] { false, true })
                {
                    var parser = new MAVLink.MavlinkParse { signingKey = key, sendlinkid = 3 };
                    // the C# encoder writes the target into its copy of the payload; give both the same struct
                    var copy = MavlinkUtil.ByteArrayToStructureGC(MavlinkUtil.StructureToByteArray(obj), info.type, 0, (byte)size);
                    var cs = parser.GenerateMAVLinkPacket20((MAVLink.MAVLINK_MSG_ID)info.msgid, copy, sign, source, 1, seq,
                        target, target.HasValue ? (byte)1 : (byte)0);
                    ulong timestamp = 0;
                    if (sign)
                    {
                        // the C# encoder takes the time itself: sign the core's frame at the same time
                        for (int i = 5; i >= 0; i--)
                            timestamp = (timestamp << 8) | cs[cs.Length - 12 + i];
                    }

                    var payload = MavlinkUtil.StructureToByteArray(obj);
                    if (target.HasValue)
                    {
                        // what the C# encoder's SetPayloadTarget does for a target component
                        var field = info.type.GetField("target_component");
                        if (field != null)
                            payload[(int)System.Runtime.InteropServices.Marshal.OffsetOf(info.type, field.Name)] = 1;
                    }

                    var core = MpFrameEncoder.Encode(info.msgid, payload, source, 1, target ?? 0, seq, false,
                        sign ? key : null, 3, timestamp, out int error);
                    frames++;
                    if (!Same(cs, core))
                        Fail(string.Format("{0} v2 source {1} target {2} signed {3}: C# {4} core {5} {6}", info.name, source,
                            target, sign, Hex(cs), Hex(core), error));
                }
            }
        }

        Console.WriteLine(messages + " messages, " + frames + " frames compared");
        if (skipped.Count > 0)
            Console.WriteLine("not in the core's dialect (all.xml): " + string.Join(", ", skipped));
        Console.WriteLine(failures == 0 ? "ok" : "FAIL (" + failures + " differences)");
        return failures == 0 ? 0 : 1;
    }
}
