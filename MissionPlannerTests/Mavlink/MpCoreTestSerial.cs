using System;
using System.Collections.Generic;
using System.IO;
using MissionPlanner.Comms;

namespace MissionPlannerTests.Mavlink
{
    /// <summary>
    /// A port whose bytes are all there at once and are read as much as asked (unlike Sysid32Tests'
    /// one byte at a time), so the C++ core reads in chunks as on a real port. Writes are kept.
    /// </summary>
    internal sealed class MpCoreTestSerial : ICommsSerial
    {
        readonly Queue<byte> incoming = new Queue<byte>();
        public readonly MemoryStream Written = new MemoryStream();
        public void Feed(byte[] bytes) { foreach (byte b in bytes) incoming.Enqueue(b); }
        public Stream BaseStream => Written;
        public int BaudRate { get; set; }
        public int BytesToRead => incoming.Count;
        public int BytesToWrite => 0;
        public int DataBits { get; set; }
        public bool DtrEnable { get; set; }
        public bool IsOpen { get; private set; } = true;
        public string PortName { get; set; } = "test";
        public int ReadBufferSize { get; set; }
        public int ReadTimeout { get; set; } = 100;
        public bool RtsEnable { get; set; }
        public int WriteBufferSize { get; set; }
        public int WriteTimeout { get; set; }
        public void Close() { IsOpen = false; }
        public void Open() { IsOpen = true; }
        public void DiscardInBuffer() { incoming.Clear(); }
        public int Read(byte[] buffer, int offset, int count)
        {
            int n = Math.Min(count, incoming.Count);
            for (int i = 0; i < n; i++) buffer[offset + i] = incoming.Dequeue();
            return n;
        }
        public int ReadByte() { return incoming.Dequeue(); }
        public int ReadChar() { return ReadByte(); }
        public string ReadExisting() { return ""; }
        public string ReadLine() { return ""; }
        public void Write(string text) { }
        public void Write(byte[] buffer, int offset, int count) { Written.Write(buffer, offset, count); }
        public void WriteLine(string text) { }
        public void toggleDTR() { }
        public void Dispose() { Close(); }
    }
}
