using System;
using System.Collections.Generic;
using System.IO;
using MissionPlanner.Comms;

namespace MissionPlannerTests.Mavlink
{
    /// <summary>
    /// A port whose bytes are all there at once and are read as much as asked (unlike Sysid32Tests'
    /// one byte at a time), so the C++ core reads in chunks as on a real port. Writes are kept, and
    /// OnWrite can answer them (a simulated vehicle).
    /// </summary>
    internal sealed class MpCoreTestSerial : ICommsSerial
    {
        readonly Queue<byte> incoming = new Queue<byte>();
        public readonly MemoryStream Written = new MemoryStream();
        public Action<byte[]> OnWrite;
        public void Feed(byte[] bytes) { lock (incoming) foreach (byte b in bytes) incoming.Enqueue(b); }
        public Stream BaseStream => Written;
        public int BaudRate { get; set; }
        public int BytesToRead { get { lock (incoming) return incoming.Count; } }
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
        public void DiscardInBuffer() { lock (incoming) incoming.Clear(); }
        public int Read(byte[] buffer, int offset, int count)
        {
            lock (incoming)
            {
                int n = Math.Min(count, incoming.Count);
                for (int i = 0; i < n; i++) buffer[offset + i] = incoming.Dequeue();
                return n;
            }
        }
        public int ReadByte() { lock (incoming) return incoming.Dequeue(); }
        public int ReadChar() { return ReadByte(); }
        public string ReadExisting() { return ""; }
        public string ReadLine() { return ""; }
        public void Write(string text) { }
        public void Write(byte[] buffer, int offset, int count)
        {
            Written.Write(buffer, offset, count);
            if (OnWrite != null)
            {
                var frame = new byte[count];
                Array.Copy(buffer, offset, frame, 0, count);
                OnWrite(frame);
            }
        }
        public void WriteLine(string text) { }
        public void toggleDTR() { }
        public void Dispose() { Close(); }
    }
}
