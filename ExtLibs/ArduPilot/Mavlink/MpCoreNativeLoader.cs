using System;
using System.IO;
using System.Runtime.InteropServices;

namespace MissionPlanner.ArduPilot.Mavlink
{
    /// <summary>Finds the mpcore native library for DllImport("mpcore").</summary>
    public static class MpCoreNativeLoader
    {
        [DllImport("kernel32", CharSet = CharSet.Unicode, SetLastError = true)]
        static extern IntPtr LoadLibraryW(string path);

        static readonly object Sync = new object();
        static bool _done;

        /// <summary>
        /// Mission Planner is AnyCPU: on Windows load the DLL matching the process from x86/ or x64/
        /// next to this assembly (the layout Utilities/NativeLibrary.cs uses), so DllImport("mpcore")
        /// finds it already loaded. Elsewhere Mono looks next to the assembly itself.
        /// </summary>
        public static void Load()
        {
            lock (Sync)
            {
                if (_done)
                    return;
                _done = true;
                if (!RuntimeInformation.IsOSPlatform(OSPlatform.Windows))
                    return;
                var dir = Path.GetDirectoryName(typeof(MpCoreNativeLoader).Assembly.Location) ?? "";
                var path = Path.Combine(dir, Environment.Is64BitProcess ? "x64" : "x86", "mpcore.dll");
                if (File.Exists(path))
                    LoadLibraryW(path);
            }
        }
    }
}
