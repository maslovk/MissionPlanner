# mpcore

The MAVLink core of Mission Planner in C++17, with a C ABI. It is the first step of moving
Mission Planner from C# to C/C++: the protocol layer is rebuilt here and runs inside today's
C# application through P/Invoke, so it can be proven against real vehicles before a Qt UI is
built on the same library. See [DESIGN.md](DESIGN.md).

Unlike the rest of the solution it builds on Linux, macOS and Windows with plain CMake.

## Build and test

Needs CMake 3.21+, a C++17 compiler, Python 3.8+ (to generate the MAVLink headers) and the
pymavlink submodule:

```sh
git submodule update --init ExtLibs/mpcore/third_party/pymavlink
cmake --preset linux-debug -S ExtLibs/mpcore        # or linux-release, linux-asan
cmake --build ExtLibs/mpcore/build/linux-debug
ctest --test-dir ExtLibs/mpcore/build/linux-debug
```

On Windows use the `windows-x64` or `windows-x86` preset (the newest Visual Studio installed) and add
`--config Release` / `-C Release`. Mission Planner is AnyCPU, so both DLLs are needed.

To also test the C# binding and the shadow comparer, pass a C# compiler:
`-DMPCORE_CSHARP_COMPILER=$(command -v mcs)` (Mono, with `libmono-system-runtime-serialization4.0-cil`;
an `mcs.exe` is run with `mono`), or Roslyn's `csc.exe` on Windows. If the compiler cannot find
`System.Runtime.Serialization.dll` by name, give its path in `MPCORE_CSHARP_SERIALIZATION_REF`.

Inside Mission Planner the core runs behind three settings, all off by default: `mpcore_shadow`
(beside the C# parser, logging differences), `mpcore_receive` (instead of the C# byte loop in
`readPacketAsync`) and `mpcore_send` (encoding the frames `generatePacket` sends); see DESIGN.md.

`mpcore_tests [name]` runs only the C++ cases whose name contains `name`.

## Layout

| Path | |
|---|---|
| `include/mpcore/mpcore.h` | the C ABI: the contract for C# now and anything else later |
| `src/protocol.*` | message table, frame encoding, signatures |
| `src/frame_parser.*` | byte stream to frames, resync, text lines |
| `src/seq_tracker.*` | packet loss from sequence numbers |
| `src/link.*` | one MAVLink stream: peers, signing, targeting, send |
| `src/capi.cpp` | the exported C functions |
| `bindings/csharp/` | P/Invoke declarations and an `MpLink` wrapper (linked into the ArduPilot project), a smoke test, the shadow parity test |
| `tests/` | C++ unit tests, a C ABI test, independent wire fixtures |
| `cmake/MavlinkHeaders.cmake` | generates the MAVLink C headers at build time |
| `third_party/pymavlink` | upstream pymavlink (submodule, pinned) for the C generator |

The MAVLink headers are generated from `ExtLibs/Mavlink/message_definitions/all.xml`, the same
definitions `Mavlink.cs` comes from, into the build tree (about 11 MB, not committed).
