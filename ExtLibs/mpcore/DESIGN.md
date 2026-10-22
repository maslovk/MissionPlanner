# mpcore design

## Goal and path

Mission Planner moves from C# to C/C++. The new application is Qt/C++ built with CMake, but it
is grown core first:

1. **Core (this library).** The MAVLink protocol layer in C++ with no UI and no Qt, exposed
   through a C ABI. It replaces the internals of `MAVLinkInterface` inside the shipping C#
   application, so every user connection tests it.
2. **Vehicle model.** `CurrentState` (about 270 telemetry properties) and the sub-protocols
   (parameters, missions, MAVFTP, logs) move into the core.
3. **Qt application.** A Qt UI on the same library, screen by screen: Flight Data, Flight
   Plan, setup, then the long tail. The C# application is frozen when the Qt one covers it.

The core does not depend on Qt so that it can be loaded into the C# process (no Qt runtime to
ship with Mission Planner) and so that a Qt layer later only wraps it in `QObject`s.

## Decisions

**No I/O in the core.** A link is fed received bytes (`mp_link_feed`) and hands bytes to send to
a callback. Transports stay with the caller: Mission Planner assigns `BaseStream` in about 30
places (serial, TCP, UDP, WebSocket, BLE, plugins) and reads `BaseStream.IsOpen` 133 times, so
replacing the transports first would touch everything at once. The Qt application will drive
the same API from `QSerialPort`/`QUdpSocket`. It also makes the core deterministic: tests feed
exact bytes and times.

**Single-threaded, caller-serialised.** A link is not thread-safe; callbacks run synchronously
inside `feed`/`send`. Mission Planner today has one reader (`MainV2.SerialReader`) plus
request/response operations that "take the port" (`giveComport`) and pump `readPacketAsync`
themselves. In the core those operations become state machines driven by received frames, so
there is only ever one reader and `giveComport` goes away (phase 2).

**Upstream MAVLink C code, pinned.** Headers are generated at build time by upstream pymavlink
(submodule at `bd0a1b29`, 2026-10-16) from Mission Planner's own XMLs. Upstream already supports
the 32-bit system IDs Mission Planner added (`SYSID32`/`TARGET32`, header lengths 10/13/14/17),
so its parser (`mavlink_frame_char_buffer`, with caller-owned buffers, no globals) and encoder
(`mavlink_finalize_message_buffer_target`) are used as they are. The generated headers are not
committed (11 MB).

**C ABI rules.** Opaque handle; every struct starts with `struct_size` and only grows at the end
(older callers keep working; the library never reads or writes past the caller's size);
`__cdecl` on Windows; no exceptions cross the boundary; only `mp_*` symbols are exported.
Pointers given to callbacks are valid during the callback only.

## What matches Mission Planner (MAVLinkInterface.readPacketAsync / generatePacket)

| Behaviour | Mission Planner (C#) | mpcore |
|---|---|---|
| Framing, 32-bit IDs | v1, v2, SYSID32, TARGET32; unknown incompat flags rejected | same (upstream parser) |
| Unknown msgid | CRC unverifiable; forwarded to mirrors | delivered flagged `MP_FRAME_UNKNOWN_MSGID`, no peer state |
| Bytes outside frames | printable chars collected into lines, lines longer than 3 kept | same (`on_text`) |
| Signed frames | try the peer key, then every key in `MAVAuthKeys`; a match enables signing to that peer; unsigned frames always accepted; no replay check | same |
| Target filter | only frames with a TARGET32 header not addressed to us are dropped (after tlog) | delivered flagged `MP_FRAME_NOT_FOR_US` (so they can still be logged), not counted |
| Packet loss | 8-bit seq, 64-entry mask of received seqs, late packets give loss back if the previous packet was < 1 s ago, decay ×0.8 every 5 s, float counters | same algorithm (`SeqTracker`), including the fix in `cafd00845` |
| New peer | first packet counted as a duplicate of itself | same |
| MAVLink 1 vs 2 on send | v1 until the peer has sent v2, unless msgid > 255 or an ID > 255; v1 payload cut to the base length | same |
| Wide target on send | payload `target_system` = 255, full ID in TARGET32, only for messages with a target field | same |
| Signing on send | per peer key and link id; timestamp in 10 µs since 2015, strictly increasing | same |

## Deliberate differences

* **Resync after a failed frame.** Mission Planner looks for a new start byte only in the last
  20 bytes of a frame that failed its CRC, and drops a frame with unknown incompat flags whole.
  mpcore scans every byte after the failed frame's start again. A corrupted length byte then
  cannot swallow the frames that follow (tested with 500 frames in random noise). One visible
  consequence: a valid frame embedded in the payload of a rejected frame is found
  (`rejects_unknown_incompat_flags_and_recovers`). Failures that start inside rescanned bytes are
  not counted again, and an unknown-msgid "frame" found there is not delivered, since its CRC
  cannot be checked.
* **Stalled frame starts are rescanned, not dropped.** A false start in line noise (for example
  `FE FE`, a MAVLink 1 frame claiming 254 bytes) waits for bytes that never come. Mission Planner
  times out after 1.2 s and drops what it read; mpcore gives up after the same 1.2 s
  (`mp_link_poll`, or the next `feed`) and scans those bytes again, so the real frames inside are
  delivered.
* **Signatures are verified only by mpcore**, not by the generated parser (`MAVLINK_NO_SIGNATURE_CHECK`),
  so the key ring behaviour above is kept.

## Found while surveying the C# code (not ported)

These are recorded so the port does not copy them; they need their own fixes in C# if it keeps
running meanwhile.

* `getRallyPoint` re-requests with the `FENCE_FETCH_POINT` msgid (`MAVLinkInterface.cs` ~6354, 6378).
* `setRallyPoint` verifies with `newfp.plla.Lng == rp.lng`, degrees against degrees×1e7, so it never matches.
* `saveWPsFast` checks sysid/compid with `&&` where `||` is meant (`FlightPlanner.cs` ~6351).
* `uploadPartial` resumes INVALID_SEQUENCE at an absolute seq in a relative index (`mav_mission.cs` ~228).
* `setWPTotalAsync` writes WP_TOTAL/CMD_TOTAL/MIS_TOTAL on the current MAV, not the target.
* The `setWPAsync` MISSION_ITEM overload stores the MISSION_REQUEST_INT reply only into `wps`, whatever the mission type.
* `getParamListAsync` reads packets without `giveComport`, competing with the main reader.
* `giveComport`'s "wait for the current read" locks the `SemaphoreSlim` object as a monitor, which never waits for a read.
* Signed frames are not checked for replay (the timestamp is computed and ignored). mpcore keeps
  this for now for compatibility; it should become an option once signing is driven by the core.

## Phase 1: running inside Mission Planner (next)

1. **Ship the libraries.** Build `mpcore.dll` for x86 and x64 (and `libmpcore.so` for the Linux
   package) in CI and place them as `<appdir>/<arch>/`, the layout `Utilities/NativeLibrary.cs`
   already uses for SkiaSharp; preload them in `Program.cs` the same way so `DllImport("mpcore")`
   resolves. Add `bindings/csharp/MpCore.cs` to `MissionPlanner.ArduPilot.csproj` (it needs
   `AllowUnsafeBlocks`, and stays out of the `LIB` build until Android ships the `.so`).
2. **Shadow mode.** Behind a setting (off by default), `MAVLinkInterface.readPacketAsync` also
   feeds every byte it reads into an `MpLink` and compares: each frame the C# parser returns must
   come out of the core with the same bytes, and the per-MAV loss counters must agree. Differences
   are logged. This runs on real links with no change in behaviour.
3. **Switch the receive path.** With the setting on, `readPacketAsync` reads what is available
   from `BaseStream`, feeds it to the core and returns frames from a queue filled by the frame
   callback, one per call. Its contract (one `MAVLinkMessage` per call) stays the same for all 27
   callers. `MAVState.packetslost`, `packetsnotlost`, `synclost`, `mavlinkv2` and the signing state
   come from `mp_peer_stats`. `processInfoFromStream`, subscriptions and `OnPacketReceived` stay in
   C# for now.
4. **Switch the send path.** `generatePacket` calls `mp_link_send`; the write callback writes to
   `BaseStream` under `writelock` and to the tlog.
5. **Then phase 2:** parameter, mission (all three types) and MAVFTP transfers move into the core
   as state machines with progress and cancellation, replacing the `giveComport` read loops; then
   `CurrentState`.

## Tests

* `tests/wire.hpp` builds frames byte by byte with its own CRC, ported from
  `MissionPlannerTests/Mavlink/Sysid32Tests.cs`, so both implementations are checked against the
  same fixtures (all eight flag combinations, boundary IDs, signatures).
* Parser: fragmentation, rejection and recovery, corruption, truncation, maximum lengths,
  unknown msgids, text lines, random noise.
* Sequence tracking: each rule above, including the fast-link wrap case.
* Link: MAVLink 1/2 negotiation, 32-bit IDs and the target filter, signing with a key ring,
  bad signatures, playback, timestamps, re-entrant handlers, stalls, statistics.
* `test_capi.c`: the ABI from plain C, including an older caller's smaller structs.
* `bindings/csharp/MpCoreSmoke.cs`: the C# binding against the shared library (struct layouts
  checked against the native sizes, both bitnesses in CI).
* Everything also runs under AddressSanitizer and UBSan (`linux-asan`).
