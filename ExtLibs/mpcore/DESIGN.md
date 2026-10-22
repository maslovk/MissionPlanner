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
* The `setWPAsync` MISSION_ITEM_INT overload takes MISSION_REQUEST but not MISSION_REQUEST_INT as the next
  request, so a vehicle asking with MISSION_REQUEST_INT (PX4) times out after the first item; ArduPilot asks
  with MISSION_REQUEST.
* `getParamListAsync` reads packets without `giveComport`, competing with the main reader.
* `giveComport`'s "wait for the current read" locks the `SemaphoreSlim` object as a monitor, which never waits for a read.
* MAVFtp takes any NAK as the answer to its request (no opcode or seq check), answers NoSessionsAvailable
  with a blocking `kCmdResetSessions` inside its packet handler (on the reader thread, which then cannot
  deliver the reply), sends ResetSessions, TerminateSession and ReadFile without setting the target, and
  `kCmdCreateDirectory` throws on errno EEXIST although it marks the request complete.
* `kCmdWriteFile(Stream)` finishes at the last chunk's ACK even when an earlier chunk was lost, and
  `kCmdBurstReadFile` returns the partial file on a timeout; its `FindMissing` never reports a missing tail.
* Signed frames are not checked for replay (the timestamp is computed and ignored). mpcore keeps
  this for now for compatibility; it should become an option once signing is driven by the core.

## Phase 1: running inside Mission Planner

1. **Ship the libraries (done).** `main.yml` builds `mpcore.dll` for x64 and x86 into
   `bin/Release/net461/<arch>/`, the layout `Utilities/NativeLibrary.cs` uses for SkiaSharp;
   `MpCoreShadow` preloads the one matching the process. `Linux/build.sh` puts `libmpcore.so` next
   to the assemblies, where Mono looks (when CMake and the pymavlink submodule are available).
   `bindings/csharp/MpCore.cs` is linked into `MissionPlanner.ArduPilot.csproj`; it needs no
   unsafe code. Without the native library the shadow logs "not available" and does nothing.
2. **Shadow mode (done, needs field time).** See below.
3. **Switch the receive path (done, opt-in, setting `mpcore_receive`).** See "Native receive".
4. **Switch the send path (done, opt-in, setting `mpcore_send`).** See "Native send". The
   per-vehicle state (MAVLink 1/2, signing keys and timestamps, loss counters) stays in `MAVState`
   for now: the core computes the same (shadow mode and the receive tests check it), and it moves
   into the core with the protocols in phase 2, when C# stops needing it, rather than being kept
   in two places meanwhile.
5. **Then phase 2** (below).

## Phase 2: the protocols

The sub-protocols move into the core as state machines with the same shape as `mp_link`: no I/O,
frames in, messages out through a callback, `tick(now)` for timers. Mission Planner keeps sending
(through `generatePacket`, so one sequence counter and one signing state), feeds the frames its
subscriptions see, and keeps everything user-facing (progress, cancelling, errors, where results
go). Each comes behind its own setting, off by default, with a test that runs the C# and the core
version against the same simulated vehicle.

1. **Parameter download (done, setting `mpcore_params`).** `ParamDownload`
   (`src/param_download.*`, C ABI `mp_param_download_*`, C# `MpParamDownload`) does what
   `getParamListAsync` does: PARAM_REQUEST_LIST, a 4 s quiet timer, the whole list again while
   under 75 % (twice once the total is known), then up to 10 missing indexes a second, at once if
   the last index came with gaps, no overall timeout. `getParamListNativeAsync` keeps the rest:
   it still pumps `readPacketAsync` itself (the reader may be stopped while connecting), builds the
   `MAVLinkParamList` (float or bytewise per the vehicle's capabilities), `param_types`, progress
   and the "User Canceled" / "Not Connected" / "Missing Params" outcomes. Mission Planner's "skip a
   round while the list is still arriving" check never takes effect (it loops straight into the
   round); the core does the same, as the quiet timer already means the list has stopped.
   Tests: `tests/test_params.cpp` (a simulated vehicle: loss, silence, gaps, duplicates, 65535,
   other vehicles, no parameters, 32-bit IDs), the smoke test (the binding), and
   `MissionPlannerTests/Mavlink/MpCoreParamsTests.cs` (`getParamListAsync` both ways on a vehicle
   that loses a fifth of the first list: the same list, the same requests).
2. **Parameter set and single read (done, same setting).** `Exchange` (`src/exchange.*`, C ABI
   `mp_exchange_*` with `mp_param_set_create` / `mp_param_read_create`, C# `MpExchange`) is the
   request/reply loop most of `MAVLinkInterface` repeats: send, wait for the matching reply (700 ms),
   send again up to 3 times, then time out. `setParamAsync` and `GetParamAsync` keep their checks,
   build the request as before, and with `mpcore_params` wait through `NativeExchangeAsync`, which
   reads the link the same way the old loops do; the table updates on an answer are shared by both
   paths (`ApplySetParamReply`, `ApplyGetParamReply`). A fire-and-forget read
   (`requireresponce: false`) stays as it was. The same exchange will carry the other request loops
   (mission count and items, home position, commands). Tests: `tests/test_exchange.cpp`, the C ABI
   and smoke tests, and `MpCoreParamsTests.NativeParamSetAndReadBehaveTheSame` (a vehicle that
   ignores the first two sets, reads by name and index, and the timeouts, both ways).
3. **Mission download (done, setting `mpcore_missions`).** `MissionDownload`
   (`src/mission_download.*`, C ABI `mp_mission_download_*`, C# `MpMissionDownload`) runs what
   `mav_mission.download` does with `getWPCountAsync` and `getWPAsync`: MISSION_REQUEST_LIST (700 ms,
   6 resends), each item by MISSION_REQUEST_INT or, without the vehicle's MISSION_INT capability,
   MISSION_REQUEST (2.5 s, 5 resends, at once again for a stale seq), and the MISSION_ACK. Replies
   must be for this GCS (`IsTargetedTo`) and, unlike the C# loops, of the mission type asked for (a
   fence count can no longer be taken for the mission's), and a request asked again keeps its kind
   (the C# loop falls back to MISSION_REQUEST after a MISSION_ITEM). `MissionDownloadNativeAsync`
   turns the item frames into `Locationwp` with the same conversion as `getWPAsync`
   (`LocationFromItem`, now shared). Tests: `tests/test_mission.cpp`, the smoke test and
   `MissionPlannerTests/Mavlink/MpCoreMissionTests.cs` (both ways, with and without MISSION_INT, a
   lost count and a stale item: the same items, requests and ACK).
4. **Mission upload (done, same setting).** `MissionUpload` (`src/mission_upload.*`, C ABI
   `mp_mission_upload_*`, C# `MpMissionUpload`) lets the vehicle drive the upload as the protocol
   intends: MISSION_COUNT (700 ms, 3 resends, as `setWPTotalAsync`), then every MISSION_REQUEST_INT
   or MISSION_REQUEST answered with the item it names, in any order and as often as asked, until the
   vehicle's MISSION_ACK; the last thing sent goes again after 450 ms, up to 10 times (as
   `setWPAsync`). `mav_mission.upload` instead sends item by item and waits for the next request,
   so it needs MISSION_WRITE_PARTIAL_LIST and "which item do you want" recovery when a request or
   an ACK is lost; answering what is asked needs neither. `MissionUploadNativeAsync` builds the
   items as `setWPAsync` does (`MissionItem`, now shared), does `setWPTotalAsync`'s bookkeeping once
   the vehicle takes the count (`ApplyWPTotal`, shared, still on the current MAV), keeps the items
   in `wps`/`fencepoints`/`rallypoints` once the upload is accepted (the C# loop keeps each as it
   goes), and `mav_mission.upload` turns the ACK into the same errors. Tests:
   `tests/test_mission.cpp`, the smoke test and `MpCoreMissionTests.NativeMissionUploadSendsTheSameItems`
   (both ways, with and without MISSION_INT, a lost count and a lost request: the same items on the
   vehicle and in `wps`, the same resends).
5. **MAVFTP (done, setting `mpcore_ftp`).** `Ftp` (`src/ftp.*`, C ABI `mp_ftp_*`, C# `MpFtp`) runs
   one MAVFtp operation at a time: `GetFile` (ResetSessions, OpenFileRO, BurstReadFile from 0, each
   burst continuing where the furthest data ended, then the holes by ReadFile; a short read or an
   EOF NAK ends the file), `UploadFile` (ResetSessions, CreateFile, 80-byte WriteFiles, the first
   alone and then up to 5 outstanding, ResetSessions), `kCmdListDirectory` (timed, falling back to
   plain for good when the vehicle NAKs it) and the single commands, with RetryTimeout's timings
   (ResetSessions 5 x 1 s, OpenFileRO 5 x 2 s, listings 5 x 1 s, CalcFileCRC32 3 x 30 s, the rest
   30 x 1 s, data restarting the count). Replies and NAKs must answer the request (its opcode and,
   but for a burst, its seq) and be for this GCS; the next seq follows the vehicle's latest reply,
   which keeps ArduPilot's "same seq again: send the last reply again" from firing on a new
   request. `MAVFtp` keeps its API: with the setting each of those methods runs the core on its
   own thread as the RetryTimeout loops do (the reader thread delivers the replies), shares
   `seq_no` and the plain-listing flag with it, and turns the outcome into the same exceptions,
   `false` or `null`. AP_REPL's own sessions (`kCmdOpenFileRO`/`WO`, `kCmdReadFile`, `kCmdWriteFile`
   at an offset) stay in C#. Differences: a download that times out returns `null` (MAVFtp's burst
   returns what it had), an upload ends once every chunk is acknowledged, a NAK that MAVFtp only
   logs ends the operation at once with the outcome MAVFtp reaches after its retries, and a
   missing tail is read. Tests: `tests/test_ftp.cpp` (a simulated ArduPilot FTP server: lost
   packets, bursts, holes, short files, sessions, timeouts, listings, commands), the C ABI and
   smoke tests, and `MissionPlannerTests/Mavlink/MpCoreFtpTests.cs` (both ways on one vehicle that
   loses a burst packet and a write ACK: the same file, upload, listing, CRC and errors).
6. **`CurrentState` (started: a shadow, setting `mpcore_state_shadow`).** `VehicleState`
   (`src/vehicle_state.*`, C ABI `mp_vehicle_*` with the flat `mp_vehicle_state`, C# `MpVehicle`)
   decodes what the HUD, map and status screens are built from as `Parent_OnPacketReceived` and
   the property setters do: HEARTBEAT, SYS_STATUS, EXTENDED_SYS_STATE, BATTERY2, BATTERY_STATUS
   (nine batteries, battery 0's voltage smoothed 0.4/0.6, its remaining clamped), ATTITUDE,
   GLOBAL_POSITION_INT and GPS_RAW_INT (the GPS's position only while GLOBAL_POSITION_INT has none,
   its speed and course only with a fix), GPS_STATUS, RADIO/RADIO_STATUS (from any system),
   MISSION_CURRENT, NAV_CONTROLLER_OUTPUT, RC_CHANNELS(_RAW), SERVO_OUTPUT_RAW (two ports), VFR_HUD
   (reversed throttle), VIBRATION, EKF_STATUS_REPORT (CurrentState's red rules), SCALED_PRESSURE and
   TERRAIN_REPORT. The values are the stored ones, before display units; the layout is ordered by
   field size so C and C# cannot pad it differently. `MpCoreStateShadow` decodes each message a
   CurrentState takes with the core too and, after each one the core decodes, compares the values
   that message sets with CurrentState's (backing fields where a property converts units) and logs
   the differences, like the frame shadow. What reads settings, parameters or the firmware's mode
   names (the mode string, messageHigh, speech, low airspeed) and what CurrentState derives from
   the wall clock (climb rate without VFR_HUD, mAh from SYS_STATUS current, time in the air) stays
   in C#. Next: the rest of the messages, then CurrentState reading the core's values for the
   decoded ones. Tests: `tests/test_vehicle_state.cpp`, the smoke test (layout and a decode) and
   `MissionPlannerTests/Mavlink/MpCoreStateShadowTests.cs` (flight.tlog through MAVLinkInterface and
   CurrentState with the shadow: no differences).

## Shadow mode

`ExtLibs/ArduPilot/Mavlink/MpCoreShadow.cs` runs the core beside `MAVLinkInterface`'s own parser
on a live link. It changes nothing the application does; it only logs.

**Turning it on.** With Mission Planner closed, add `<mpcore_shadow>True</mpcore_shadow>` to
`config.xml` (Windows: `Documents\Mission Planner\`; Linux: `~/.local/share/Mission Planner/`, or
`~/Documents/Mission Planner/` if that exists). It applies to each new connection.

**What it compares.**
* Bytes: the three `BaseStream.Read` calls in `readPacketAsync` feed exactly what they read to the
  core (the C# resync re-parses its own buffer and is not fed again). Log playback is not shadowed.
* Frames: each frame `readPacketAsync` accepts (CRC and signature checked, before the TARGET32
  filter) must come out of the core with the same bytes. Both see the same bytes in the same order,
  so frames are matched in order; a frame still unmatched after 2.5 s (longer than the core's 1.2 s
  stall timeout) is reported. Unknown-msgid frames are not compared (neither side can check them).
* Sync losses: after C# counts a frame, the growth of `MAVState.synclost` since the first check must
  equal the growth of the core's count for that sysid/compid. Only the growth is compared, since
  the C# count may predate the shadow, and only while both sides have produced the same frames.
  (Lost-packet counts decay with timing that differs by microseconds, so they are not compared.)

**What it logs** (to `MissionPlanner.log`): "started" with the core's version, or "not available"
and why; up to 50 detailed differences (`missed by core` / `extra in core` with the frame's
header and first bytes, `sync loss differs`); a summary every minute when something changed; and
the totals when the link closes.

**Reading the results.** `missed by core` is a bug in the core and blocks switching over. `extra
in core` is expected where the core recovers frames the C# parser loses (see the differences
above) and where C# rejects a MAVLink 1 frame whose length is not the message's base length,
which the upstream parser accepts; anything else is worth a look. Bytes that other code reads from
`BaseStream` directly (terminal, firmware upload) reach neither parser, so they cause no
differences.

**Parity test.** `bindings/csharp/MpCoreShadowParity.cs` drives the same class with Mission
Planner's C# parser (`MavlinkParse`) over a recorded flight (`flight.tlog` in the pymavlink
submodule: 163,851 MAVLink 1 frames from three systems). Clean, every frame matches with no
differences; with line noise and 316 corrupted frames, the core matches every frame C# accepted
(158,536) and recovers 4,999 more.

## Native receive

With `<mpcore_receive>True</mpcore_receive>` in `config.xml` (same place as `mpcore_shadow`),
`MAVLinkInterface.readPacketAsync` no longer runs its byte loop: `MpCoreReceiver`
(`ExtLibs/ArduPilot/Mavlink/`) reads `BaseStream`, the core frames the bytes, and each frame enters
the C# code where the byte loop used to end (the `framed:` label). From there on nothing changed:
the C# CRC and MAVLink 1 length checks, signature verification with `MAVState`'s keys and
`signingignore` (the core runs with signatures ignored), the TARGET32 filter, tlog, sequence and
loss counting, `processInfoFromStream`, subscriptions and events. The C# 20-byte resync is skipped,
as the core has already scanned the bytes after a damaged frame. The setting is read once per
connection; if the native library is missing it logs "mpcore receive: not available" and the C#
byte loop runs as before.

* **Reading.** At most 64 bytes per read, so bytes past the current frame stay in the port for code
  that takes the port over and reads it directly (terminal, firmware upload). Frames parsed from the
  same chunk wait in the receiver; `MAVLinkInterface.HasBufferedFrames` reports them, and
  `MainV2.SerialReader` keeps reading while it is set (it used to wait for `BytesToRead`).
* **Raw log and text.** Every byte read still goes to the raw log. Text lines outside frames go to
  `plaintxtline`/`plaintxtlinebuffer` (used by the "no heartbeat" error) and the log as before, as
  whole lines rather than character by character on the console.
* **Timeouts.** A call returns `Invalid` after 1.2 s without bytes, as the byte loop did.
* **Not used** in log playback, which keeps its own reader.

Tests: `bindings/csharp/MpCoreReceiverTest.cs` runs the receiver on a fake port (the 163,851-frame
flight, text and noise, the 64-byte reads, buffered frames, timeouts, closing).
`MissionPlannerTests/Mavlink/MpCoreReceiveTests.cs` runs `MAVLinkInterface` itself both ways on the
same streams (several vehicles, MAVLink 1 and 2, a 32-bit ID, a command to another system, a gap,
text) and requires the same frames, loss counters and text, and that frames after a corrupted
length byte are recovered; CI runs it with the native library (`MPCORE_REQUIRED=1`).

## Native send

With `<mpcore_send>True</mpcore_send>` (independent of `mpcore_receive`),
`MAVLinkInterface.generatePacket` has the core encode each frame through `mp_frame_encode`, which
keeps no state: the caller passes every decision. `generatePacket` still makes them all as before
(MAVLink 1 when the vehicle has not sent MAVLink 2, signing with `MAVState`'s key, link ID and
timestamp, the sequence number, a TARGET32 header for a 32-bit destination), writes the frame under
`writelock`, logs it to the tlog and raises `OnPacketSent`; only the bytes come from the core. A
message the core's dialect lacks (`zVIDEO_STREAM_INFORMATION` from `offspec.xml`) is encoded by the
C# code as before, and so is everything if the native library is missing.

Tests: `bindings/csharp/MpCoreEncoderParity.cs` encodes every message of the C# dialect (353, 6,066
frames: MAVLink 1 and 2, zero-trimmed, signed, 8 and 32-bit source and destination) with the C#
encoder and the core and requires the same bytes, and the same CRC extras and lengths in both
dialect tables. (Its reference MAVLink 1 frame is cut to the base length as `generatePacket` does:
`MavlinkParse.GenerateMAVLinkPacket10` itself sends the MAVLink 2 extension fields in MAVLink 1
frames.) `MissionPlannerTests/Mavlink/MpCoreSendTests.cs` sends every message through
`generatePacket` to four vehicles both ways and requires the same frames (signed ones up to their
timestamp, and the core's signatures verified).

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
* `bindings/csharp/MpCoreShadowParity.cs`: the shadow comparer with Mission Planner's C# parser on
  a recorded flight (above).
* `bindings/csharp/MpCoreReceiverTest.cs` and `MissionPlannerTests/Mavlink/MpCoreReceiveTests.cs`:
  native receive (above).
* `bindings/csharp/MpCoreEncoderParity.cs` and `MissionPlannerTests/Mavlink/MpCoreSendTests.cs`:
  native send (above).
* Everything also runs under AddressSanitizer and UBSan (`linux-asan`).
