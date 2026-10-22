# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Mission Planner is the ArduPilot Ground Control Station (GCS): a large C# / .NET Framework
Windows Forms desktop application for configuring, flashing, planning missions for, and
monitoring ArduPilot/PX4 vehicles over the MAVLink protocol. The same source also targets
Android (Xamarin) and macOS/iOS via a cross-platform library build.

## Build & run

The main build is **Windows + Visual Studio 2022 + MSBuild**. Linux has a separate build,
`Linux/build.sh` (needs the .NET 10 SDK, mono-devel, rsync; see `Linux/README.md`); macOS only
*runs* prebuilt binaries via Mono. The C++ core `ExtLibs/mpcore` builds anywhere with CMake (below).

```bat
git submodule update --init          :: required once; pulls ExtLibs/mono and others
.nuget\nuget.exe restore MissionPlanner.sln
msbuild -v:m -restore -t:Build -p:Configuration=Release MissionPlanner.sln
```

- The main app is built from **`MissionPlanner.sln`** → `MissionPlanner.csproj` (`net472`, `Exe`).
- Output goes to **`bin\Release\net461\`** (or `bin\Debug\net461\`) — the `net461` folder name is
  intentional (`AppendTargetFrameworkToOutputPath=false`), it is *not* a separate target.
- `build.bat`, `build - debug.bat`, `build - Clean.bat` are the maintainer's local build scripts
  (they also do appx packaging/signing/rsync — not needed for plain builds).
- To import the exact VS workload/component set, use `vs2022.vsconfig` (or `vs2019`/`vs2026`).

### Cross-platform library build (Android / Mac)

`MissionPlannerLib.csproj` (`netstandard2.0`, `library`) recompiles the same sources with
`DefineConstants=...;LIB`. Android packaging is done via `build - Lib.ps1` and
`ExtLibs/Xamarin/Xamarin.Android`. See the `LIB` conditional-compilation note below — it is the
single most important thing to understand before touching shared code.

### CI

GitHub Actions (`.github/workflows/main.yml` = "DotNet Build", plus `android.yml`, `mac.yml`)
build on Windows with MSBuild. CI builds the solution and zips `bin/Release/net461`; tagging
`beta` publishes a release. `linux.yml` builds and smoke-tests the Linux package; `mpcore.yml`
builds and tests `ExtLibs/mpcore` on Linux and Windows (x86/x64).

### C++ core (`ExtLibs/mpcore`)

The start of the move from C# to C/C++: the MAVLink core in C++17 with a C ABI (`mpcore.h`),
loaded by the C# app through P/Invoke first, a Qt app later. Read `ExtLibs/mpcore/DESIGN.md`
before changing it. It does no I/O (bytes in via `mp_link_feed`, out via a callback) and is
single-threaded per link. MAVLink C headers are generated at build time by the pinned
`third_party/pymavlink` submodule from `ExtLibs/Mavlink/message_definitions/all.xml`. Inside the
app it runs behind three settings, all off by default: `mpcore_shadow` (`MpCoreShadow`: parses the
same bytes as `MAVLinkInterface.readPacketAsync` and logs differences), `mpcore_receive`
(`MpCoreReceiver`: frames the bytes instead of `readPacketAsync`'s byte loop, everything after
framing unchanged), `mpcore_send` (`generatePacket` keeps every decision, the core encodes) and
`mpcore_params` (the parameter download, set and read decided in the core) and `mpcore_missions`
(mission download and upload) `mpcore_ftp` (MAVFtp) and `mpcore_state_shadow` (CurrentState decoded beside it), phase 2 in DESIGN.md.

```sh
git submodule update --init ExtLibs/mpcore/third_party/pymavlink
cmake --preset linux-debug -S ExtLibs/mpcore && cmake --build ExtLibs/mpcore/build/linux-debug
ctest --test-dir ExtLibs/mpcore/build/linux-debug   # also: linux-asan, windows-x64/x86 presets
```

## Tests

`MissionPlannerTests/` is an MSTest project (`MSTest.TestFramework` 2.2.10, `net472`) that
references the main projects. Tests use `[TestClass]` / `[TestMethod]` and `Assert`.

```bat
:: build then run via the VS Test Explorer, or:
vstest.console.exe bin\Release\net461\MissionPlannerTests.dll
:: single test:
vstest.console.exe MissionPlannerTests.dll /Tests:DetectBoardTest
```

Coverage is light and focused on pure logic (board detection, firmware parsing, downloads), plus
the 32-bit system ID MAVLink suite (`MissionPlannerTests/Mavlink/Sysid32Tests.cs`), which is the
only part CI runs (`dotnet test --filter FullyQualifiedName~Sysid32Tests` in `main.yml`). When
adding tests, mirror the existing folder layout (`MissionPlannerTests/Utilities`, `/GCSViews`,
`/Mavlink`, ...).

## Architecture (the big picture)

The dependency flow is: **transport → protocol → vehicle state → UI**.

- **`Program.cs`** — process entry point and crash/exception handling.
- **`MainV2.cs`** — the main `Form` and central hub. It owns the active connection as the static
  `MainV2.comPort` (a `MAVLinkInterface`), hosts the menu, and swaps the `GCSViews` screens in and
  out of the main window. Most global state hangs off `MainV2`. This file is ~190 KB; navigate by
  symbol, don't read top-to-bottom.
- **`GCSViews/`** — the top-level screens the menu switches between: `FlightData` (HUD/telemetry/
  map), `FlightPlanner` (waypoint/mission editor), `InitialSetup`, `ConfigurationView` (Config/
  Tuning + the many `Config*.cs` sub-panels), `SITL`, `Help`.

### Connection & telemetry stack (in `ExtLibs/`)

- **`ExtLibs/Comms/`** — transport layer implementing `ICommsSerial`: `CommsSerialPort`,
  `CommsTCP`, `CommsUDP`, `CommsBLE`, `CommsNTRIP`, etc. Pick a transport, hand it to the interface.
- **`ExtLibs/Mavlink/` (`MAVLink.csproj`)** — the MAVLink protocol: generated message structs,
  `MavlinkParse` (frame parse/serialize), CRC. Largely machine-generated.
- **`ExtLibs/ArduPilot/Mavlink/`** — the live link logic:
  - `MAVLinkInterface` (`: MAVLink, IMAVLinkInterface`) — reads/writes packets over a transport,
    exposes high-level operations (get/set params, mission up/download, command_long, etc.).
  - `MAVState` — represents a single connected vehicle (sysid/compid) and holds `cs`, its
    `CurrentState`. `MAVList` tracks multiple vehicles on one link.
  - `CurrentState` (`ExtLibs/ArduPilot/CurrentState.cs`) — the decoded, UI-facing telemetry model
    (attitude, position, battery, mode...). Forms/controls bind to `MainV2.comPort.MAV.cs`.
- **`ExtLibs/ArduPilot/`** — the vehicle/domain layer beyond telemetry: firmware
  (`APFirmware`, `Firmwares`), geofences, `BoardDetect`, mission packing.

### Other notable `ExtLibs/` (each is its own csproj; there are ~90)

`Controls/` (custom WinForms controls), `Core/` + `Interfaces/` (shared primitives, the
`ServiceLocator` DI registry, `ICommsSerial`), `Maps/` + `GMap.NET.*` (mapping), `GDAL`/`ProjNet`/
`SharpKml` (geo/projection/KML), `DroneCAN`/`UAVCANFlasher`, `Comms`, plus many vendored DLLs.
The solution has ~93 projects; treat `ExtLibs/` as the bulk of the real functionality.

### Plugins & scripting

- **`Plugin/`** (`Plugin` base class + `PluginLoader`) and **`Plugins/`** (built-in plugins plus
  `exampleNN-*.cs` templates) — plugins are loaded at runtime to extend the UI/behavior.
- **`Script.cs` + `Scripts/`** — IronPython scripting host exposed to users (`cs`, `Script.*` API).

## Conventions & gotchas

- **`LIB` conditional compilation is the #1 trap.** Shared code is compiled both into the desktop
  `Exe` (full `System.Drawing` / `System.Windows.Forms`) and into the netstandard `LIB` build for
  Android/Mac. You will see `extern alias Drawing;` guarded by `#if !LIB`, and `ZZZLibShims.cs`
  provides cross-platform shims (e.g. `MissionPlanner.Drawing`) for the `LIB` build. When adding
  code that touches drawing, WinForms, P/Invoke, or other desktop-only APIs, guard it with
  `#if !LIB` or it will break the Android/Mac build.
- **Localization.** Each Form has an English `Name.resx` (the source of truth) plus per-culture
  siblings like `FlightData.ko-KR.resx`, `MainV2.zh-Hans.resx`. App strings go through `Strings.resx`
  / the `L10N` class (`Strings.Culture` is set from the `language` setting). Translations are
  managed via Crowdin (`crowdin.bat`) — **do not hand-edit the non-English `.resx`**; edit the
  English resource and let translation flow back.
- **WinForms file triplets.** A screen is `Foo.cs` (logic) + `Foo.Designer.cs` (generated layout) +
  `Foo.resx` (embedded resources). Edit the `Designer.cs` through intent, not by hand where possible.
- **`createproj.bat`** regenerates the `<Compile>`/`<EmbeddedResource>` item lists for the csproj by
  scanning the tree — useful when files are added/removed but kept out of sync with the project.
- **`ServiceLocator`** (`ExtLibs/Interfaces/ServiceLocator.cs`) is the lightweight DI used to inject
  platform-specific implementations (resolve with `ServiceLocator.Get<T>()`).
- **`.editorconfig`** intentionally downgrades many `CAxxxx` analyzer rules to suggestion/silent —
  don't treat those as actionable warnings.
- **`Resources/`** holds icons/images; themes are `*.mpsystheme` files driven by `ThemeManager.cs`.
