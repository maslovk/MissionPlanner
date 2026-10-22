add_executable(mpcore_tests
    test_main.cpp
    test_protocol.cpp
    test_parser.cpp
    test_seq.cpp
    test_link.cpp
)
target_include_directories(mpcore_tests PRIVATE "${PROJECT_SOURCE_DIR}/src")
target_link_libraries(mpcore_tests PRIVATE mpcore_core)
mpcore_set_warnings(mpcore_tests)
add_test(NAME mpcore_tests COMMAND mpcore_tests)

# Written in C, against the installed header and the shared library only, to prove the ABI
# is usable from plain C (and so from P/Invoke).
add_executable(mpcore_capi_test test_capi.c)
target_link_libraries(mpcore_capi_test PRIVATE mpcore)
mpcore_set_warnings(mpcore_capi_test)
add_test(NAME mpcore_capi_test COMMAND mpcore_capi_test)

# C# tests against the shared library, when a C# compiler is configured:
#   -DMPCORE_CSHARP_COMPILER=/path/to/mcs.exe (run with mono) or a csc/mcs executable
#  - MpCoreSmoke: the P/Invoke binding (struct layouts, callbacks, signing);
#  - MpCoreShadowParity: Mission Planner's own C# parser and the shadow comparer that
#    MAVLinkInterface uses, on a recorded flight, clean and with line noise;
#  - MpCoreEncoderParity: every message encoded by the C# encoder and by the core (mpcore_send);
#  - MpCoreReceiverTest: the receiver behind the mpcore_receive setting, on a fake port.
set(MPCORE_CSHARP_COMPILER "" CACHE FILEPATH "C# compiler for the C# tests (optional)")
set(MPCORE_CSHARP_SERIALIZATION_REF "System.Runtime.Serialization.dll" CACHE STRING
    "Reference that ExtLibs/Mavlink needs, as a name or a path")
if(MPCORE_CSHARP_COMPILER)
    set(compiler "${MPCORE_CSHARP_COMPILER}")
    if(NOT WIN32)
        find_program(MPCORE_MONO mono REQUIRED)
        if(compiler MATCHES "\\.exe$")
            set(compiler "${MPCORE_MONO}" "${MPCORE_CSHARP_COMPILER}")
        endif()
    endif()
    # the same bitness as the library, so the 32-bit struct layouts are checked on x86
    if(CMAKE_SIZEOF_VOID_P EQUAL 4)
        set(platform -platform:x86)
    else()
        set(platform -platform:x64)
    endif()

    set(run_env "")
    if(NOT WIN32)
        set(run_env "LD_LIBRARY_PATH=$<TARGET_FILE_DIR:mpcore>")
        # an ASan build of the library loaded into mono needs the runtime preloaded; mono itself
        # leaks at exit by design
        if(CMAKE_CXX_FLAGS MATCHES "-fsanitize=[^ ]*address")
            execute_process(COMMAND ${CMAKE_CXX_COMPILER} -print-file-name=libasan.so
                OUTPUT_VARIABLE asan_runtime OUTPUT_STRIP_TRAILING_WHITESPACE)
            list(APPEND run_env "LD_PRELOAD=${asan_runtime}" "ASAN_OPTIONS=detect_leaks=0")
        endif()
    endif()

    # Builds one C# program and runs it as a test; the program's assemblies go next to it.
    function(mpcore_csharp_test name)
        cmake_parse_arguments(arg "" "" "SOURCES;OPTIONS;DEPENDS;ARGS" ${ARGN})
        set(exe "${CMAKE_CURRENT_BINARY_DIR}/${name}.exe")
        add_custom_command(
            OUTPUT "${exe}"
            COMMAND ${compiler} -warn:4 -nologo ${platform} ${arg_OPTIONS} "-out:${exe}" ${arg_SOURCES}
            DEPENDS ${arg_SOURCES} ${arg_DEPENDS}
            COMMENT "Compiling ${name}.exe"
            VERBATIM)
        add_custom_target(${name} ALL DEPENDS "${exe}")
        if(WIN32)
            # P/Invoke finds mpcore.dll in the working directory
            add_test(NAME ${name} COMMAND "${exe}" ${arg_ARGS} WORKING_DIRECTORY "$<TARGET_FILE_DIR:mpcore>")
        else()
            add_test(NAME ${name} COMMAND ${CMAKE_COMMAND} -E env ${run_env} "${MPCORE_MONO}" "${exe}" ${arg_ARGS})
        endif()
    endfunction()

    set(binding "${PROJECT_SOURCE_DIR}/bindings/csharp/MpCore.cs")
    # Mission Planner's code around the core (ExtLibs/ArduPilot/Mavlink)
    set(mavlink_glue_dir "${PROJECT_SOURCE_DIR}/../ArduPilot/Mavlink")
    set(mavlink_glue "${mavlink_glue_dir}/MpCoreNativeLoader.cs")
    mpcore_csharp_test(MpCoreSmoke SOURCES "${binding}" "${PROJECT_SOURCE_DIR}/bindings/csharp/MpCoreSmoke.cs")

    # Mission Planner's MAVLink library, built from its sources as MAVLink.csproj does
    set(mavlink_dir "${PROJECT_SOURCE_DIR}/../Mavlink")
    set(tlog "${MPCORE_PYMAVLINK_DIR}/generator/swift/Tests/MAVLinkTests/Testdata/flight.tlog")
    if(EXISTS "${mavlink_dir}/MavlinkParse.cs" AND EXISTS "${tlog}")
        file(GLOB mavlink_sources CONFIGURE_DEPENDS "${mavlink_dir}/*.cs")
        set(mavlink_dll "${CMAKE_CURRENT_BINARY_DIR}/MAVLink.dll")
        add_custom_command(
            OUTPUT "${mavlink_dll}"
            COMMAND ${compiler} -nologo -target:library -unsafe -define:TRACE -define:UNSAFE -nowarn:162,612,618
                    "-r:${MPCORE_CSHARP_SERIALIZATION_REF}" "-out:${mavlink_dll}" ${mavlink_sources}
            DEPENDS ${mavlink_sources}
            COMMENT "Compiling Mission Planner's MAVLink.dll"
            VERBATIM)
        mpcore_csharp_test(MpCoreShadowParity
            SOURCES "${binding}" ${mavlink_glue} "${mavlink_glue_dir}/MpCoreShadow.cs"
                    "${PROJECT_SOURCE_DIR}/bindings/csharp/MpCoreShadowParity.cs"
            OPTIONS "-r:${mavlink_dll}"
            DEPENDS "${mavlink_dll}"
            ARGS "${tlog}")
        # every message of the C# dialect, encoded by the C# encoder and by the core
        mpcore_csharp_test(MpCoreEncoderParity
            SOURCES "${binding}" "${PROJECT_SOURCE_DIR}/bindings/csharp/MpCoreEncoderParity.cs"
            OPTIONS "-r:${mavlink_dll}"
            DEPENDS "${mavlink_dll}")
        # the receiver that replaces readPacketAsync's byte loop, against a fake port
        mpcore_csharp_test(MpCoreReceiverTest
            SOURCES "${binding}" ${mavlink_glue} "${mavlink_glue_dir}/MpCoreReceiver.cs"
                    "${PROJECT_SOURCE_DIR}/../Interfaces/ICommsSerial.cs"
                    "${PROJECT_SOURCE_DIR}/../Utilities/PollDelay.cs"
                    "${PROJECT_SOURCE_DIR}/bindings/csharp/MpCoreReceiverTest.cs"
            ARGS "${tlog}")
    endif()
endif()
