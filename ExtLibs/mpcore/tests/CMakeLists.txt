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

# The C# P/Invoke binding against the shared library, when a C# compiler is configured:
#   -DMPCORE_CSHARP_COMPILER=/path/to/mcs.exe (run with mono) or a csc/mcs executable
set(MPCORE_CSHARP_COMPILER "" CACHE FILEPATH "C# compiler for the binding smoke test (optional)")
if(MPCORE_CSHARP_COMPILER)
    find_program(MPCORE_MONO mono REQUIRED)
    set(compiler "${MPCORE_CSHARP_COMPILER}")
    if(compiler MATCHES "\\.exe$" AND NOT WIN32)
        set(compiler "${MPCORE_MONO}" "${MPCORE_CSHARP_COMPILER}")
    endif()
    set(smoke_exe "${CMAKE_CURRENT_BINARY_DIR}/MpCoreSmoke.exe")
    set(sources "${PROJECT_SOURCE_DIR}/bindings/csharp/MpCore.cs"
                "${PROJECT_SOURCE_DIR}/bindings/csharp/MpCoreSmoke.cs")
    # the same bitness as the library, so the 32-bit struct layouts are checked on x86
    if(CMAKE_SIZEOF_VOID_P EQUAL 4)
        set(platform -platform:x86)
    else()
        set(platform -platform:x64)
    endif()
    add_custom_command(
        OUTPUT "${smoke_exe}"
        COMMAND ${compiler} -unsafe -warn:4 -nologo ${platform} "-out:${smoke_exe}" ${sources}
        DEPENDS ${sources}
        COMMENT "Compiling the C# binding smoke test"
        VERBATIM)
    add_custom_target(mpcore_csharp_smoke ALL DEPENDS "${smoke_exe}")
    if(WIN32)
        # P/Invoke finds mpcore.dll in the working directory
        add_test(NAME mpcore_csharp_smoke COMMAND "${smoke_exe}" WORKING_DIRECTORY "$<TARGET_FILE_DIR:mpcore>")
    else()
        set(env "LD_LIBRARY_PATH=$<TARGET_FILE_DIR:mpcore>")
        # an ASan build of the library loaded into mono needs the runtime preloaded; mono itself
        # leaks at exit by design
        if(CMAKE_CXX_FLAGS MATCHES "-fsanitize=[^ ]*address")
            execute_process(COMMAND ${CMAKE_CXX_COMPILER} -print-file-name=libasan.so
                OUTPUT_VARIABLE asan_runtime OUTPUT_STRIP_TRAILING_WHITESPACE)
            list(APPEND env "LD_PRELOAD=${asan_runtime}" "ASAN_OPTIONS=detect_leaks=0")
        endif()
        add_test(NAME mpcore_csharp_smoke
            COMMAND ${CMAKE_COMMAND} -E env ${env} "${MPCORE_MONO}" "${smoke_exe}")
    endif()
endif()
