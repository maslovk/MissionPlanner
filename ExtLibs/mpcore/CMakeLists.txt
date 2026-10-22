cmake_minimum_required(VERSION 3.21)

project(mpcore VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
# executables next to the shared library, so tests find it on Windows too
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

option(MPCORE_BUILD_TESTS "Build the mpcore tests" ${PROJECT_IS_TOP_LEVEL})
option(MPCORE_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

# The same message definitions Mission Planner generates Mavlink.cs from, so the C# and C++
# sides cannot drift apart.
set(MPCORE_MAVLINK_XML "${CMAKE_CURRENT_SOURCE_DIR}/../Mavlink/message_definitions/all.xml"
    CACHE FILEPATH "Top-level MAVLink dialect XML")
set(MPCORE_PYMAVLINK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/pymavlink"
    CACHE PATH "pymavlink checkout providing the C generator")

include(cmake/MavlinkHeaders.cmake)
mpcore_generate_mavlink_headers(mpcore_mavlink "${MPCORE_MAVLINK_XML}")

function(mpcore_set_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive- /utf-8)
        if(MPCORE_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE /WX)
        endif()
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion
            -Wno-sign-conversion)
        if(MPCORE_WARNINGS_AS_ERRORS)
            target_compile_options(${target} PRIVATE -Werror)
        endif()
    endif()
endfunction()

# The C++ core. Tests and a later Qt application link this directly.
add_library(mpcore_core STATIC
    src/exchange.cpp
    src/frame_parser.cpp
    src/link.cpp
    src/mission_download.cpp
    src/param_download.cpp
    src/protocol.cpp
    src/seq_tracker.cpp
)
add_library(mpcore::core ALIAS mpcore_core)
target_include_directories(mpcore_core
    PUBLIC  "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(mpcore_core PRIVATE mpcore_mavlink)
mpcore_set_warnings(mpcore_core)

# The C ABI shared library that Mission Planner (C#) loads through P/Invoke.
add_library(mpcore SHARED src/capi.cpp)
target_include_directories(mpcore
    PUBLIC  "${CMAKE_CURRENT_SOURCE_DIR}/include"
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(mpcore PRIVATE mpcore_core)
target_compile_definitions(mpcore PRIVATE MPCORE_BUILDING)
set_target_properties(mpcore PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR})
if(WIN32)
    # one DLL per architecture; Mission Planner is AnyCPU and loads x86 or x64 at runtime
    set_target_properties(mpcore PROPERTIES PREFIX "" OUTPUT_NAME "mpcore")
elseif(APPLE)
    target_link_options(mpcore PRIVATE "LINKER:-exported_symbol,_mp_*")
else()
    target_link_options(mpcore PRIVATE
        "LINKER:--version-script=${CMAKE_CURRENT_SOURCE_DIR}/src/mpcore.map")
    set_property(TARGET mpcore APPEND PROPERTY LINK_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/mpcore.map")
endif()
mpcore_set_warnings(mpcore)

if(MPCORE_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

include(GNUInstallDirs)
install(TARGETS mpcore
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
install(FILES include/mpcore/mpcore.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/mpcore)
