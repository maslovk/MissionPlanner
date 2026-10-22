# Generates the MAVLink C headers at build time with the pinned pymavlink generator, into the
# build tree. The output (about 11 MB for all.xml) is not committed; only the generator and the
# XML definitions are.

find_package(Python3 3.8 REQUIRED COMPONENTS Interpreter)

function(mpcore_generate_mavlink_headers target xml)
    if(NOT EXISTS "${MPCORE_PYMAVLINK_DIR}/generator/mavgen_c.py")
        message(FATAL_ERROR
            "pymavlink not found at ${MPCORE_PYMAVLINK_DIR}.\n"
            "Run: git submodule update --init ExtLibs/mpcore/third_party/pymavlink")
    endif()

    get_filename_component(xml_dir "${xml}" DIRECTORY)
    get_filename_component(xml_name "${xml}" NAME_WE)
    file(GLOB xml_deps CONFIGURE_DEPENDS "${xml_dir}/*.xml")
    file(GLOB generator_deps CONFIGURE_DEPENDS
        "${MPCORE_PYMAVLINK_DIR}/generator/*.py"
        "${MPCORE_PYMAVLINK_DIR}/generator/C/include_v2.0/*.h")

    set(out_dir "${CMAKE_CURRENT_BINARY_DIR}/generated/mavlink")
    set(stamp "${out_dir}/${xml_name}/mavlink.h")
    # pymavlink is imported as a package, so its parent directory goes on PYTHONPATH
    get_filename_component(pymavlink_parent "${MPCORE_PYMAVLINK_DIR}" DIRECTORY)

    add_custom_command(
        OUTPUT "${stamp}"
        COMMAND ${CMAKE_COMMAND} -E rm -rf "${out_dir}"
        COMMAND ${CMAKE_COMMAND} -E env "PYTHONPATH=${pymavlink_parent}"
                "${Python3_EXECUTABLE}" -m pymavlink.tools.mavgen
                --lang=C --wire-protocol=2.0 --no-validate
                "--output=${out_dir}" "${xml}"
        DEPENDS ${xml_deps} ${generator_deps}
        COMMENT "Generating MAVLink C headers from ${xml_name}.xml"
        VERBATIM)
    add_custom_target(${target}_generate DEPENDS "${stamp}")

    add_library(${target} INTERFACE)
    add_dependencies(${target} ${target}_generate)
    # SYSTEM: the generated headers are not held to mpcore's warning flags
    target_include_directories(${target} SYSTEM INTERFACE "${out_dir}")
    target_compile_definitions(${target} INTERFACE
        MPCORE_MAVLINK_DIALECT_HEADER="${xml_name}/mavlink.h")
endfunction()
