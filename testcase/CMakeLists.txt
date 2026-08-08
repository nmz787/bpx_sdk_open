option(BPX_SDK_BUILD_HARDWARE_TESTS
    "Build interactive tests that move a real robot" OFF)

if(BPX_SDK_BUILD_DIFFERENTIAL_TESTS AND TARGET bpx_sdk_recovered AND TARGET bpx_sdk_precompiled)
    find_package(Python3 COMPONENTS Interpreter QUIET)

    add_executable(
        api_assumption_probe_recovered
        api_assumption_probe.cpp
    )
    target_link_libraries(api_assumption_probe_recovered
        PRIVATE
            bpx_sdk_recovered
    )
    set_target_properties(api_assumption_probe_recovered PROPERTIES
        BUILD_RPATH "$<TARGET_FILE_DIR:bpx_sdk_recovered>"
    )

    add_executable(
        api_assumption_probe_precompiled
        api_assumption_probe.cpp
    )
    target_link_libraries(api_assumption_probe_precompiled
        PRIVATE
            bpx_sdk_precompiled
    )
    set_target_properties(api_assumption_probe_precompiled PROPERTIES
        BUILD_RPATH "$<TARGET_FILE_DIR:bpx_sdk_precompiled>"
    )

    add_test(
        NAME compare_api_assumptions
        COMMAND ${CMAKE_COMMAND}
            -DRECOVERED_PROBE=$<TARGET_FILE:api_assumption_probe_recovered>
            -DPRECOMPILED_PROBE=$<TARGET_FILE:api_assumption_probe_precompiled>
            -P "${CMAKE_CURRENT_LIST_DIR}/compare_api_assumptions.cmake"
    )

    add_executable(
        recovered_runtime_probe
        recovered_runtime_probe.cpp
    )
    target_link_libraries(recovered_runtime_probe
        PRIVATE
            bpx_sdk_recovered
    )
    set_target_properties(recovered_runtime_probe PROPERTIES
        BUILD_RPATH "$<TARGET_FILE_DIR:bpx_sdk_recovered>"
    )

    add_test(
        NAME recovered_runtime_probe
        COMMAND $<TARGET_FILE:recovered_runtime_probe>
    )

    add_executable(
        recovered_packet_parse_probe
        recovered_packet_parse_probe.cpp
    )
    target_link_libraries(recovered_packet_parse_probe
        PRIVATE
            bpx_sdk_recovered
    )
    set_target_properties(recovered_packet_parse_probe PROPERTIES
        BUILD_RPATH "$<TARGET_FILE_DIR:bpx_sdk_recovered>"
    )

    add_test(
        NAME recovered_packet_parse_probe
        COMMAND $<TARGET_FILE:recovered_packet_parse_probe>
    )

    add_executable(
        transport_socket_probe
        transport_socket_probe.cpp
    )
    target_link_libraries(transport_socket_probe
        PRIVATE
            bpx_sdk_recovered
    )
    set_target_properties(transport_socket_probe PROPERTIES
        BUILD_RPATH "$<TARGET_FILE_DIR:bpx_sdk_recovered>"
    )

    add_test(
        NAME transport_socket_probe
        COMMAND $<TARGET_FILE:transport_socket_probe>
    )

    if(Python3_Interpreter_FOUND)
        add_test(
            NAME python_recovered_runtime_probe
            COMMAND ${Python3_EXECUTABLE}
                "${CMAKE_CURRENT_LIST_DIR}/python_recovered_runtime_probe.py"
                "${CMAKE_CURRENT_LIST_DIR}/.."
                "$<TARGET_FILE:bpx_sdk_recovered>"
        )

        add_test(
            NAME python_compare_api_assumptions
            COMMAND ${Python3_EXECUTABLE}
                "${CMAKE_CURRENT_LIST_DIR}/python_compare_api_assumptions.py"
                "${CMAKE_CURRENT_LIST_DIR}/.."
                "${CMAKE_CURRENT_LIST_DIR}/../lib/libbpx_sdk_x86_64.so"
                "$<TARGET_FILE:bpx_sdk_recovered>"
        )
    endif()
endif()

if(BPX_SDK_BUILD_HARDWARE_TESTS)
    add_executable(
        motion_level_control_gait_test
        motion_level_control_gait_test.cpp
    )
    target_link_libraries(motion_level_control_gait_test
        PRIVATE
            bpx_sdk
    )
    set_target_properties(motion_level_control_gait_test PROPERTIES
        BUILD_RPATH "${CMAKE_CURRENT_LIST_DIR}/../lib"
    )
    if(WIN32 AND NOT BPX_SDK_USE_RECOVERED_SOURCES)
        add_custom_command(TARGET motion_level_control_gait_test POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${BPX_SDK_RUNTIME_LIBRARY_PATH}"
                "$<TARGET_FILE_DIR:motion_level_control_gait_test>"
            VERBATIM
        )
    endif()
endif()
