option(BPX_SDK_BUILD_HARDWARE_TESTS
    "Build interactive tests that move a real robot" OFF)

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
