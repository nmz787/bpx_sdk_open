execute_process(
    COMMAND "${RECOVERED_PROBE}"
    RESULT_VARIABLE recovered_result
    OUTPUT_VARIABLE recovered_output
    ERROR_VARIABLE recovered_error
)
if(NOT recovered_result EQUAL 0)
    message(FATAL_ERROR
        "Recovered connected-runtime probe failed with exit code ${recovered_result}\n${recovered_error}")
endif()

execute_process(
    COMMAND "${PRECOMPILED_PROBE}"
    RESULT_VARIABLE precompiled_result
    OUTPUT_VARIABLE precompiled_output
    ERROR_VARIABLE precompiled_error
)
if(NOT precompiled_result EQUAL 0)
    message(FATAL_ERROR
        "Precompiled connected-runtime probe failed with exit code ${precompiled_result}\n${precompiled_error}")
endif()

if(NOT recovered_output STREQUAL precompiled_output)
    message(FATAL_ERROR
        "Recovered and precompiled connected-runtime probes diverged.\n"
        "--- recovered ---\n${recovered_output}\n"
        "--- precompiled ---\n${precompiled_output}\n")
endif()
