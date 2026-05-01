cmake_minimum_required(VERSION 3.16)
project(siri_bridge_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_EXTENSIONS OFF)

# Default to Debug so assert() stays live. `-UNDEBUG` belt-and-braces for
# toolchains that try to be clever.
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()
add_compile_options(-Wall -Wextra -Wpedantic -Werror -UNDEBUG)

# AddressSanitizer + UndefinedBehaviorSanitizer on host tests. Virtually
# free to run on our tiny test binaries and catches memory bugs that would
# otherwise only show up on-device.
add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
add_link_options(-fsanitize=address,undefined)

# Build each component as a plain static library from its real source path.
# No ESP-IDF headers — that's the invariant these tests defend.
set(DECODER_DIR "${CMAKE_CURRENT_LIST_DIR}/../../components/report_decoder")
add_library(report_decoder STATIC
    ${DECODER_DIR}/report_decoder.c
)
target_include_directories(report_decoder PUBLIC ${DECODER_DIR}/include)

set(EVENT_STATE_DIR "${CMAKE_CURRENT_LIST_DIR}/../../components/event_state")
add_library(event_state STATIC
    ${EVENT_STATE_DIR}/event_state.c
)
target_include_directories(event_state PUBLIC ${EVENT_STATE_DIR}/include)
target_link_libraries(event_state PUBLIC report_decoder)

# Just the pure validator from siri_audio. The full siri_audio.c pulls in
# opus + FreeRTOS + esp-log, which we deliberately avoid in the host
# environment — siri_audio_validate.c has no such deps.
set(SIRI_AUDIO_DIR "${CMAKE_CURRENT_LIST_DIR}/../../components/siri_audio")
add_library(siri_audio_validate STATIC
    ${SIRI_AUDIO_DIR}/siri_audio_validate.c
)
target_include_directories(siri_audio_validate PUBLIC ${SIRI_AUDIO_DIR}/include)

enable_testing()

# One executable per test file — CTest runs each independently so a crash
# in one doesn't mask others.
function(add_host_test name)
    add_executable(${name} ${name}.c)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_host_test(test_report_decoder)
target_link_libraries(test_report_decoder PRIVATE report_decoder)

add_host_test(test_event_state)
target_link_libraries(test_event_state PRIVATE event_state)

add_host_test(test_siri_audio)
target_link_libraries(test_siri_audio PRIVATE siri_audio_validate)
