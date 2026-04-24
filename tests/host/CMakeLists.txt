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

# Build the decoder as a plain static library from its real source path.
# No ESP-IDF headers — that's the invariant these tests defend.
set(DECODER_DIR "${CMAKE_CURRENT_LIST_DIR}/../../components/report_decoder")
add_library(report_decoder STATIC
    ${DECODER_DIR}/report_decoder.c
)
target_include_directories(report_decoder PUBLIC ${DECODER_DIR}/include)

enable_testing()

# One executable per test file — CTest runs each independently so a crash
# in one doesn't mask others.
function(add_host_test name)
    add_executable(${name} ${name}.c)
    target_link_libraries(${name} PRIVATE report_decoder)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_host_test(test_report_decoder)
