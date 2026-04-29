cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(siri-bridge)

# Upstream xiph/opus emits SILK code that newer GCC flags as
# `maybe-uninitialized`. The flagged paths are false positives (the
# variables are written through pointer aliasing the optimizer can't
# follow), and esphome/micro-opus's vendored snapshot hasn't been
# patched for it. Silence the warning for the micro-opus component
# only — our own code keeps the strict default. Revisit when upstream
# updates.
if(TARGET __idf_esphome__micro-opus)
    target_compile_options(__idf_esphome__micro-opus PRIVATE -Wno-maybe-uninitialized)
endif()
