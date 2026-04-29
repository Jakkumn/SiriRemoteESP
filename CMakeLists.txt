cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(siri-bridge)

# Upstream xiph/opus emits SILK code that newer GCC flags as
# `maybe-uninitialized`. The flagged paths are false positives (the
# variables are written through pointer aliasing the optimizer can't
# follow), but ESP-IDF treats the warning as an error. Demote it to a
# warning for the micro-opus component only — our own code keeps the
# strict default. esphome/micro-opus upstream may eventually patch this.
if(TARGET __idf_esphome__micro-opus)
    target_compile_options(__idf_esphome__micro-opus PRIVATE -Wno-error=maybe-uninitialized)
endif()
