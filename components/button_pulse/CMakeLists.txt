idf_component_register(
    SRCS "button_pulse.c"
    INCLUDE_DIRS "include"
    REQUIRES report_decoder
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wall -Wextra -Werror)
