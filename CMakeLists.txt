cmake_minimum_required(VERSION 3.29)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

set(CMAKE_Swift_COMPILATION_MODE wholemodule)
set(CMAKE_Swift_COMPILER_WORKS YES)

project(Picoliter)
pico_sdk_init()
enable_language(Swift)

add_executable(Picoliter Main.swift)
pico_enable_stdio_usb(Picoliter 1)
set_target_properties(Picoliter PROPERTIES LINKER_LANGUAGE CXX)

# Clear the default COMPILE_OPTIONS which include C specific compiler flags that the Swift compiler will not accept
# Instead, set those options to only apply when compiling C code.
set_target_properties(pico_standard_link PROPERTIES INTERFACE_COMPILE_OPTIONS "")
target_compile_options(pico_standard_link INTERFACE "$<$<COMPILE_LANGUAGE:C>:SHELL: -ffunction-sections -fdata-sections>")

set(SWIFT_INCLUDES)
foreach(dir ${CMAKE_C_IMPLICIT_INCLUDE_DIRECTORIES})
    string(CONCAT SWIFT_INCLUDES ${SWIFT_INCLUDES} "-Xcc ")
    string(CONCAT SWIFT_INCLUDES ${SWIFT_INCLUDES} "-I${dir} ")
endforeach()

target_compile_options(Picoliter PUBLIC "$<$<COMPILE_LANGUAGE:Swift>:SHELL:
        -enable-experimental-feature Embedded
        -target armv6m-none-none-eabi -Xcc -mfloat-abi=soft -Xcc -fshort-enums -Xfrontend -function-sections
        -import-bridging-header \"${CMAKE_CURRENT_LIST_DIR}/BridgingHeader.h\"
        ${SWIFT_INCLUDES}
    >")

target_link_libraries(Picoliter pico_stdlib hardware_uart hardware_gpio)
pico_add_extra_outputs(Picoliter)
