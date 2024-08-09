cmake_minimum_required(VERSION 3.13)
option(BUILD_SDK_UNIT_TESTS "Build SDK unit tests" OFF)

# Supply the compiler with system-wide processor definitions for the count of radios associated with a given CMAKE configuration.

add_compile_definitions(SWC_RADIO_COUNT=1)


add_subdirectory(lib)
add_subdirectory(core)
add_subdirectory(app)
add_subdirectory(bsp)