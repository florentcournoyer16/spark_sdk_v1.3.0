cmake_minimum_required(VERSION 3.18)

if (GITLAB_CI_UNIT_TEST)
    # Setup projet for SRAC standalone compilation with unit tests
    project(ranging_core)
    find_package(Catch2 REQUIRED)
    add_executable(sdk_unit_tests "")
    set(BUILD_SDK_UNIT_TESTS ON)

    #Enable error checking
    set(ACCESS_ADV_ERR_CHECK_EN "ACCESS_ADV_ERR_CHECK_EN=1" CACHE INTERNAL "Access advanced error check flag")
endif()

zephyr_library_named(ranging_core)
if(BUILD_SDK_UNIT_TESTS)
    add_subdirectory(unit-tests)
    if (GITLAB_CI_UNIT_TEST)
        add_subdirectory(lib/spark/memory)
        add_subdirectory(lib/spark/buffer)
        add_subdirectory(lib/spark/test_utils)
        target_link_libraries(sdk_unit_tests PRIVATE Catch2::Catch2WithMain ranging_core test_utils)
    endif()
endif()

add_subdirectory(api)

target_link_libraries(ranging_core PUBLIC memory)
