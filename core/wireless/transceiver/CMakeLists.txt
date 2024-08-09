if(TRANSCEIVER STREQUAL "SR1000")
    add_subdirectory(sr1000)
else()
    add_subdirectory(sr1100)
endif()

target_sources(wps
    PUBLIC
        sr_api_error.h
)

target_include_directories(wps PUBLIC ${CMAKE_CURRENT_LIST_DIR})