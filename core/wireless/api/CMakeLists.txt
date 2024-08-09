target_sources(wps
    PRIVATE
        swc_stats.c
    PUBLIC
        swc_error.h
        swc_stats.h
)

if(TRANSCEIVER STREQUAL "SR1000")
    add_subdirectory(sr1000)
else()
    add_subdirectory(sr1100)
endif()

target_include_directories(wps PUBLIC ${CMAKE_CURRENT_LIST_DIR})
