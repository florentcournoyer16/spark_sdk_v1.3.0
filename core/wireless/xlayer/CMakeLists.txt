target_sources(wps
    PUBLIC
        xlayer.h
        xlayer_queue.h
        xlayer_queue.c
)

target_include_directories(wps PUBLIC ${CMAKE_CURRENT_LIST_DIR})
