target_sources(wps
    PRIVATE
        swc_api.c
    PUBLIC
        swc_api.h
)
target_include_directories(wps PUBLIC ${CMAKE_CURRENT_LIST_DIR})
