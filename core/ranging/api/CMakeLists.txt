target_sources(ranging_core
    PRIVATE
        srac_api.c
    PUBLIC
        srac_api.h
        srac_error.h
)

target_include_directories(ranging_core PUBLIC ${CMAKE_CURRENT_LIST_DIR})
