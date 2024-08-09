target_sources(wps
    PRIVATE
        wps_phy_common.c
    PUBLIC
        wps_phy_def.h
)

target_include_directories(wps PUBLIC ${CMAKE_CURRENT_LIST_DIR})
