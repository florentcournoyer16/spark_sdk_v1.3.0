if (MULTI_TRANSCEIVER STREQUAL "DUAL_TRANSCEIVER")
    set(PHY_DIRECTORY multi_radio)
else()
    set(PHY_DIRECTORY single_radio)
endif()

target_sources(wps
    PRIVATE
        wps.c
        wps_callback.c
        wps_mac.c
        wps_mac_protocols.c
        wps_frag.c
        wps_stats.c
        wps_utils.c
        ${PHY_DIRECTORY}/wps_phy.c
    PUBLIC
        wps.h
        wps_callback.h
        wps_def.h
        wps_error.h
        wps_frag.h
        wps_mac.h
        wps_mac_protocols.h
        wps_stats.h
        wps_utils.h
        ${PHY_DIRECTORY}/wps_phy.h
)

if(TRANSCEIVER STREQUAL "SR1000")
    add_subdirectory(sr1000)
else()
    add_subdirectory(sr1100)
endif()

if (RADIO_INTERFACE_QSPI)
    target_compile_definitions(wps PUBLIC RADIO_QSPI_ENABLED=1)
else()
    target_compile_definitions(wps PUBLIC RADIO_QSPI_ENABLED=0)
endif()

target_include_directories(wps PUBLIC ${CMAKE_CURRENT_LIST_DIR} ${SPARK_WPS_CFG_PATH} ${PHY_DIRECTORY})
