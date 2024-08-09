zephyr_library_named(iface_core)
set_target_properties(iface_core PROPERTIES LINKER_LANGUAGE C)

set(INTERFACE_PATH ${CMAKE_CURRENT_LIST_DIR}/interface CACHE INTERNAL "Path to BSP interface cores and apps root.")
set(IFACE_WIRELESS_PATH ${INTERFACE_PATH}/wireless_core CACHE INTERNAL "Path to BSP interface Wireless Core source files.")
set(IFACE_APP_PATH ${INTERFACE_PATH}/app CACHE INTERNAL "Path to BSP interface App CMakeLists file.")
set(IFACE_LIB_PATH ${INTERFACE_PATH}/lib CACHE INTERNAL "Path to BSP interface SPARK Libraries files.")

message("Selecting board support package (bsp) for ${HARDWARE} hardware.")

set(IFACE_LIB_CRITICAL_SECTION_OPT_PATH ${IFACE_LIB_PATH}/queue/nrf CACHE INTERNAL "Path to SPARK circular queue critical section optimization files.")

target_sources(iface_core
    PRIVATE
        ${IFACE_WIRELESS_PATH}/iface_wireless_nrf.c
    PUBLIC
        ${IFACE_LIB_CRITICAL_SECTION_OPT_PATH}/circular_queue_critical_section.h
)

set(SPARK_CIRC_QUEUE_OPT_PATH ${IFACE_LIB_CRITICAL_SECTION_OPT_PATH} CACHE PATH "Path to BSP interface SPARK Libraries files project wide" FORCE)
set(SPARK_LIB_OPT_DEPENDANCIES hardware CACHE INTERNAL "Path to the BSP library for SPARK library dependancies")

target_include_directories(iface_core PUBLIC ${IFACE_WIRELESS_PATH} ${IFACE_LIB_CRITICAL_SECTION_OPT_PATH} ${NRF_HW_CONGIG_PATH} ${BSP_PATH})

target_link_libraries(iface_core 
    PRIVATE 
        wps 
        app
)

add_subdirectory(interface/app)
add_subdirectory(${SPARK_LIB_OPT_DEPENDANCIES})
