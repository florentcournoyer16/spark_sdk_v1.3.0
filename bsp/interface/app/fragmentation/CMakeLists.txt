zephyr_library_named(iface_fragmentation)
target_link_libraries(iface_fragmentation PUBLIC wps hardware)
target_include_directories(iface_fragmentation PUBLIC ${CMAKE_CURRENT_LIST_DIR})

if (HARDWARE STREQUAL "EVK")
    target_sources(iface_fragmentation PRIVATE iface_fragmentation_evk.c)

elseif(HARDWARE STREQUAL "PULSAR")
    target_sources(iface_fragmentation PRIVATE iface_fragmentation_pulsar.c)

elseif(HARDWARE STREQUAL "QUASAR")
    target_sources(iface_fragmentation PRIVATE iface_fragmentation_quasar.c)
    set(TINY_USB_CONFIG_PATH ${INTERFACE_PATH}/lib/tinyusb_config/single_cdc_high_speed/ CACHE INTERNAL "Path to tinyusb cfg files used by CMakeLists.")
    target_link_libraries(iface_fragmentation PRIVATE tinyusb)
endif()
