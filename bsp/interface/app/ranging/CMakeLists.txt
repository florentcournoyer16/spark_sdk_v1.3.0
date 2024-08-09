zephyr_library_named(iface_ranging)
target_link_libraries(iface_ranging PUBLIC wps hardware ranging_core)
target_include_directories(iface_ranging PUBLIC ${CMAKE_CURRENT_LIST_DIR})

if (HARDWARE STREQUAL "EVK")
    target_sources(iface_ranging PRIVATE iface_ranging_evk.c)

elseif(HARDWARE STREQUAL "PULSAR")
    target_sources(iface_ranging PRIVATE iface_ranging_pulsar.c)

elseif(HARDWARE STREQUAL "QUASAR")
    target_sources(iface_ranging PRIVATE iface_ranging_quasar.c)
    set(TINY_USB_CONFIG_PATH ${INTERFACE_PATH}/lib/tinyusb_config/single_cdc_high_speed/ CACHE INTERNAL "Path to tinyusb cfg files used by CMakeLists.")
    target_link_libraries(iface_ranging PRIVATE tinyusb)
endif()
