zephyr_library_named(iface_hello_world)set_target_properties(iface_hello_world PROPERTIES LINKER_LANGUAGE C)

target_link_libraries(iface_hello_world PUBLIC wps max98091_driver audio_core hardware)
target_include_directories(iface_hello_world PUBLIC ${CMAKE_CURRENT_LIST_DIR})

if (HARDWARE STREQUAL "EVK")
    target_sources(iface_hello_world PRIVATE iface_hello_world_evk.c)

elseif(HARDWARE STREQUAL "PULSAR")
    target_sources(iface_hello_world PRIVATE iface_hello_world_pulsar.c)

elseif(HARDWARE STREQUAL "QUASAR")
    target_sources(iface_hello_world PRIVATE iface_hello_world_quasar.c)
    set(TINY_USB_CONFIG_PATH ${INTERFACE_PATH}/lib/tinyusb_config/single_cdc_high_speed/ CACHE INTERNAL "Path to tinyusb cfg files used by CMakeLists.")
    target_link_libraries(iface_hello_world PRIVATE tinyusb)
endif()
