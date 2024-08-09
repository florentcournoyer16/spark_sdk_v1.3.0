zephyr_library_named(iface_audio_4-channel)set_target_properties(iface_audio_4-channel PROPERTIES LINKER_LANGUAGE C)

target_link_libraries(iface_audio_4-channel PUBLIC wps max98091_driver audio_core hardware)
target_include_directories(iface_audio_4-channel PUBLIC ${CMAKE_CURRENT_LIST_DIR})

if (HARDWARE STREQUAL "EVK")
    target_sources(iface_audio_4-channel PRIVATE iface_audio_4-channel_evk.c)

elseif(HARDWARE STREQUAL "PULSAR")
    target_sources(iface_audio_4-channel PRIVATE iface_audio_4-channel_pulsar.c)

elseif(HARDWARE STREQUAL "QUASAR")
    target_sources(iface_audio_4-channel PRIVATE iface_audio_4-channel_quasar.c)
    set(TINY_USB_CONFIG_PATH ${INTERFACE_PATH}/lib/tinyusb_config/single_cdc_high_speed/ CACHE INTERNAL "Path to tinyusb cfg files used by CMakeLists.")
    target_link_libraries(iface_audio_4-channel PRIVATE tinyusb)
endif()
