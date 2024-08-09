zephyr_library_named(iface_audio_user_data)set_target_properties(iface_audio_user_data PROPERTIES LINKER_LANGUAGE C)

target_link_libraries(iface_audio_user_data PUBLIC wps max98091_driver audio_core hardware)
target_include_directories(iface_audio_user_data PUBLIC ${CMAKE_CURRENT_LIST_DIR})

if (HARDWARE STREQUAL "EVK")
    target_sources(iface_audio_user_data PRIVATE iface_audio_user_data_evk.c)

elseif(HARDWARE STREQUAL "PULSAR")
    target_sources(iface_audio_user_data PRIVATE iface_audio_user_data_pulsar.c)

elseif(HARDWARE STREQUAL "QUASAR")
    target_sources(iface_audio_user_data PRIVATE iface_audio_user_data_quasar.c)
    set(TINY_USB_CONFIG_PATH ${INTERFACE_PATH}/lib/tinyusb_config/single_cdc_high_speed/ CACHE INTERNAL "Path to tinyusb cfg files used by CMakeLists.")
    target_link_libraries(iface_audio_user_data PRIVATE tinyusb)
endif()
