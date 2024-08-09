zephyr_library_named(iface_pairing_gaming_hub)set_target_properties(iface_pairing_gaming_hub PROPERTIES LINKER_LANGUAGE C)

target_link_libraries(iface_pairing_gaming_hub PUBLIC wps max98091_driver audio_core hardware)
target_include_directories(iface_pairing_gaming_hub PUBLIC ${CMAKE_CURRENT_LIST_DIR})

if (HARDWARE STREQUAL "EVK")
    target_sources(iface_pairing_gaming_hub PRIVATE iface_pairing_gaming_hub_evk.c)

elseif(HARDWARE STREQUAL "PULSAR")
    target_sources(iface_pairing_gaming_hub PRIVATE iface_pairing_gaming_hub_pulsar.c)

elseif(HARDWARE STREQUAL "QUASAR")
    target_sources(iface_pairing_gaming_hub PRIVATE iface_pairing_gaming_hub_quasar.c)
endif()
