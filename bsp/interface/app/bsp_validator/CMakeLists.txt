zephyr_library_named(iface_validator)set_target_properties(iface_validator PROPERTIES LINKER_LANGUAGE C)

target_link_libraries(iface_validator PUBLIC hardware)
target_include_directories(iface_validator
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${IFACE_LIB_CRITICAL_SECTION_OPT_PATH}
)

if (HARDWARE STREQUAL "EVK")
    target_sources(iface_validator
    PRIVATE
        iface_validator_evk.c
    )

elseif(HARDWARE STREQUAL "PULSAR")
    target_sources(iface_validator
    PRIVATE
        iface_validator_pulsar.c
    )

elseif(HARDWARE STREQUAL "QUASAR")
    target_sources(iface_validator
    PRIVATE
        iface_validator_quasar.c
    )
endif()
