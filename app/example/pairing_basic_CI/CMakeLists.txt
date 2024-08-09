set(CORTICAL_IMPLANT_PATH ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "")

zephyr_library_named(cortical_implant)
if (MYPL STREQUAL "COORD")
    target_sources(cortical_implant
    PRIVATE
        pairing_basic_coord.c
        swc_cfg_coord.h
    PUBLIC
        pairing_basic_coord.h
    )
elseif (MYPL STREQUAL "NODE")
    target_sources(cortical_implant
    PRIVATE
        pairing_basic_node.c
        swc_cfg_node.h
    PUBLIC
        pairing_basic_node.h
    )
endif()

target_link_libraries(cortical_implant PRIVATE iface_core iface_cortical_implant wps)