zephyr_library_named(iface_cortical_implant)
set_target_properties(iface_cortical_implant PROPERTIES LINKER_LANGUAGE C)

target_include_directories(iface_cortical_implant 
    PUBLIC 
        ${CMAKE_CURRENT_LIST_DIR} 
    PRIVATE
        ${BSP_PATH})

target_link_libraries(iface_cortical_implant PRIVATE app)

target_sources(iface_cortical_implant 
    PRIVATE 
        iface_cortical_implant_nrf.c
    PUBLIC
        iface_cortical_implant.h)


