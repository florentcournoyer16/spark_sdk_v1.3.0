add_executable(connection_priority_coordinator.elf "")
target_sources(connection_priority_coordinator.elf
    PRIVATE
        connection_priority_coord.c
        swc_cfg_coord.h
)
target_link_libraries(connection_priority_coordinator.elf PRIVATE iface_core iface_connection_priority wps)
add_custom_command(TARGET connection_priority_coordinator.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary connection_priority_coordinator.elf connection_priority_coordinator.bin
)

add_executable(connection_priority_node.elf "")
target_sources(connection_priority_node.elf
    PRIVATE
        connection_priority_node.c
        swc_cfg_node.h
)
target_link_libraries(connection_priority_node.elf PRIVATE iface_core iface_connection_priority wps)
target_compile_definitions(connection_priority_node.elf PRIVATE NODE)
add_custom_command(TARGET connection_priority_node.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary connection_priority_node.elf connection_priority_node.bin
)
