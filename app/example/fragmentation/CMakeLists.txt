add_executable(fragmentation_coordinator.elf "")
target_sources(fragmentation_coordinator.elf
    PRIVATE
        frag_coord.c
        swc_cfg_coord.h
)
target_link_libraries(fragmentation_coordinator.elf PRIVATE iface_core iface_fragmentation pseudo_data)
add_custom_command(TARGET fragmentation_coordinator.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary fragmentation_coordinator.elf fragmentation_coordinator.bin
)

add_executable(fragmentation_node.elf "")
target_sources(fragmentation_node.elf
    PRIVATE
        frag_node.c
        swc_cfg_node.h
)
target_link_libraries(fragmentation_node.elf PRIVATE iface_core iface_fragmentation pseudo_data)
add_custom_command(TARGET fragmentation_node.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary fragmentation_node.elf fragmentation_node.bin
)
