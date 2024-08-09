add_executable(ranging_coordinator.elf "")
target_sources(ranging_coordinator.elf
    PRIVATE
        ranging_coord.c
        swc_cfg_coord.h
)
target_link_libraries(ranging_coordinator.elf PRIVATE ranging_core iface_core iface_ranging)
add_custom_command(TARGET ranging_coordinator.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary ranging_coordinator.elf ranging_coordinator.bin
)

add_executable(ranging_node.elf "")
target_sources(ranging_node.elf
    PRIVATE
        ranging_node.c
        swc_cfg_node.h
)
target_link_libraries(ranging_node.elf PRIVATE ranging_core iface_core iface_ranging)
add_custom_command(TARGET ranging_node.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary ranging_node.elf ranging_node.bin
)
