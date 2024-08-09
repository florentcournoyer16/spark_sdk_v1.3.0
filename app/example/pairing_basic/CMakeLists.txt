add_executable(pairing_basic_coordinator.elf "")
target_sources(pairing_basic_coordinator.elf
    PRIVATE
        pairing_basic_coord.c
        swc_cfg_coord.h
)
target_compile_definitions(pairing_basic_coordinator.elf PUBLIC COORDINATOR)
target_link_libraries(pairing_basic_coordinator.elf PRIVATE iface_core iface_pairing_basic wps)
add_custom_command(TARGET pairing_basic_coordinator.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary pairing_basic_coordinator.elf pairing_basic_coordinator.bin
)

add_executable(pairing_basic_node.elf "")
target_sources(pairing_basic_node.elf
    PRIVATE
        pairing_basic_node.c
        swc_cfg_node.h
)
target_compile_definitions(pairing_basic_node.elf PUBLIC NODE)
target_link_libraries(pairing_basic_node.elf PRIVATE iface_core iface_pairing_basic wps)
add_custom_command(TARGET pairing_basic_node.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary pairing_basic_node.elf pairing_basic_node.bin
)
