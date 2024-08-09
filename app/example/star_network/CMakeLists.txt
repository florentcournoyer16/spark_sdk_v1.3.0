add_executable(star_network_coordinator.elf "")
target_sources(star_network_coordinator.elf
    PRIVATE
        star_network_coord.c
        swc_cfg_coord.h
)
target_link_libraries(star_network_coordinator.elf PRIVATE iface_core iface_star_network wps)
add_custom_command(TARGET star_network_coordinator.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary star_network_coordinator.elf star_network_coordinator.bin
)

add_executable(star_network_node1.elf "")
target_sources(star_network_node1.elf
    PRIVATE
        star_network_node.c
        swc_cfg_node1.h
)
target_link_libraries(star_network_node1.elf PRIVATE iface_core iface_star_network wps)
target_compile_definitions(star_network_node1.elf PRIVATE NODE1)
add_custom_command(TARGET star_network_node1.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary star_network_node1.elf star_network_node1.bin
)

add_executable(star_network_node2.elf "")
target_sources(star_network_node2.elf
    PRIVATE
        star_network_node.c
        swc_cfg_node2.h
)
target_link_libraries(star_network_node2.elf PRIVATE iface_core iface_star_network wps)
target_compile_definitions(star_network_node2.elf PRIVATE NODE2)
add_custom_command(TARGET star_network_node2.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary star_network_node2.elf star_network_node2.bin
)
