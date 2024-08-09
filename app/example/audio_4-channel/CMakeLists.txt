add_executable(audio_4-channel_coordinator.elf "")
target_sources(audio_4-channel_coordinator.elf
    PRIVATE
        audio_4-channel_coord.c
        swc_cfg_coord.h
)
target_link_libraries(audio_4-channel_coordinator.elf PRIVATE audio_core iface_core iface_audio_4-channel max98091_driver wps)
add_custom_command(TARGET audio_4-channel_coordinator.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary audio_4-channel_coordinator.elf audio_4-channel_coordinator.bin
)

add_executable(audio_4-channel_node1.elf "")
target_sources(audio_4-channel_node1.elf
    PRIVATE
        audio_4-channel_node.c
        swc_cfg_node1.h
)
target_link_libraries(audio_4-channel_node1.elf PRIVATE audio_core iface_core iface_audio_4-channel max98091_driver wps)
target_compile_definitions(audio_4-channel_node1.elf PRIVATE NODE1)
add_custom_command(TARGET audio_4-channel_node1.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary audio_4-channel_node1.elf audio_4-channel_node1.bin
)

add_executable(audio_4-channel_node2.elf "")
target_sources(audio_4-channel_node2.elf
    PRIVATE
        audio_4-channel_node.c
        swc_cfg_node2.h
)
target_link_libraries(audio_4-channel_node2.elf PRIVATE audio_core iface_core iface_audio_4-channel max98091_driver wps)
target_compile_definitions(audio_4-channel_node2.elf PRIVATE NODE2)
add_custom_command(TARGET audio_4-channel_node2.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary audio_4-channel_node2.elf audio_4-channel_node2.bin
)

add_executable(audio_4-channel_node3.elf "")
target_sources(audio_4-channel_node3.elf
    PRIVATE
        audio_4-channel_node.c
        swc_cfg_node3.h
)
target_link_libraries(audio_4-channel_node3.elf PRIVATE audio_core iface_core iface_audio_4-channel max98091_driver wps)
target_compile_definitions(audio_4-channel_node3.elf PRIVATE NODE3)
add_custom_command(TARGET audio_4-channel_node3.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary audio_4-channel_node3.elf audio_4-channel_node3.bin
)

add_executable(audio_4-channel_node4.elf "")
target_sources(audio_4-channel_node4.elf
    PRIVATE
        audio_4-channel_node.c
        swc_cfg_node4.h
)
target_link_libraries(audio_4-channel_node4.elf PRIVATE audio_core iface_core iface_audio_4-channel max98091_driver wps)
target_compile_definitions(audio_4-channel_node4.elf PRIVATE NODE4)
add_custom_command(TARGET audio_4-channel_node4.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary audio_4-channel_node4.elf audio_4-channel_node4.bin
)
