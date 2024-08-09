add_executable(audio_high_resolution_coordinator.elf "")
target_sources(audio_high_resolution_coordinator.elf
    PRIVATE
        audio_high_resolution_coord.c
        swc_cfg_coord.h
)
target_link_libraries(audio_high_resolution_coordinator.elf PRIVATE audio_core iface_core iface_audio_high_resolution max98091_driver wps)
add_custom_command(TARGET audio_high_resolution_coordinator.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary audio_high_resolution_coordinator.elf audio_high_resolution_coordinator.bin
)

add_executable(audio_high_resolution_node.elf "")
target_sources(audio_high_resolution_node.elf
    PRIVATE
        audio_high_resolution_node.c
        swc_cfg_node.h
)
target_link_libraries(audio_high_resolution_node.elf PRIVATE audio_core iface_core iface_audio_high_resolution max98091_driver wps)
add_custom_command(TARGET audio_high_resolution_node.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary audio_high_resolution_node.elf audio_high_resolution_node.bin
)
