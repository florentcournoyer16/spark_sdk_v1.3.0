add_executable(audio_compression_coordinator.elf "")
target_sources(audio_compression_coordinator.elf
    PRIVATE
        audio_compression_coord.c
        swc_cfg_coord.h
)
target_link_libraries(audio_compression_coordinator.elf PRIVATE audio_core iface_core iface_audio_compression max98091_driver wps)
add_custom_command(TARGET audio_compression_coordinator.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary audio_compression_coordinator.elf audio_compression_coordinator.bin
)

add_executable(audio_compression_node.elf "")
target_sources(audio_compression_node.elf
    PRIVATE
        audio_compression_node.c
        swc_cfg_node.h
)
target_link_libraries(audio_compression_node.elf PRIVATE audio_core iface_core iface_audio_compression max98091_driver wps)
add_custom_command(TARGET audio_compression_node.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary audio_compression_node.elf audio_compression_node.bin
)
