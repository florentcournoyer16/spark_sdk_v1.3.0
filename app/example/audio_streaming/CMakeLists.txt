add_executable(audio_streaming_coordinator.elf "")
target_sources(audio_streaming_coordinator.elf
    PRIVATE
        audio_streaming_coord.c
        swc_cfg_coord.h
)
target_link_libraries(audio_streaming_coordinator.elf PRIVATE audio_core iface_core iface_audio_streaming max98091_driver wps)
add_custom_command(TARGET audio_streaming_coordinator.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary audio_streaming_coordinator.elf audio_streaming_coordinator.bin
)

add_executable(audio_streaming_node.elf "")
target_sources(audio_streaming_node.elf
    PRIVATE
        audio_streaming_node.c
        swc_cfg_node.h
)
target_link_libraries(audio_streaming_node.elf PRIVATE audio_core iface_core iface_audio_streaming max98091_driver wps)
add_custom_command(TARGET audio_streaming_node.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary audio_streaming_node.elf audio_streaming_node.bin
)
