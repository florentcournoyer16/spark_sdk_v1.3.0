add_executable(audio_mixer_coordinator.elf "")
target_sources(audio_mixer_coordinator.elf
    PRIVATE
        audio_mixer_coord.c
        swc_cfg_coord.h
)
target_link_libraries(audio_mixer_coordinator.elf PRIVATE audio_core iface_core iface_audio_mixer max98091_driver wps)
add_custom_command(TARGET audio_mixer_coordinator.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary audio_mixer_coordinator.elf audio_mixer_coordinator.bin
)

add_executable(audio_mixer_node1.elf "")
target_sources(audio_mixer_node1.elf
    PRIVATE
        audio_mixer_node.c
        swc_cfg_node1.h
)
target_link_libraries(audio_mixer_node1.elf PRIVATE audio_core iface_core iface_audio_mixer max98091_driver wps)
target_compile_definitions(audio_mixer_node1.elf PRIVATE NODE1)
add_custom_command(TARGET audio_mixer_node1.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary audio_mixer_node1.elf audio_mixer_node1.bin
)

add_executable(audio_mixer_node2.elf "")
target_sources(audio_mixer_node2.elf
    PRIVATE
        audio_mixer_node.c
        swc_cfg_node2.h
)
target_link_libraries(audio_mixer_node2.elf PRIVATE audio_core iface_core iface_audio_mixer max98091_driver wps)
target_compile_definitions(audio_mixer_node2.elf PRIVATE NODE2)
add_custom_command(TARGET audio_mixer_node2.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary audio_mixer_node2.elf audio_mixer_node2.bin
)
