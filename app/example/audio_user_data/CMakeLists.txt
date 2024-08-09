add_executable(audio_user_data_coordinator.elf "")
target_sources(audio_user_data_coordinator.elf
    PRIVATE
        audio_user_data_coord.c
        swc_cfg_coord.h
)
target_link_libraries(audio_user_data_coordinator.elf PRIVATE audio_core iface_core iface_audio_user_data max98091_driver wps)
add_custom_command(TARGET audio_user_data_coordinator.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary audio_user_data_coordinator.elf audio_user_data_coordinator.bin
)

add_executable(audio_user_data_node.elf "")
target_sources(audio_user_data_node.elf
    PRIVATE
        audio_user_data_node.c
        swc_cfg_node.h
)
target_link_libraries(audio_user_data_node.elf PRIVATE audio_core iface_core iface_audio_user_data max98091_driver wps)
add_custom_command(TARGET audio_user_data_node.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary audio_user_data_node.elf audio_user_data_node.bin
)
