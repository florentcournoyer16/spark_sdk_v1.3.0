add_executable(pairing_gaming_hub_dongle.elf "")
target_sources(pairing_gaming_hub_dongle.elf
    PRIVATE
        pairing_gaming_hub_dongle.c
        swc_cfg_dongle.h
)
target_compile_definitions(pairing_gaming_hub_dongle.elf PUBLIC COORDINATOR)
target_link_libraries(pairing_gaming_hub_dongle.elf PRIVATE audio_core iface_core iface_pairing_gaming_hub max98091_driver wps)
add_custom_command(TARGET pairing_gaming_hub_dongle.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary pairing_gaming_hub_dongle.elf pairing_gaming_hub_dongle.bin
)

add_executable(pairing_gaming_hub_headset.elf "")
target_sources(pairing_gaming_hub_headset.elf
    PRIVATE
        pairing_gaming_hub_headset.c
        swc_cfg_headset.h
)
target_compile_definitions(pairing_gaming_hub_headset.elf PUBLIC NODE1)
target_link_libraries(pairing_gaming_hub_headset.elf PRIVATE audio_core iface_core iface_pairing_gaming_hub max98091_driver wps)
add_custom_command(TARGET pairing_gaming_hub_headset.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary pairing_gaming_hub_headset.elf pairing_gaming_hub_headset.bin
)

add_executable(pairing_gaming_hub_mouse.elf "")
target_sources(pairing_gaming_hub_mouse.elf
    PRIVATE
        pairing_gaming_hub_mouse.c
        swc_cfg_mouse.h
)
target_compile_definitions(pairing_gaming_hub_mouse.elf PUBLIC NODE2)
target_link_libraries(pairing_gaming_hub_mouse.elf PRIVATE iface_core iface_pairing_gaming_hub wps)
add_custom_command(TARGET pairing_gaming_hub_mouse.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary pairing_gaming_hub_mouse.elf pairing_gaming_hub_mouse.bin
)

add_executable(pairing_gaming_hub_keyboard.elf "")
target_sources(pairing_gaming_hub_keyboard.elf
    PRIVATE
        pairing_gaming_hub_keyboard.c
        swc_cfg_mouse.h
)
target_compile_definitions(pairing_gaming_hub_keyboard.elf PUBLIC NODE3)
target_link_libraries(pairing_gaming_hub_keyboard.elf PRIVATE iface_core iface_pairing_gaming_hub wps)
add_custom_command(TARGET pairing_gaming_hub_keyboard.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary pairing_gaming_hub_keyboard.elf pairing_gaming_hub_keyboard.bin
)
