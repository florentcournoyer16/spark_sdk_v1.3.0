add_executable(validator.elf "")
target_sources(validator.elf
    PRIVATE
        validator.c
)
target_link_libraries(validator.elf PRIVATE iface_validator)
add_custom_command(TARGET validator.elf
    POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary validator.elf validator.bin
)
