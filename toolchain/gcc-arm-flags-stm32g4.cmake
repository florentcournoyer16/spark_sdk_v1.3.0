set(CMAKE_CXX_FLAGS
    "-O2 -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=soft -fmessage-length=0 -ffunction-sections -fdata-sections -fno-common -fno-omit-frame-pointer -fsingle-precision-constant -Wdouble-promotion -fno-move-loop-invariants -fno-stack-protector -Wall -Wextra -Wshadow=compatible-local"
    CACHE STRING
    "Flags used by the C++ compiler during all build types."
    FORCE
)

set(CMAKE_C_FLAGS
    "-O2 -mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=soft -fmessage-length=0 -ffunction-sections -fdata-sections -fno-common -fno-omit-frame-pointer -fsingle-precision-constant -Wdouble-promotion -fno-move-loop-invariants -fno-stack-protector -Wall -Wextra -Wshadow=compatible-local -Werror=implicit-function-declaration"
    CACHE STRING
    "Flags used by the C compiler during all build types."
    FORCE
)

set(CMAKE_ASM_FLAGS
    "-mcpu=cortex-m4 -march=armv7e-m -mthumb -mfloat-abi=soft -fmessage-length=0 -ffunction-sections -fdata-sections -fno-move-loop-invariants -fno-stack-protector -Wall -Wextra -Wshadow=compatible-local"
    CACHE STRING
    "Flags used by the C assembler during all build types."
    FORCE
)

set(CMAKE_C_FLAGS_DEBUG
    "-g3 -ffreestanding -DDEBUGGING"
    CACHE INTERNAL
    "c compiler flags debug"
)

set(CMAKE_ASM_FLAGS_DEBUG
    "-g3"
    CACHE INTERNAL
    "asm compiler flags debug"
)

set(CMAKE_EXE_LINKER_FLAGS
    "-specs=nosys.specs -specs=nano.specs -u _printf_float -Xlinker -Map=output.map -Wl,--gc-sections -Wl,--print-memory-usage -fno-exceptions -fno-rtti -lm -L${STM32_MCU_CCMRAM_LINKER_SCRIPT_PATH} -T${STM32G4_LINKER_SCRIPT_PATH}"
    CACHE STRING
    "Flags used by the linker during all build types."
    FORCE
)
