#!/bin/bash
sourceWithoutArgs() {
    local fileToSource="$1"
    shift
    source "$fileToSource"
}

SOURCE=$( realpath $( dirname -- "${BASH_SOURCE[0]}"; ); )
sourceWithoutArgs $SOURCE/../script/activate.sh
arm-none-eabi-gdb "$@"
