target_sources(wps
    PRIVATE
        api/pairing_api.c
        api/pairing_error.c
        communication/pairing_address.c
        communication/pairing_event.c
        utilities/pairing_security.c
        utilities/pairing_timer.c
        state/pairing_state.c
        state/pairing_state_coordinator.c
        state/pairing_state_machine.c
        state/pairing_state_node.c
        wireless/pairing_wireless.c
    PUBLIC
        api/pairing_api.h
        api/pairing_error.h
        communication/pairing_address.h
        communication/pairing_event.h
        communication/pairing_message.h
        utilities/pairing_security.h
        utilities/pairing_timer.h
        state/pairing_state.h
        state/pairing_state_coordinator.h
        state/pairing_state_machine.h
        state/pairing_state_node.h
        wireless/pairing_wireless.h
)

target_include_directories(wps
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/api
        ${CMAKE_CURRENT_LIST_DIR}/communication
        ${CMAKE_CURRENT_LIST_DIR}/state
        ${CMAKE_CURRENT_LIST_DIR}/utilities
        ${CMAKE_CURRENT_LIST_DIR}/wireless
)
