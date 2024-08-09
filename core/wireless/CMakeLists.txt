cmake_minimum_required(VERSION 3.18)

zephyr_library_named(wps)
set(TRANSCEIVER "SR1000" CACHE STRING "Set SR1000 transceiver model has default if the variable wasn't initialized.")

message("WPS Core being compiled for a number of " ${WPS_RADIO_COUNT} " " ${TRANSCEIVER} " transceiver")

target_compile_definitions(wps PUBLIC ${ACCESS_ADV_ERR_CHECK_EN} ${SPARK_WPS_CFG_FILE_EXISTS} ${TRANSCEIVER})

add_subdirectory(api)
add_subdirectory(link)
add_subdirectory(pairing)
add_subdirectory(phy)
add_subdirectory(protocol_stack)
add_subdirectory(transceiver)
add_subdirectory(xlayer)

target_link_libraries(wps PUBLIC buffer memory queue)
