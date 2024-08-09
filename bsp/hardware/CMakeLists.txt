#zephyr_library_named(hardware)
set(BSP_PATH ${CMAKE_CURRENT_LIST_DIR}/usens CACHE PATH "Path to usens source files for use by the whole project.")

message(${PROJECT_SOURCE_DIR})
message(${CMAKE_CURRENT_LIST_DIR})

#target_sources(hardware
#    PRIVATE
#        ${BSP_PATH}/usens_radio.c
#        ${BSP_PATH}/usens_timer.c
#    PUBLIC
#        ${BSP_PATH}/usens_radio.h
#        ${BSP_PATH}/usens_timer.h
#)

target_include_directories(app 
    PUBLIC  
        ${BSP_PATH}
        ${NRF_HW_CONGIG_PATH}
)

target_sources(app 
  PRIVATE
    ${BSP_PATH}/usens_it.c
    ${BSP_PATH}/usens_radio.c
    ${BSP_PATH}/usens_timer.c
    ${BSP_PATH}/usens_base.c
  PUBLIC
    ${BSP_PATH}/usens_it.h
    ${BSP_PATH}/usens_radio.h
    ${BSP_PATH}/usens_timer.h
    ${BSP_PATH}/usens_base.h
)