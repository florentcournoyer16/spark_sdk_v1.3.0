/** @file  iface_fragmentation.h
 *  @brief This file contains the prototypes of functions configuring the
 *         fragmentation application which calls the underlying BSP functions.
 *
 *  @copyright Copyright (C) 2023 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
               Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef IFACE_FRAGMENTATION_H_
#define IFACE_FRAGMENTATION_H_

/* INCLUDES *******************************************************************/
#include <stdarg.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* MACROS *********************************************************************/
#define ARRAY_SIZE(a) (sizeof(a) / sizeof(*(a)))

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Initialize hardware drivers in the underlying board support package.
 */
void iface_board_init(void);

/** @brief Poll for button presses.
 *
 *  @note Set NULL in place of unused callback.
 *
 *  @param[in] button1_callback  Function to execute when pressing button #1.
 *  @param[in] button2_callback  Function to execute when pressing button #2.
 */
void iface_button_handling(void (*button1_callback)(void), void (*button2_callback)(void));

/** @brief Notify user of the wireless TX connection status.
 */
void iface_tx_conn_status(void);

/** @brief Notify user of the wireless RX connection status.
 */
void iface_rx_conn_status(void);

/** @brief Blocking delay with a 1ms resolution.
 *
 *  @param[in] ms_delay  Delay in milli seconds to wait.
 */
void iface_delay_ms(uint32_t ms_delay);

/** @brief Print a string of characters.
 *
 *  @param[in] string  Null terminated string to print.
 */
void iface_print_string(char *string);

#ifdef __cplusplus
}
#endif

#endif /* IFACE_FRAGMENTATION_H_ */
