/** @file  iface_hello_world.h
 *  @brief This file contains the prototypes of functions configuring the
 *         hello_world application which calls the underlying BSP functions.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef IFACE_HELLO_WORLD_H_
#define IFACE_HELLO_WORLD_H_

/* INCLUDES *******************************************************************/
#include <stdarg.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* MACROS *********************************************************************/
#define ARRAY_SIZE_SPARK(a) (sizeof(a) / sizeof(*(a)))

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Initialize hardware drivers in the underlying board support package.
 */
void iface_board_init(void);

/** @brief Notify user of the wireless TX connection status.
 */
void iface_tx_conn_status(void);

/** @brief Notify user of the wireless RX connection status.
 */
void iface_rx_conn_status(void);

/** @brief Blocking delay with a 1ms resolution.
 *
 *  @param[in] ms_delay  Delay in milliseconds to wait.
 */
void iface_delay(uint32_t ms_delay);

/** @brief Print a string of characters.
 *
 *  @param[in] string  Null terminated string to print.
 */
void iface_print_string(char *string);

#ifdef __cplusplus
}
#endif

#endif /* IFACE_HELLO_WORLD_H_ */
