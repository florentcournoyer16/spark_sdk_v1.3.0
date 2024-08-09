/** @file  iface_connection_priority.h
 *  @brief This file contains the prototypes of functions used by the
 *         connection priority application to abstract the underlying BSP functions.
 *
 *  @copyright Copyright (C) 2023 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef IFACE_CONNECTION_PRIORITY_H_
#define IFACE_CONNECTION_PRIORITY_H_

/* INCLUDES *******************************************************************/
#include <stdint.h>
#include "sr_api_hal.h"

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

/** @brief Initialize and set the timer 1 period.
 *
 *  @param[in] period_us  Timer period in microseconds.
 */
void iface_packet_rate_timer1_init(uint32_t period_us);

/** @brief Set the timer 1 callback.
 *
 *  @param[in] callback  Callback when timer expires.
 */
void iface_packet_rate_set_timer1_callback(void (*callback)(void));

/** @brief Start the timer 1.
 */
void iface_packet_rate_timer1_start(void);

/** @brief Initialize and set the timer 2 period.
 *
 *  @param[in] period_us  Timer period in microseconds.
 */
void iface_packet_rate_timer2_init(uint32_t period_us);

/** @brief Set the timer 2 callback.
 *
 *  @param[in] callback  Callback when timer expires.
 */
void iface_packet_rate_set_timer2_callback(void (*callback)(void));

/** @brief Start the timer 2.
 */
void iface_packet_rate_timer2_start(void);

/** @brief Initialize and set the stats timer period.
 *
 *  @param[in] period_ms  Timer period in milliseconds.
 */
void iface_stats_timer_init(uint32_t period_ms);

/** @brief Set the stats timer callback.
 *
 *  @param[in] callback  Callback when timer expires.
 */
void iface_stats_set_timer_callback(void (*callback)(void));

/** @brief Start the stats timer.
 */
void iface_stats_timer_start(void);

/** @brief Print string.
 *
 *  @param[in] string  Null terminated string to print.
 */
void iface_print_string(char *string);

#ifdef __cplusplus
}
#endif

#endif /* IFACE_CONNECTION_PRIORITY_H_ */
