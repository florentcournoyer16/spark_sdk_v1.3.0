/** @file  iface_pairing_basic.h
 *  @brief This file contains the prototypes of functions configuring the
 *         pairing basic application which calls the underlying BSP functions.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef IFACE_PAIRING_BASIC_H_
#define IFACE_PAIRING_BASIC_H_

/* INCLUDES *******************************************************************/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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

/** @brief Notify user of payload present in frame.
 */
void iface_payload_received_status(void);

/** @brief Notify user of no payload present in frame.
 */
void iface_empty_payload_received_status(void);

/** @brief Turn on all LEDs.
 */
void iface_led_all_on(void);

/** @brief Turn off all LEDs.
 */
void iface_led_all_off(void);

/** @brief Toggle all LEDs.
 */
void iface_led_all_toggle(void);

/** @brief Blink LED twice.
 */
void iface_notify_enter_pairing(void);

/** @brief Blink all LEDs twice.
 */
void iface_notify_not_paired(void);

/** @brief Turn on and turn off the LEDs in a knight rider fashion.
 */
void iface_notify_pairing_successful(void);

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

#endif /* IFACE_PAIRING_BASIC_H_ */
