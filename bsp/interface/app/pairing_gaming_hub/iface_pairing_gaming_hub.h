/** @file  iface_pairing_gaming_hub.h
 *  @brief This file contains the prototypes of functions configuring the
 *         pairing gaming hub application which calls the underlying BSP functions.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef IFACE_PAIRING_GAMING_HUB_H_
#define IFACE_PAIRING_GAMING_HUB_H_

/* INCLUDES *******************************************************************/
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Initialize hardware drivers in the underlying board support package.
 */
void iface_board_init(void);

/** @brief Initialize the coordinator's SAI peripheral.
 *
 *  Configure the serial audio interface to MONO or Stereo
 */
void iface_audio_coord_init(void);

/** @brief Initialize the node's SAI peripheral.
 *
 *  Configure the serial audio interface to MONO or Stereo
 */
void iface_audio_node_init(void);

/** @brief Set the serial audio interface transfer complete callbacks
 *
 *  @note Set NULL in place of unused callback.
 *
 *  @param tx_callback  Audio i2s tx complete callback
 *  @param rx_callback  Audio i2s rx complete callback
 */
void iface_set_sai_complete_callback(void (*tx_callback)(void), void (*rx_callback)(void));

/** @brief One shot rising button handling.
 *
 *  This function handles the one-shot rising behavior of a button. It ensures that the associated button
 *  callback function is called only once when a rising edge is detected.
 *
 *  @note Set NULL in place of unused callback.
 *
 *  @param[in] button1_callback  Function to execute when pressing button #1.
 *  @param[in] button2_callback  Function to execute when pressing button #2.
 */
void iface_button_handling_osr(void (*button1_callback)(void), void (*button2_callback)(void));

/** @brief One shot falling button handling.
 *
 *  This function handles the one-shot falling behavior of a button. It ensures that the associated button
 *  callback function is called only once when a falling edge is detected.
 *
 *  @note Set NULL in place of unused callback.
 *
 *  @param[in] button1_callback  Function to execute when releasing button #1.
 *  @param[in] button2_callback  Function to execute when releasing button #2.
 */
void iface_button_handling_osf(void (*button1_callback)(void), void (*button2_callback)(void));

/** @brief Notify user of the wireless TX connection status.
 */
void iface_tx_conn_status(void);

/** @brief Reset the TX connection LED status when the device is not connected.
 */
void iface_reset_tx_conn_status(void);

/** @brief Notify user of the wireless RX connection status.
 */
void iface_rx_conn_status(void);

/** @brief Reset the RX connection LED status when the devices is not connected.
 */
void iface_reset_rx_conn_status(void);

/** @brief Notify user of keyboard button press to latch.
 */
void iface_keyboard_button_latched_status(void);

/** @brief Notify user of keyboard button press to unlatch.
 */
void iface_keyboard_button_unlatched_status(void);

/** @brief Notify user of mouse press toggle on.
 */
void iface_mouse_button_latched_status(void);

/** @brief Notify user of mouse press toggle off.
 */
void iface_mouse_button_unlatched_status(void);

/** @brief Notify user of payload present in frame.
 */
void iface_payload_sent_status(void);

/** @brief Notify user of no payload present in frame.
 */
void iface_empty_payload_sent_status(void);

/** @brief Turn on all LEDs.
 */
void iface_led_all_on(void);

/** @brief Turn off all LEDs.
 */
void iface_led_all_off(void);

/** @brief Toggle all LEDs.
 */
void iface_led_all_toggle(void);

/** @brief Blink a LED twice.
 */
void iface_notify_enter_pairing(void);

/** @brief Blink all LEDs twice.
 */
void iface_notify_not_paired(void);

/** @brief Turn on and turn off the LEDs in a knight rider fashion.
 */
void iface_notify_pairing_successful(void);

#ifdef __cplusplus
}
#endif

#endif /* IFACE_PAIRING_GAMING_HUB_H_ */
