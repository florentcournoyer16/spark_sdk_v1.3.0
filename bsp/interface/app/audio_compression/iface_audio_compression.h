/** @file  iface_audio_compression.h
 *  @brief This file contains the prototypes of functions configuring the
 *         audio compression application which calls the underlying BSP functions.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef IFACE_AUDIO_COMPRESSION_H_
#define IFACE_AUDIO_COMPRESSION_H_

#ifdef __cplusplus
extern "C" {
#endif

/* INCLUDES *******************************************************************/
#include <stdbool.h>
#include <stdint.h>

/* MACROS *********************************************************************/
#define ARRAY_SIZE(a) (sizeof(a) / sizeof(*(a)))

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

/** @brief Notify user of the fallback status.
 */
void iface_fallback_status(bool on);

/** @brief Initialize and set the audio process timer period.
 *
 *  @param[in] period_us  Timer period in us.
 */
void iface_audio_process_timer_init(uint32_t period_us);

/** @brief Set the audio process timer callback.
 *
 *  @param[in] callback  Callback when timer expires.
 */
void iface_audio_process_set_timer_callback(void (*callback)(void));

/** @brief Start the audio process timer.
 */
void iface_audio_process_timer_start(void);

/** @brief Initialize and set the stats timer period.
 *
 *  @param[in] period_ms  Timer period in ms.
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

#endif /* IFACE_AUDIO_COMPRESSION_H_ */
