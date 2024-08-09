/** @file  iface_star_network.h
 *  @brief This file contains the prototypes of functions configuring the
 *         star network application which calls the underlying BSP functions
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef IFACE_STAR_NETWORK_H_
#define IFACE_STAR_NETWORK_H_

/* INCLUDES *******************************************************************/
#include <stdarg.h>
#include <stdint.h>
#include "sr_api_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* MACROS *********************************************************************/
#define ARRAY_SIZE(a) (sizeof(a) / sizeof(*(a)))

/* TYPES **********************************************************************/
/** @brief Board's User button enumeration.
 */
typedef enum star_network_btn {
    BUTTON_A,
    BUTTON_B
} star_network_btn_t;

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Initialize hardware drivers in the underlying board support package.
 */
void iface_board_init(void);

/** @brief Blocking delay with a 1 ms resolution.
 *
 *  @param[in] ms_delay  Delay in milliseconds to wait.
 */
void iface_delay(uint32_t ms_delay);

/** @brief Read the input from the user button.
 *
 *  @param[in] button  User button.
 *
 *  @return true if button is pressed.
 */
bool iface_read_button_status(star_network_btn_t button);

/** @brief Print characters through USB.
 *
 *  @param[in] fmt  Pointer to the characters to be printed.
 *  @param[in] ...  Variable argument list.
 */
void iface_usb_printf(const char *fmt, ...);

/** @brief Notify user of payload present in frame.
 */
void iface_payload_sent_status(void);

/** @brief Notify user of no payload present in frame.
 */
void iface_empty_payload_sent_status(void);

/** @brief Notify user of payload present in frame.
 */
void iface_payload_received_status(void);

/** @brief Notify user of no payload present in frame.
 */
void iface_empty_payload_received_status(void);

#ifdef __cplusplus
}
#endif

#endif /* IFACE_STAR_NETWORK_H_ */
