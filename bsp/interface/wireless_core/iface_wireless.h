/** @file  iface_wireless.h
 *  @brief This file contains the prototypes of functions configuring the
 *         wireless core which calls the underlying BSP functions.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef IFACE_WIRELESS_H_
#define IFACE_WIRELESS_H_

/* INCLUDES *******************************************************************/
#include "swc_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/* PRIVATE FUNCTION PROTOTYPES ************************************************/
/** @brief Initialize the Wireless Core HAL.
 *
 *  @param[out] hal  Wireless Core HAL.
 */
void iface_swc_hal_init(swc_hal_t *hal);

/** @brief Link the three Wireless Core handlers to the right peripheral.
 */
void iface_swc_handlers_init(void);

#if (WPS_RADIO_COUNT == 2)
/** @brief Initialize the MCU timer needed for dual radio operations.
 */
void iface_swc_dual_radio_timer_init(void);
#endif

#ifdef __cplusplus
}
#endif

#endif /* IFACE_WIRELESS_H_ */
