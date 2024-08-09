/** @file  iface_audio.h
 *  @brief This file contains the prototypes of functions configuring the
 *         Audio Core which calls the underlying BSP functions.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef IFACE_AUDIO_H_
#define IFACE_AUDIO_H_

/* INCLUDES *******************************************************************/
#include "sac_api.h"
#include "swc_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/* TYPES **********************************************************************/
/** @brief SPARK Wireless Core endpoint instance.
 */
typedef struct ep_swc_instance {
    swc_connection_t *connection; /*!< Wireless connection to use when producing or consuming */
} ep_swc_instance_t;

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Initialize the Audio Core HAL.
 *
 *  @param[out] hal  Audio Core HAL.
 */
void iface_sac_hal_init(sac_hal_t *hal);

/** @brief Initialize Wireless Core audio endpoint interfaces.
 *
 *  @param[out] swc_producer_iface  Wireless Core producer audio endpoint interface.
 *  @param[out] swc_consumer_iface  Wireless Core consumer audio endpoint interface.
 */
void iface_audio_swc_endpoint_init(sac_endpoint_interface_t *swc_producer_iface,
                                   sac_endpoint_interface_t *swc_consumer_iface);

/** @brief Initialize MAX98091 audio endpoint interfaces.
 *
 *  @param[out] max98091_producer_iface  MAX98091 producer audio endpoint interface.
 *  @param[out] max98091_consumer_iface  MAX98091 consumer audio endpoint interface.
 */
void iface_audio_max98091_endpoint_init(sac_endpoint_interface_t *max98091_producer_iface,
                                        sac_endpoint_interface_t *max98091_consumer_iface);

#ifdef __cplusplus
}
#endif

#endif /* IFACE_AUDIO_H_ */
