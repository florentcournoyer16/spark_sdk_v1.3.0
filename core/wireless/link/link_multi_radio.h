/** @file link_multi_radio.h
 *  @brief Multi radio module.
 *
 *  @copyright Copyright (C) 2020 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef LINK_MULTI_RADIO_H_
#define LINK_MULTI_RADIO_H_

/* INCLUDES *******************************************************************/
#include <stdint.h>
#include "link_lqi.h"

#ifdef __cplusplus
extern "C" {
#endif

/* TYPES **********************************************************************/
typedef enum multi_radio_mode {
    MULTI_RADIO_MODE_0, /**< Dual radio processing */
    MULTI_RADIO_MODE_1, /**< Single radio processing */
} multi_radio_mode_t;

typedef struct multi_radio {
    lqi_t *radios_lqi;            /**< Radios LQI */
    uint8_t radio_count;          /**< Radio count */
    uint16_t avg_sample_count;    /**< Number of samples to average on */
    uint16_t hysteresis_tenth_db; /**< Hysteresis between radios (only for mode 0)*/
    uint8_t replying_radio;       /**< Replying radio*/
    uint8_t radio_select;         /**< Radio selection for debug, 0 for algorithm, specific radio otherwise */
    multi_radio_mode_t mode;      /**< Chosen multi radio mode */
    uint8_t rssi_threshold;       /**< RSSI threshold (only for mode 1) */
} multi_radio_t;

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Update multi radio module.
 *
 *  @param[in] multi_radio  Multi radio object.
 *  @return None.
 */
void link_multi_radio_update(multi_radio_t *multi_radio);

/** @brief Get replying radio.
 *
 *  @param[in] multi_radio  Multi radio object.
 *  @return Replying radio.
 */
uint8_t link_multi_radio_get_replying_radio(multi_radio_t *multi_radio);

#ifdef __cplusplus
}
#endif
#endif /* LINK_MULTI_RADIO_H_ */
