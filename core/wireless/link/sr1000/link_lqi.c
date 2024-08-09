/** @file link_lqi.c
 *  @brief LQI module.
 *
 *  @copyright Copyright (C) 2020 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "link_lqi.h"

/* PUBLIC FUNCTIONS ***********************************************************/
void link_lqi_init(lqi_t *lqi, lqi_mode_t mode)
{
    memset(lqi, 0, sizeof(lqi_t));
    lqi->mode = mode;
}

uint16_t link_lqi_get_avg_rssi_tenth_db(lqi_t *lqi)
{
    switch (lqi->mode) {
    case LQI_MODE_0:
        if (!lqi->total_count) {
            return 0;
        }
        return (lqi->rssi_total_tenth_db / lqi->total_count);
        break;
    case LQI_MODE_1:
        if (!lqi->received_count) {
            return 0;
        }
        return (lqi->rssi_total_tenth_db / lqi->received_count);
        break;
    default:
        if (!lqi->total_count) {
            return 0;
        }
        return (lqi->rssi_total_tenth_db / lqi->total_count);
    }
}

uint16_t link_lqi_get_avg_rnsi_tenth_db(lqi_t *lqi)
{
    switch (lqi->mode) {
    case LQI_MODE_0:
        if (!lqi->total_count) {
            return 0;
        }
        return (lqi->rnsi_total_tenth_db / lqi->total_count);
        break;
    case LQI_MODE_1:
        if (!lqi->received_count) {
            return 0;
        }
        return (lqi->rnsi_total_tenth_db / lqi->received_count);
        break;
    default:
        if (!lqi->total_count) {
            return 0;
        }
        return (lqi->rnsi_total_tenth_db / lqi->total_count);
    }
}
