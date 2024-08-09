/** @file  link_fallback.c
 *  @brief Link module to handle dynamic settings based on the payload size.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
               Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "link_fallback.h"

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
void link_fallback_init(link_fallback_t *link_fallback, uint8_t *threshold, uint8_t threshold_count)
{
    link_fallback->threshold       = threshold;
    link_fallback->threshold_count = threshold_count;
}

uint8_t link_fallback_get_index(link_fallback_t *link_fallback, uint8_t payload_size)
{
    for (uint8_t i = 0; i < link_fallback->threshold_count; i++) {
        if (payload_size <= link_fallback->threshold[i]) {
            return i + 1;
        }
    }
    return 0;
}
