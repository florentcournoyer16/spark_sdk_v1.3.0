/** @file  link_fallback.h
 *  @brief Link module to handle dynamic settings based on the payload size.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
               Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef LINK_FALLBACK_H_
#define LINK_FALLBACK_H_

/* INCLUDES *******************************************************************/
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* TYPES **********************************************************************/
/** @brief Link fallback module structure.
 */
typedef struct link_fallback {
    uint8_t *threshold;      /*!< Array of fallback threshold, in payload size */
    uint8_t threshold_count; /*!< Number of fallback threshold */
} link_fallback_t;

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Initialize the link fallback module.
 *
 *  @param[in] link_fallback   Link fallback instance.
 *  @param[in] threshold        Array of thresholds.
 *  @param[in] threshold_count  Number of thresholds.
 */
void link_fallback_init(link_fallback_t *link_fallback, uint8_t *threshold, uint8_t threshold_count);

/** @brief Get the current fallback channel index based on the payload size.
 *
 *  @param[in] link_fallback  Link fallback instance.
 *  @param[in] payload_size   Current payload size.
 *  @return Channel index.
 */
uint8_t link_fallback_get_index(link_fallback_t *link_fallback, uint8_t payload_size);

#ifdef __cplusplus
}
#endif

#endif /* LINK_FALLBACK_H_ */
