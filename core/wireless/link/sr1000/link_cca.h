/** @file link_cca.h
 *  @brief Clear Channel Assessment module.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef LINK_CCA_H_
#define LINK_CCA_H_

/* INCLUDES *******************************************************************/
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* TYPES **********************************************************************/
/** @brief CCA fail action.
 */
typedef enum cca_fail_action {
    CCA_FAIL_ACTION_TX       = 0, /**< Transmit anyway */
    CCA_FAIL_ACTION_ABORT_TX = 1  /**< Abort transmission */
} cca_fail_action_t;

/** @brief Clear channel assessment information.
 */
typedef struct {
    uint8_t threshold;              /**< Clear channel threshold, valid values are between 0 and 47 */
    uint8_t max_try_count;          /**< Maximum number of failed CCA tries before taking the configured fail action */
    cca_fail_action_t fail_action;  /**< Action to take when all tries failed */
    uint16_t retry_time_pll_cycles; /**< RX pause time register value */
    uint8_t *fbk_try_count;         /**< Fallback try count array */
    bool enable;                    /**< Enable feature */
} link_cca_t;

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Initialize CCA object.
 *
 *  @param[in] cca                    CCA object.
 *  @param[in] threshold              CCA threshold.
 *  @param[in] retry_time_pll_cycles  CCA retry time.
 *  @param[in] on_time_pll_cycles     CCA On time.
 *  @param[in] max_try_count          CCA max try count.
 *  @param[in] fbk_try_count          CCA fallback try count array.
 *  @param[in] fail_action            CCA fail action.
 *  @param[in] enable                 CCA enable flag.
 */
void link_cca_init(link_cca_t *cca, uint8_t threshold, uint16_t retry_time_pll_cycles, uint16_t on_time_pll_cycles, uint8_t max_try_count,
                   uint8_t *fbk_try_count, cca_fail_action_t fail_action, bool enable);

#ifdef __cplusplus
}
#endif
#endif /* LINK_CCA_H_ */
