/** @file link_cca.c
 *  @brief Clear Channel Assessment module.
 *
 *  @copyright Copyright (C) 2021 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "link_cca.h"

/* PUBLIC FUNCTIONS ***********************************************************/
void link_cca_init(link_cca_t *cca, uint8_t threshold, uint16_t retry_time_pll_cycles, uint16_t on_time_pll_cycles, uint8_t max_try_count,
                   uint8_t *fbk_try_count, cca_fail_action_t fail_action, bool enable)
{
    (void)on_time_pll_cycles;

    cca->enable                = enable;
    cca->threshold             = threshold;
    cca->retry_time_pll_cycles = retry_time_pll_cycles;
    cca->max_try_count         = max_try_count;
    cca->fail_action           = fail_action;
    cca->fbk_try_count         = fbk_try_count;
}
