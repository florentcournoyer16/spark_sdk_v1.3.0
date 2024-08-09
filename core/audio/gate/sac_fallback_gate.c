/** @file  sac_fallback_gate.c
 *  @brief SPARK Audio Core Fallback gate is used to gate a processing stage based on the fallback state.
 *
 *  @copyright Copyright (C) 2023 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "sac_fallback_gate.h"
#include "sac_fallback_module.h"

/* CONSTANTS ******************************************************************/
/*! Fallback flag value when fallback is active. */
#define FALLBACK_ACTIVE   1
/*! Fallback flag value when fallback is inactive. */
#define FALLBACK_INACTIVE 0

/* PUBLIC FUNCTIONS ***********************************************************/
bool sac_fallback_gate_is_fallback_on(void *instance, sac_pipeline_t *pipeline, sac_header_t *header,
                                      uint8_t *data_in, uint16_t size, sac_error_t *err)
{
    (void)instance;
    (void)pipeline;
    (void)data_in;
    (void)size;

    *err = SAC_ERR_NONE;

    /* Update packet header. */
    header->fallback = sac_fallback_is_active();

    return (header->fallback == FALLBACK_ACTIVE);
}

bool sac_fallback_gate_is_fallback_off(void *instance, sac_pipeline_t *pipeline, sac_header_t *header,
                                       uint8_t *data_in, uint16_t size, sac_error_t *err)
{
    (void)instance;
    (void)pipeline;
    (void)data_in;
    (void)size;

    *err = SAC_ERR_NONE;

    /* Update packet header. */
    header->fallback = sac_fallback_is_active();

    return (header->fallback == FALLBACK_INACTIVE);
}

bool sac_fallback_gate_fallback_on_detect(void *instance, sac_pipeline_t *pipeline, sac_header_t *header,
                                          uint8_t *data_in, uint16_t size, sac_error_t *err)
{
    (void)instance;
    (void)pipeline;
    (void)data_in;
    (void)size;

    *err = SAC_ERR_NONE;

    /* Set the fallback module state. */
    if (header->fallback == FALLBACK_ACTIVE) {
        sac_fallback_set_fallback_flag();
    } else {
        sac_fallback_clear_fallback_flag();
    }

    return (header->fallback == FALLBACK_ACTIVE);
}

bool sac_fallback_gate_fallback_off_detect(void *instance, sac_pipeline_t *pipeline, sac_header_t *header,
                                           uint8_t *data_in, uint16_t size, sac_error_t *err)
{
    (void)instance;
    (void)pipeline;
    (void)data_in;
    (void)size;

    *err = SAC_ERR_NONE;

    /* Set the fallback module state. */
    if (header->fallback == FALLBACK_ACTIVE) {
        sac_fallback_set_fallback_flag();
    } else {
        sac_fallback_clear_fallback_flag();
    }

    return (header->fallback == FALLBACK_INACTIVE);
}
