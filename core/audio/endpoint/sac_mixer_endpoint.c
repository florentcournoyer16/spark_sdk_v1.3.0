/** @file  sac_mixer_endpoint.c
 *  @brief SPARK Audio Core endpoint for the SPARK Audio Core Mixer Module.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
               Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "sac_mixer_endpoint.h"

/* PUBLIC FUNCTIONS ***********************************************************/
uint16_t ep_mixer_produce(void *instance, uint8_t *samples, uint16_t size)
{
    (void)instance;
    (void)samples;
    (void)size;

    return 0;
}

uint16_t ep_mixer_consume(void *instance, uint8_t *samples, uint16_t size)
{
    (void)instance;
    (void)samples;
    (void)size;

    return 0;
}

void ep_mixer_start(void *instance)
{
    (void)instance;
}

void ep_mixer_stop(void *instance)
{
    (void)instance;
}
