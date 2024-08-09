/** @file  sac_mixer_endpoint.h
 *  @brief SPARK Audio Core endpoint for the SPARK Audio Core Mixer Module.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
               Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SAC_MIXER_ENDPOINT_H_
#define SAC_MIXER_ENDPOINT_H_

/* INCLUDES *******************************************************************/
#include "sac_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Mixer Endpoint's Produce action.
 *
 *  @param[in]  instance  Endpoint instance.
 *  @param[out] samples   Produced samples.
 *  @param[in]  size      Size of samples to produce in bytes.
 *  @return Number of bytes produced.
 */
uint16_t ep_mixer_produce(void *instance, uint8_t *samples, uint16_t size);

/** @brief Mixer Endpoint's Consume action.
 *
 *  @param[in]  instance  Endpoint instance.
 *  @param[out] samples   Consumed samples.
 *  @param[in]  size      Size of samples to consume in bytes.
 *  @return Number of bytes consumed.
 */
uint16_t ep_mixer_consume(void *instance, uint8_t *samples, uint16_t size);

/** @brief Start the Mixer endpoint.
 *
 *  @param[in] instance  Endpoint instance.
 */
void ep_mixer_start(void *instance);

/** @brief Stop the Mixer endpoint.
 *
 *  @param[in] instance  Endpoint instance.
 */
void ep_mixer_stop(void *instance);

#ifdef __cplusplus
}
#endif

#endif /* SAC_MIXER_ENDPOINT_H_ */
