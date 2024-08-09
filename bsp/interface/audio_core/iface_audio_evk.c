/** @file  iface_audio_evk.c
 *  @brief This file contains the implementation of functions configuring the
 *         audio core which calls the functions of the BSP of the EVK1.4.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */

/* INCLUDES *******************************************************************/
#include "iface_audio.h"
#include "evk.h"
#include "sac_api.h"
#include "swc_api.h"

/* PRIVATE FUNCTION PROTOTYPES ************************************************/
static uint16_t ep_max98091_action_produce(void *instance, uint8_t *samples, uint16_t size);
static void ep_max98091_start_produce(void *instance);
static void ep_max98091_stop_produce(void *instance);
static uint16_t ep_max98091_action_consume(void *instance, uint8_t *samples, uint16_t size);
static void ep_max98091_start_consume(void *instance);
static void ep_max98091_stop_consume(void *instance);
static uint16_t ep_swc_action_produce(void *instance, uint8_t *samples, uint16_t size);
static uint16_t ep_swc_action_consume(void *instance, uint8_t *samples, uint16_t size);
static void ep_swc_start(void *instance);
static void ep_swc_stop(void *instance);

/* PUBLIC FUNCTIONS ***********************************************************/
void iface_sac_hal_init(sac_hal_t *hal)
{
    hal->enter_critical = evk_enter_critical;
    hal->exit_critical = evk_exit_critical;
}

void iface_audio_swc_endpoint_init(sac_endpoint_interface_t *swc_producer_iface,
                                   sac_endpoint_interface_t *swc_consumer_iface)
{
    if (swc_producer_iface != NULL) {
        swc_producer_iface->action = ep_swc_action_produce;
        swc_producer_iface->start = ep_swc_start;
        swc_producer_iface->stop = ep_swc_stop;
    }

    if (swc_consumer_iface != NULL) {
        swc_consumer_iface->action = ep_swc_action_consume;
        swc_consumer_iface->start = ep_swc_start;
        swc_consumer_iface->stop = ep_swc_stop;
    }
}

void iface_audio_max98091_endpoint_init(sac_endpoint_interface_t *max98091_producer_iface,
                                        sac_endpoint_interface_t *max98091_consumer_iface)
{
    if (max98091_producer_iface != NULL) {
        max98091_producer_iface->action = ep_max98091_action_produce;
        max98091_producer_iface->start = ep_max98091_start_produce;
        max98091_producer_iface->stop = ep_max98091_stop_produce;
    }

    if (max98091_consumer_iface != NULL) {
        max98091_consumer_iface->action = ep_max98091_action_consume;
        max98091_consumer_iface->start = ep_max98091_start_consume;
        max98091_consumer_iface->stop = ep_max98091_stop_consume;
    }
}

/* PRIVATE FUNCTIONS **********************************************************/
/** @brief Produce Endpoint of the audio codec
 *
 *  @param[in]  instance  Endpoint instance (not used).
 *  @param[out] samples   Location to put produced samples.
 *  @param[in]  size      Size of samples to produce in bytes.
 *  @return Number of bytes produced (always 0 since production is delayed).
 */
static uint16_t ep_max98091_action_produce(void *instance, uint8_t *samples, uint16_t size)
{
    (void)instance;

    evk_audio_sai_read_non_blocking(samples, size);

    return 0;
}

/** @brief Start the endpoint when used as a producer.
 *
 *  @param[in] instance  Endpoint instance (not used).
 */
static void ep_max98091_start_produce(void *instance)
{
    (void)instance;

    evk_audio_sai_start_read_non_blocking();
}

/** @brief Stop the endpoint when used as a producer.
 *
 *  @param[in] instance  Endpoint instance (not used).
 */
static void ep_max98091_stop_produce(void *instance)
{
    (void)instance;

    evk_audio_sai_stop_read_non_blocking();
}

/** @brief Consume Endpoint of the audio codec
 *
 *  @param[in] instance  Endpoint instance (not used).
 *  @param[in] samples   Samples to consume.
 *  @param[in] size      Size of samples to consume in bytes.
 *  @return Number of bytes consumed (always 0 since consumption is delayed).
 */
static uint16_t ep_max98091_action_consume(void *instance, uint8_t *samples, uint16_t size)
{
    (void)instance;

    evk_audio_sai_write_non_blocking(samples, size);

    return 0;
}

/** @brief Start the endpoint when used as a consumer.
 *
 *  @param[in] instance  Endpoint instance (not used).
 */
static void ep_max98091_start_consume(void *instance)
{
    (void)instance;

    evk_audio_sai_start_write_non_blocking();
}

/** @brief Stop the endpoint when used as a consumer.
 *
 *  @param[in] instance  Endpoint instance (not used).
 */
static void ep_max98091_stop_consume(void *instance)
{
    (void)instance;

    evk_audio_sai_stop_write_non_blocking();
}

/** @brief Produce Endpoint of the SPARK Wireless Core.
 *
 *  @param[in]  instance  Endpoint instance.
 *  @param[out] samples   Produced samples.
 *  @param[in]  size      Size of samples to produce in bytes.
 *  @return Number of bytes produced.
 */
static uint16_t ep_swc_action_produce(void *instance, uint8_t *samples, uint16_t size)
{
    uint8_t *payload = NULL;
    uint8_t  payload_size;
    swc_error_t err;
    ep_swc_instance_t *inst = (ep_swc_instance_t *)instance;
    (void)size;

    payload_size = swc_connection_receive(inst->connection, &payload, &err);

    memcpy(samples, payload, payload_size);
    swc_connection_receive_complete(inst->connection, &err);

    return payload_size;
}

/** @brief Consume Endpoint of the SPARK Wireless Core.
 *
 *  @param[in] instance  Endpoint instance.
 *  @param[in] samples   Samples to consume.
 *  @param[in] size      Size of samples to consume in bytes.
 *  @return Number of bytes consumed.
 */
static uint16_t ep_swc_action_consume(void *instance, uint8_t *samples, uint16_t size)
{
    uint8_t *buf;
    swc_error_t err;
    ep_swc_instance_t *inst = (ep_swc_instance_t *)instance;

    swc_connection_get_payload_buffer(inst->connection, &buf, &err);
    if (buf != NULL) {
        memcpy(buf, samples, size);
        swc_connection_send(inst->connection, buf, size, &err);
        return size;
    } else {
        return 0;
    }
}

/** @brief Start the endpoint.
 *
 *  @param[in] instance  Endpoint instance.
 */
static void ep_swc_start(void *instance)
{
    (void)instance;
}

/** @brief Stop the endpoint.
 *
 *  @param[in] instance  Endpoint instance.
 */
static void ep_swc_stop(void *instance)
{
    (void)instance;
}
