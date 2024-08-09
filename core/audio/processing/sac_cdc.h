/** @file  sac_cdc.h
 *  @brief Clock drift compensation processing stage using audio buffer load averaging for
 *         detecting the drift and interpolation for correcting it.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
 *             Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SAC_CDC_H_
#define SAC_CDC_H_

/* INCLUDES *******************************************************************/
#include "resampling.h"
#include "sac_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/* TYPES **********************************************************************/
/** @brief SPARK Audio Core commands.
 */
typedef enum sac_cdc_cmd {
    /*! Set the Clock Drift Compensation target queue size. */
    SAC_CDC_SET_TARGET_QUEUE_SIZE,
} sac_cdc_cmd_t;

/** @brief CDC Instance.
 */
typedef struct sac_cdc_instance {
    /*! Amount of samples used when resampling. */
    uint16_t cdc_resampling_length;
    /*! Amount of measurements used when averaging the consumer queue size. */
    uint16_t cdc_queue_avg_size;
    struct {
        /*! Internal: An instance of the resampling. */
        resampling_instance_t resampling_instance;
        /*! Internal: Number of bytes per audio sample. */
        uint8_t  size_of_buffer_type;
        /*! Internal: An circular array of tx queue lengths used for averaging. */
        uint16_t *avg_arr;
        /*! Internal: Rolling average of the avg_arr. */
        uint32_t avg_sum;
        /*! Internal: Normalized average of avg_sum to increase resolution. */
        uint32_t avg_val;
        /*! Internal: Used to ensure a minimum number of queue length samples before determining a resampling action. */
        uint32_t count;
        /*! Internal: Index into the avg_arr. */
        uint16_t avg_idx;
        /*! Internal: Trigger level to determine whether to take a resampling action. */
        uint16_t max_queue_offset;
        /*! Internal: Normalized queue size to increase trigger resolution. */
        uint32_t normal_queue_size;
        /*! Internal: Size of the averaging array avg_arr. */
        uint16_t queue_avg_size;
        /*!
         * Internal: Set due to feedback from audio source that its tx queue is full.
         * This will pause any resampling activity until the audio source tx queue has emptied.
         */
        bool wait_for_queue_full;
        /*! Internal: Number of samples in each audio payload to resample. */
        uint32_t sample_amount;
    } _internal;
} sac_cdc_instance_t;

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Initialize the CDC processing stage.
 *
 *  @param[in]  instance  CDC instance.
 *  @param[in]  pipeline  Pipeline instance.
 *  @param[in]  mem_pool  Memory pool handle.
 *  @param[out] err       Error code.
 */
void sac_cdc_init(void *instance, sac_pipeline_t *pipeline, mem_pool_t *mem_pool, sac_error_t *err);

/** @brief Control the CDC processing stage.
 *
 *  @param[in]  instance  CDC instance.
 *  @param[in]  pipeline  Pipeline instance.
 *  @param[in]  cmd       Command.
 *  @param[in]  args      Argument.
 *  @param[out] err       Error code.
 *
 *  @return Command specific value.
 */
uint32_t sac_cdc_ctrl(void *instance, sac_pipeline_t *pipeline, uint8_t cmd, uint32_t arg, sac_error_t *err);

/** @brief Process the CDC processing state.
 *
 *  This uses interpolation (resampling) in order to create or drop a sample
 *  to correct the audio clock drift.
 *
 *  @param[in]  instance  CDC instance.
 *  @param[in]  pipeline  Pipeline instance.
 *  @param[in]  header    Audio packet's header.
 *  @param[in]  data_in   Audio payload to process.
 *  @param[in]  size      Size in bytes of the audio payload.
 *  @param[out] data_out  Audio payload that has been processed.
 *  @param[out] err       Error code.
 *
 *  @return Size in bytes of the processed samples, 0 if no processing happened.
 */
uint16_t sac_cdc_process(void *instance, sac_pipeline_t *pipeline, sac_header_t *header, uint8_t *data_in, uint16_t size, uint8_t *data_out, sac_error_t *err);

#ifdef __cplusplus
}
#endif

#endif /* SAC_CDC_H_ */

