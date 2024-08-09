/** @file  sac_cdc_clock_steering.h
 *  @brief Clock drift compensation processing stage using audio buffer load averaging for
 *         detecting the drift and clock steering for correcting it.
 *
 *  @copyright Copyright (C) 2022 SPARK Microsystems International Inc. All rights reserved.
 *  @license   This source code is proprietary and subject to the SPARK Microsystems
               Software EULA found in this package in file EULA.txt.
 *  @author    SPARK FW Team.
 */
#ifndef SAC_CDC_CLOCK_STEERING_H_
#define SAC_CDC_CLOCK_STEERING_H_

/* INCLUDES *******************************************************************/
#include "sac_api.h"

#ifdef __cplusplus
extern "C" {
#endif

/* TYPES **********************************************************************/
/** @brief CDC Correction Modes.
 */
typedef enum sac_cdc_correction {
    SAC_CDC_CORRECTION_NONE,
    SAC_CDC_CORRECTION_ADD,
    SAC_CDC_CORRECTION_REMOVE,
} sac_cdc_correction_t;

/** @brief CDC States.
 */
typedef enum sac_cdc_state {
    SAC_CDC_STATE_WAIT_QUEUE_FULL,
    SAC_CDC_STATE_IDLE,
    SAC_CDC_STATE_START,
    SAC_CDC_STATE_RUNNING
} sac_cdc_state_t;

/** @brief CDC Instance.
 */
typedef struct sac_cdc_clock_steering_instance {
    /*! Detection resolution in number of samples. */
    uint8_t cdc_detect_resolution;
    /*! Amount of measurements used when averaging the consumer queue size. */
    uint16_t cdc_queue_avg_size;
    /*! Function that increases the audio sampling frequency by a fraction (PLL value += 1 ). */
    void (*cdc_increase_clock)(void);
    /*! Function that decreases the audio sampling frequency by a fraction (PLL value -= 1 ). */
    void (*cdc_decrease_clock)(void);
    struct {
        /*! Internal: Current state of the CDC processing stage. */
        sac_cdc_state_t state;
        /*! Internal: Correction requested from CDC detection result. */
        sac_cdc_correction_t correction;
        /*! Internal: Number of bytes per audio sample. */
        uint8_t  size_of_buffer_type;
        /*! Internal: An circular array of TX queue lengths used for averaging. */
        uint16_t *avg_arr;
        /*! Internal: Rolling average of the avg_arr. */
        uint32_t avg_sum;
        /*! Internal: Normalized average of avg_sum to increase resolution. */
        uint32_t avg_val;
        /*! Internal: Used to ensure a minimum number of queue length samples before determining a resampling action. */
        uint32_t count;
        /*! Internal: Index into the avg_arr. */
        uint16_t avg_idx;
        /*! Internal: Trigger level to determine whether to take an action. */
        uint16_t max_queue_offset;
        /*! Internal: Normalized queue size to increase trigger resolution. */
        uint32_t normal_queue_size;
        /*! Internal: Size of the averaging array avg_arr. */
        uint16_t queue_avg_size;
        /*!
         * Internal: Set due to feedback from audio source that its TX queue is full.
         * This will pause any compensation until the audio source TX queue has emptied.
         */
        bool wait_for_queue_full;
        /* Internal: Number of samples in each audio payload to process. */
        uint32_t sample_amount;
        /*! Internal: Used by the clock steering mechanism to track the clock offset. */
        uint8_t current_offset;
        /*! Internal: Used by the clock steering mechanism to track the clock offset. */
        bool return_to_original_clock;
    } _internal;
} sac_cdc_clock_steering_instance_t;

/* PUBLIC FUNCTION PROTOTYPES *************************************************/
/** @brief Initialize the CDC processing stage.
 *
 *  @param[in]  instance  CDC instance.
 *  @param[in]  pipeline  Pipeline instance.
 *  @param[in]  mem_pool  Memory pool handle.
 *  @param[out] err       Error code.
 */
void sac_cdc_clock_steering_init(void *instance, sac_pipeline_t *pipeline, mem_pool_t *mem_pool, sac_error_t *err);

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
uint32_t sac_cdc_clock_steering_ctrl(void *instance, sac_pipeline_t *pipeline, uint8_t cmd, uint32_t arg, sac_error_t *err);

/** @brief Process the CDC processing stage.
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
uint16_t sac_cdc_process_clock_steering(void *instance, sac_pipeline_t *pipeline, sac_header_t *header, uint8_t *data_in,
                                        uint16_t size, uint8_t *data_out, sac_error_t *err);

#ifdef __cplusplus
}
#endif

#endif /* SAC_CDC_CLOCK_STEERING_H_ */

